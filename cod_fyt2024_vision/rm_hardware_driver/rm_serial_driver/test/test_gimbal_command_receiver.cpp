#include "rm_serial_driver/gimbal_command_receiver.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>

namespace {

int failures = 0;

void check(const bool condition, const char *what) {
  if (!condition) {
    std::printf("FAIL: %s\n", what);
    ++failures;
  }
}

}  // namespace

int main() {
  using fyt::serial_driver::FixedPacket;
  using fyt::serial_driver::GimbalActuatorCommand;
  using fyt::serial_driver::GimbalCommandReceiver;
  using fyt::serial_driver::GimbalPacketCommand;
  using fyt::serial_driver::GimbalPacketMode;
  using fyt::serial_driver::encodeGimbalPacketV3;

  GimbalCommandReceiver receiver;
  GimbalActuatorCommand command;

  // ── Legacy 16-byte position packet, still used by the rune solver ────────
  FixedPacket<16> legacy;
  legacy.loadData<std::uint8_t>(1U, 1);  // fire
  legacy.loadData<float>(-2.5F, 2);      // pitch
  legacy.loadData<float>(14.0F, 6);      // yaw
  legacy.loadData<float>(3.2F, 10);      // distance
  check(receiver.receiveLegacyPosition(legacy, command), "legacy frame decodes");
  check(command.control, "legacy frame drives the gimbal");
  check(command.fire, "legacy fire bit maps to fire");
  check(command.mode == GimbalPacketMode::kFire, "legacy fire maps to fire mode");
  check(command.pitch == -2.5F && command.yaw == 14.0F && command.distance == 3.2F,
        "legacy fields round trip");

  legacy.loadData<std::uint8_t>(0U, 1);
  check(receiver.receiveLegacyPosition(legacy, command), "legacy non-fire frame decodes");
  check(!command.fire && command.control, "legacy non-fire maps to control mode");

  // ── Versioned feed-forward packet ───────────────────────────────────────
  GimbalPacketCommand source;
  source.mode = GimbalPacketMode::kFire;
  source.pitch = -1.0F;
  source.yaw = 2.0F;
  source.pitch_vel = -30.0F;
  source.pitch_acc = 120.0F;
  source.yaw_vel = 250.0F;
  // 50 rad/s^2 expressed in degrees: the largest the planner can command.
  source.yaw_acc = 2864.789F;

  FixedPacket<32> packet;
  check(encodeGimbalPacketV3(source, packet), "feed-forward frame encodes");
  check(receiver.receiveVersioned(packet, command), "feed-forward frame decodes");
  check(command.mode == GimbalPacketMode::kFire, "fire mode round trips");
  check(command.control && command.fire, "fire mode drives the gimbal and fires");
  check(command.yaw_acc == source.yaw_acc, "acceleration feed-forward round trips");
  check(command.yaw_vel == source.yaw_vel, "velocity feed-forward round trips");

  source.mode = GimbalPacketMode::kControl;
  check(encodeGimbalPacketV3(source, packet), "control frame encodes");
  check(receiver.receiveVersioned(packet, command), "control frame decodes");
  check(command.control && !command.fire, "control mode drives without firing");

  // An idle frame must be distinguishable from "aim at 0 degrees". This is what
  // replaces the old behaviour of commanding yaw=0 when the target is lost.
  source.mode = GimbalPacketMode::kIdle;
  source.yaw = 0.0F;
  source.pitch = 0.0F;
  check(encodeGimbalPacketV3(source, packet), "idle frame encodes");
  check(receiver.receiveVersioned(packet, command), "idle frame decodes");
  check(!command.control && !command.fire, "idle means do not drive the gimbal");

  // ── Rejects ─────────────────────────────────────────────────────────────
  source.mode = GimbalPacketMode::kControl;
  check(encodeGimbalPacketV3(source, packet), "control frame re-encodes");

  std::array<std::uint8_t, 32> malformed{};
  std::copy(packet.buffer(), packet.buffer() + malformed.size(), malformed.begin());
  malformed[0] = 0x00U;  // break the start byte
  FixedPacket<32> broken;
  broken.copyFrom(malformed.data());
  check(!receiver.receiveVersioned(broken, command), "bad start byte is rejected");

  check(encodeGimbalPacketV3(source, packet), "control frame re-encodes");
  packet.loadData<float>(std::numeric_limits<float>::quiet_NaN(), 8);
  check(!receiver.receiveVersioned(packet, command), "NaN payload is rejected");

  check(encodeGimbalPacketV3(source, packet), "control frame re-encodes");
  packet.loadData<std::uint8_t>(7U, 2);  // out-of-range mode, CRC no longer matches either
  check(!receiver.receiveVersioned(packet, command), "out-of-range mode is rejected");

  // ── Feed-forward consistency ────────────────────────────────────────────
  // The three transmitted fields must describe one and the same trajectory. For
  // a constant-acceleration profile starting from rest they satisfy
  // vel^2 == 2 * acc * pos, which is independent of any integration scheme, so
  // a mismatch means the fields were swapped or scaled wrongly on the wire.
  constexpr int kSteps = 200;
  constexpr double kDt = 0.01;
  constexpr double kAcc = 10.0;  // inside the planner's 50 rad/s^2 bound
  double worst_residual = 0.0;
  for (int step = 0; step < kSteps; ++step) {
    const double t = step * kDt;
    GimbalPacketCommand frame;
    frame.mode = GimbalPacketMode::kControl;
    frame.yaw = static_cast<float>(0.5 * kAcc * t * t);
    frame.yaw_vel = static_cast<float>(kAcc * t);
    frame.yaw_acc = static_cast<float>(kAcc);
    assert(encodeGimbalPacketV3(frame, packet));
    assert(receiver.receiveVersioned(packet, command));

    const double residual = static_cast<double>(command.yaw_vel) * command.yaw_vel -
                            2.0 * command.yaw_acc * command.yaw;
    worst_residual = std::max(worst_residual, std::abs(residual));
  }
  std::printf("     feed-forward consistency residual = %.3e\n", worst_residual);
  check(worst_residual < 1e-2,
        "position, velocity and acceleration describe the same trajectory");

  std::printf("%s (%d failure(s))\n", failures == 0 ? "PASS" : "FAIL", failures);
  return failures == 0 ? 0 : 1;
}
