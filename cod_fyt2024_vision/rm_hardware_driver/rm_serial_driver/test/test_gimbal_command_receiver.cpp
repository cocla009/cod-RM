#include "rm_serial_driver/gimbal_command_receiver.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>

int main() {
  using fyt::serial_driver::FixedPacket;
  using fyt::serial_driver::GimbalActuatorCommand;
  using fyt::serial_driver::GimbalCommandReceiver;
  using fyt::serial_driver::GimbalPacketCommand;
  using fyt::serial_driver::GimbalPacketMode;
  using fyt::serial_driver::encodeGimbalPacketV2;

  GimbalCommandReceiver receiver;
  GimbalActuatorCommand command;
  FixedPacket<16> legacy;
  legacy.loadData<std::uint8_t>(1U, 1);
  legacy.loadData<float>(-2.5F, 2);
  legacy.loadData<float>(14.0F, 6);
  legacy.loadData<float>(3.2F, 10);
  assert(receiver.receiveLegacyPosition(legacy, command));
  assert(command.mode == GimbalPacketMode::kPosition);
  assert(command.fire);
  assert(command.pitch == -2.5F);
  assert(command.yaw == 14.0F);
  assert(command.distance == 3.2F);

  GimbalPacketCommand source;
  source.mode = GimbalPacketMode::kJerk;
  source.pitch = -1.0F;
  source.yaw = 2.0F;
  source.yaw_jerk = 100.0F;
  source.pitch_jerk = -40.0F;
  source.command_dt = 0.01F;
  source.sequence = 100U;
  FixedPacket<32> packet;
  assert(encodeGimbalPacketV2(source, packet));
  assert(receiver.receiveVersioned(packet, command));
  assert(command.mode == GimbalPacketMode::kJerk);
  assert(command.sequence == 100U);
  assert(command.yaw_jerk == 100.0F);
  assert(!receiver.receiveVersioned(packet, command));

  source.sequence = 99U;
  assert(encodeGimbalPacketV2(source, packet));
  assert(!receiver.receiveVersioned(packet, command));
  source.sequence = 101U;
  assert(encodeGimbalPacketV2(source, packet));
  assert(receiver.receiveVersioned(packet, command));

  GimbalCommandReceiver wrap_receiver;
  source.sequence = std::numeric_limits<std::uint32_t>::max();
  assert(encodeGimbalPacketV2(source, packet));
  assert(wrap_receiver.receiveVersioned(packet, command));
  source.sequence = 0U;
  assert(encodeGimbalPacketV2(source, packet));
  assert(wrap_receiver.receiveVersioned(packet, command));

  source.command_dt = 1.0F;
  assert(encodeGimbalPacketV2(source, packet));
  assert(!receiver.receiveVersioned(packet, command));
  source.command_dt = 0.01F;
  assert(encodeGimbalPacketV2(source, packet));
  std::array<std::uint8_t, 32> malformed{};
  std::copy(packet.buffer(), packet.buffer() + malformed.size(), malformed.begin());
  malformed[0] = 0x00U;
  packet.copyFrom(malformed.data());
  assert(!receiver.receiveVersioned(packet, command));

  source.sequence = 102U;
  assert(encodeGimbalPacketV2(source, packet));
  packet.loadData<float>(std::numeric_limits<float>::quiet_NaN(), 8);
  assert(!receiver.receiveVersioned(packet, command));

  receiver.resetSequence();
  double yaw = 0.0;
  double yaw_velocity = 0.0;
  double yaw_acceleration = 0.0;
  constexpr double kDt = 0.01;
  constexpr double kMaxAcceleration = 5.0;
  for (std::uint32_t frame = 0; frame < 100U; ++frame) {
    source.sequence = frame;
    source.command_dt = static_cast<float>(kDt);
    source.yaw_jerk = static_cast<float>(20.0 * std::sin(0.1 * frame));
    assert(encodeGimbalPacketV2(source, packet));
    assert(receiver.receiveVersioned(packet, command));
    const double jerk = command.yaw_jerk;
    yaw += kDt * yaw_velocity + 0.5 * kDt * kDt * yaw_acceleration +
           kDt * kDt * kDt * jerk / 6.0;
    yaw_velocity += kDt * yaw_acceleration + 0.5 * kDt * kDt * jerk;
    yaw_acceleration += kDt * jerk;
    assert(std::isfinite(yaw));
    assert(std::isfinite(yaw_velocity));
    assert(std::isfinite(yaw_acceleration));
    assert(std::abs(yaw_acceleration) <= kMaxAcceleration);
  }
  return 0;
}
