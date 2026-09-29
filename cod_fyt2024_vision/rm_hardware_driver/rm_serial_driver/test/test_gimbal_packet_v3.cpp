#include "rm_serial_driver/crc16.hpp"
#include "rm_serial_driver/gimbal_packet_v3.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdint>
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
  using fyt::serial_driver::GimbalCommandFormat;
  using fyt::serial_driver::GimbalPacketCommand;
  using fyt::serial_driver::GimbalPacketMode;
  using fyt::serial_driver::crc16;
  using fyt::serial_driver::decodeGimbalPacketV3;
  using fyt::serial_driver::encodeGimbalPacketV3;

  // ── The CRC variant is pinned to its standard check value ───────────────
  // CRC-16/MCRF4XX (poly 0x1021 reflected, init 0xFFFF, no final xor). The
  // reference implementation uses the same variant; mixing it up with
  // CRC-16/MODBUS would break every frame.
  const char *check_string = "123456789";
  check(crc16(reinterpret_cast<const std::uint8_t *>(check_string), 9) == 0x6F91,
        "crc16 matches the CRC-16/MCRF4XX check value 0x6F91");

  // ── Round trip ──────────────────────────────────────────────────────────
  GimbalPacketCommand source;
  source.mode = GimbalPacketMode::kFire;
  source.pitch = -3.25F;
  source.yaw = 21.5F;
  source.pitch_vel = -12.5F;
  source.pitch_acc = 130.0F;
  source.yaw_vel = 250.0F;
  source.yaw_acc = -900.5F;

  FixedPacket<32> packet;
  check(encodeGimbalPacketV3(source, packet), "encode accepts a valid command");

  GimbalPacketCommand decoded;
  check(decodeGimbalPacketV3(packet, decoded), "decode accepts an encoded packet");
  check(decoded.mode == source.mode, "mode round trips");
  check(decoded.pitch == source.pitch, "pitch round trips");
  check(decoded.yaw == source.yaw, "yaw round trips");
  check(decoded.pitch_vel == source.pitch_vel, "pitch_vel round trips");
  check(decoded.pitch_acc == source.pitch_acc, "pitch_acc round trips");
  check(decoded.yaw_vel == source.yaw_vel, "yaw_vel round trips");
  check(decoded.yaw_acc == source.yaw_acc, "yaw_acc round trips");

  // ── Frame format ────────────────────────────────────────────────────────
  const std::uint8_t *buffer = packet.buffer();
  check(buffer[0] == 0xFFU, "start byte is 0xFF");
  check(buffer[1] == 0xA3U, "magic is 0xA3");
  check(buffer[31] == 0x0DU, "end byte is 0x0D");

  // ── CRC rejects corruption ──────────────────────────────────────────────
  FixedPacket<32> corrupted = packet;
  corrupted.loadData<float>(source.yaw + 1.0F, 8);  // flip a payload bit pattern
  check(!decodeGimbalPacketV3(corrupted, decoded), "corrupted payload fails the CRC");

  FixedPacket<32> bad_crc = packet;
  std::uint16_t stored = 0;
  bad_crc.unloadData<std::uint16_t>(stored, 28);
  bad_crc.loadData<std::uint16_t>(static_cast<std::uint16_t>(stored ^ 0x0001U), 28);
  check(!decodeGimbalPacketV3(bad_crc, decoded), "wrong CRC value is rejected");

  // ── A v2 frame must not be mistaken for v3 ──────────────────────────────
  FixedPacket<32> v2_frame = packet;
  v2_frame.loadData<std::uint8_t>(0xA2U, 1);
  check(!decodeGimbalPacketV3(v2_frame, decoded), "a 0xA2 (v2) frame is rejected");

  // ── Field validation ────────────────────────────────────────────────────
  GimbalPacketCommand bad_mode = source;
  bad_mode.mode = static_cast<GimbalPacketMode>(3);
  FixedPacket<32> scratch;
  check(!encodeGimbalPacketV3(bad_mode, scratch), "encode rejects an unknown mode");

  GimbalPacketCommand nan_command = source;
  nan_command.yaw = std::numeric_limits<float>::quiet_NaN();
  check(!encodeGimbalPacketV3(nan_command, scratch), "encode rejects a NaN field");

  GimbalPacketCommand inf_command = source;
  inf_command.yaw_acc = std::numeric_limits<float>::infinity();
  check(!encodeGimbalPacketV3(inf_command, scratch), "encode rejects an infinite field");

  // ── Idle / control / fire are distinct ──────────────────────────────────
  check(static_cast<std::uint8_t>(GimbalPacketMode::kIdle) == 0U, "idle mode is 0");
  check(static_cast<std::uint8_t>(GimbalPacketMode::kControl) == 1U, "control mode is 1");
  check(static_cast<std::uint8_t>(GimbalPacketMode::kFire) == 2U, "fire mode is 2");

  // ── Command mapping ─────────────────────────────────────────────────────
  // An invalid plan must release the gimbal, never fire.
  check(fyt::serial_driver::commandMode(false, false) == GimbalPacketMode::kIdle,
        "no valid plan maps to idle even without a fire request");
  check(fyt::serial_driver::commandMode(false, true) == GimbalPacketMode::kIdle,
        "no valid plan maps to idle even with a fire request");
  check(fyt::serial_driver::commandMode(true, false) == GimbalPacketMode::kControl,
        "a valid plan without a fire request maps to control");
  check(fyt::serial_driver::commandMode(true, true) == GimbalPacketMode::kFire,
        "a valid plan with a fire request maps to fire");
  check(static_cast<std::uint8_t>(GimbalCommandFormat::kPosition) == 0U,
        "position format selector is 0");
  check(static_cast<std::uint8_t>(GimbalCommandFormat::kFeedforward) == 1U,
        "feedforward format selector is 1");

  GimbalPacketCommand idle;
  idle.mode = GimbalPacketMode::kIdle;
  check(encodeGimbalPacketV3(idle, packet), "encode accepts an all-zero idle command");
  check(decodeGimbalPacketV3(packet, decoded) && decoded.mode == GimbalPacketMode::kIdle,
        "idle round trips");

  std::printf("%s (%d failure(s))\n", failures == 0 ? "PASS" : "FAIL", failures);
  return failures == 0 ? 0 : 1;
}
