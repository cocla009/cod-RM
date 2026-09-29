// Versioned gimbal command packet, second generation on the wire (v3).
//
// Carries position plus velocity and acceleration feed-forward so the lower
// controller can track a planned trajectory instead of chasing a position set
// point. The field set mirrors sp_vision_25's VisionToGimbal struct; the
// framing is ours (FixedPacket start/end bytes) and the units are degrees, to
// match the legacy 16-byte packet that the rune solver still uses.
//
// This replaces the v2 "jerk" packet. The magic byte changed from 0xA2 to 0xA3
// so firmware built against v2 rejects these frames instead of misreading the
// field layout.
//
//   offset  0      0xFF            FixedPacket start
//   offset  1      0xA3            protocol magic (v3)
//   offset  2      mode            0 = idle, 1 = control, 2 = control + fire
//   offset  3      reserved        0
//   offset  4-7    pitch           float, degrees
//   offset  8-11   yaw             float, degrees
//   offset 12-15   pitch_vel       float, degrees/s
//   offset 16-19   pitch_acc       float, degrees/s^2
//   offset 20-23   yaw_vel         float, degrees/s
//   offset 24-27   yaw_acc         float, degrees/s^2
//   offset 28-29   crc16           uint16 little-endian over bytes 1..27
//   offset 30      check byte      FixedPacket, unused (0)
//   offset 31      0x0D            FixedPacket end
//
// Fire is expressed by mode == 2; there is no separate fire byte.

#ifndef RM_SERIAL_DRIVER_GIMBAL_PACKET_V3_HPP_
#define RM_SERIAL_DRIVER_GIMBAL_PACKET_V3_HPP_

#include <cmath>
#include <cstdint>

#include "rm_serial_driver/crc16.hpp"
#include "rm_serial_driver/fixed_packet.hpp"

namespace fyt::serial_driver {

constexpr std::uint8_t kGimbalPacketV3Magic = 0xA3;
constexpr int kGimbalPacketV3CrcOffset = 28;
constexpr int kGimbalPacketV3CrcCoveredBytes = 27;  // offsets 1..27 inclusive

// In-packet control state, following the reference implementation.
enum class GimbalPacketMode : std::uint8_t {
  kIdle = 0,     // do not drive the gimbal
  kControl = 1,  // drive the gimbal, do not fire
  kFire = 2,     // drive the gimbal and fire
};

// Value carried in rm_interfaces::msg::GimbalCmd::control_mode to select the
// wire format. armor_solver cannot include this header (it does not depend on
// rm_serial_driver), so its gimbal_command_contract.hpp repeats these numbers
// and must stay in sync.
enum class GimbalCommandFormat : std::uint8_t {
  kPosition = 0,     // legacy 16-byte position packet, used by the rune solver
  kFeedforward = 1,  // this packet
};

// Maps a published command onto the in-packet control state. A command without
// a valid plan becomes idle, so a planner that cannot produce a trajectory
// releases the gimbal instead of holding a stale angle.
inline GimbalPacketMode commandMode(const bool mpc_valid, const bool fire_advice) noexcept {
  if (!mpc_valid) return GimbalPacketMode::kIdle;
  return fire_advice ? GimbalPacketMode::kFire : GimbalPacketMode::kControl;
}

struct GimbalPacketCommand {
  GimbalPacketMode mode = GimbalPacketMode::kIdle;
  float pitch = 0.0F;      // degrees
  float yaw = 0.0F;        // degrees
  float pitch_vel = 0.0F;  // degrees/s
  float pitch_acc = 0.0F;  // degrees/s^2
  float yaw_vel = 0.0F;    // degrees/s
  float yaw_acc = 0.0F;    // degrees/s^2
};

inline bool validCommand(const GimbalPacketCommand &command) noexcept {
  return static_cast<std::uint8_t>(command.mode) <=
           static_cast<std::uint8_t>(GimbalPacketMode::kFire) &&
         std::isfinite(command.pitch) && std::isfinite(command.yaw) &&
         std::isfinite(command.pitch_vel) && std::isfinite(command.pitch_acc) &&
         std::isfinite(command.yaw_vel) && std::isfinite(command.yaw_acc);
}

inline bool encodeGimbalPacketV3(const GimbalPacketCommand &command,
                                 FixedPacket<32> &packet) noexcept {
  if (!validCommand(command)) return false;

  packet.clear();
  packet.loadData<std::uint8_t>(kGimbalPacketV3Magic, 1);
  packet.loadData<std::uint8_t>(static_cast<std::uint8_t>(command.mode), 2);
  packet.loadData<std::uint8_t>(0U, 3);
  packet.loadData<float>(command.pitch, 4);
  packet.loadData<float>(command.yaw, 8);
  packet.loadData<float>(command.pitch_vel, 12);
  packet.loadData<float>(command.pitch_acc, 16);
  packet.loadData<float>(command.yaw_vel, 20);
  packet.loadData<float>(command.yaw_acc, 24);

  const std::uint16_t crc =
    crc16(packet.buffer() + 1, kGimbalPacketV3CrcCoveredBytes);
  packet.loadData<std::uint16_t>(crc, kGimbalPacketV3CrcOffset);
  packet.setCheckByte(0);
  return true;
}

inline bool decodeGimbalPacketV3(const FixedPacket<32> &packet,
                                 GimbalPacketCommand &command) noexcept {
  const std::uint8_t *buffer = packet.buffer();
  if (buffer == nullptr || buffer[0] != 0xFFU || buffer[31] != 0x0DU) return false;

  std::uint8_t magic = 0;
  std::uint8_t mode = 0;
  std::uint8_t reserved = 0;
  if (!packet.unloadData<std::uint8_t>(magic, 1) || magic != kGimbalPacketV3Magic ||
      !packet.unloadData<std::uint8_t>(mode, 2) ||
      mode > static_cast<std::uint8_t>(GimbalPacketMode::kFire) ||
      !packet.unloadData<std::uint8_t>(reserved, 3)) {
    return false;
  }
  (void)reserved;

  GimbalPacketCommand decoded;
  decoded.mode = static_cast<GimbalPacketMode>(mode);
  if (!packet.unloadData<float>(decoded.pitch, 4) ||
      !packet.unloadData<float>(decoded.yaw, 8) ||
      !packet.unloadData<float>(decoded.pitch_vel, 12) ||
      !packet.unloadData<float>(decoded.pitch_acc, 16) ||
      !packet.unloadData<float>(decoded.yaw_vel, 20) ||
      !packet.unloadData<float>(decoded.yaw_acc, 24) || !validCommand(decoded)) {
    return false;
  }

  std::uint16_t stored_crc = 0;
  if (!packet.unloadData<std::uint16_t>(stored_crc, kGimbalPacketV3CrcOffset)) return false;
  if (crc16(buffer + 1, kGimbalPacketV3CrcCoveredBytes) != stored_crc) return false;

  command = decoded;
  return true;
}

}  // namespace fyt::serial_driver

#endif  // RM_SERIAL_DRIVER_GIMBAL_PACKET_V3_HPP_
