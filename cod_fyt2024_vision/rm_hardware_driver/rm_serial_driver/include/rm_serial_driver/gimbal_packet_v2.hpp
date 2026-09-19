#ifndef SERIAL_DRIVER_GIMBAL_PACKET_V2_HPP_
#define SERIAL_DRIVER_GIMBAL_PACKET_V2_HPP_

#include <cmath>
#include <cstdint>

#include "rm_serial_driver/fixed_packet.hpp"

namespace fyt::serial_driver {

constexpr std::uint8_t kGimbalPacketV2Magic = 0xA2;

enum class GimbalPacketMode : std::uint8_t {
  kPosition = 0,
  kJerk = 1,
};

struct GimbalPacketCommand {
  GimbalPacketMode mode = GimbalPacketMode::kPosition;
  bool fire = false;
  float pitch = 0.0F;
  float yaw = 0.0F;
  float yaw_jerk = 0.0F;
  float pitch_jerk = 0.0F;
  float command_dt = 0.0F;
  std::uint32_t sequence = 0;
};

inline bool encodeGimbalPacketV2(const GimbalPacketCommand &command,
                                 FixedPacket<32> &packet) noexcept {
  const auto mode = static_cast<std::uint8_t>(command.mode);
  if (mode > static_cast<std::uint8_t>(GimbalPacketMode::kJerk) ||
      !std::isfinite(command.pitch) || !std::isfinite(command.yaw) ||
      !std::isfinite(command.yaw_jerk) || !std::isfinite(command.pitch_jerk) ||
      !std::isfinite(command.command_dt) ||
      (command.mode == GimbalPacketMode::kJerk && command.command_dt <= 0.0F) ||
      (command.mode == GimbalPacketMode::kPosition && command.command_dt < 0.0F)) {
    return false;
  }
  packet.clear();
  packet.loadData<std::uint8_t>(kGimbalPacketV2Magic, 1);
  packet.loadData<std::uint8_t>(mode, 2);
  packet.loadData<std::uint8_t>(command.fire ? 1U : 0U, 3);
  packet.loadData<float>(command.pitch, 4);
  packet.loadData<float>(command.yaw, 8);
  packet.loadData<float>(command.yaw_jerk, 12);
  packet.loadData<float>(command.pitch_jerk, 16);
  packet.loadData<float>(command.command_dt, 20);
  packet.loadData<std::uint32_t>(command.sequence, 24);
  packet.setCheckByte(0);
  return true;
}

inline bool decodeGimbalPacketV2(const FixedPacket<32> &packet,
                                 GimbalPacketCommand &command) noexcept {
  std::uint8_t magic = 0;
  std::uint8_t mode = 0;
  std::uint8_t fire = 0;
  if (!packet.unloadData<std::uint8_t>(magic, 1) ||
      !packet.unloadData<std::uint8_t>(mode, 2) ||
      !packet.unloadData<std::uint8_t>(fire, 3) || magic != kGimbalPacketV2Magic ||
      mode > static_cast<std::uint8_t>(GimbalPacketMode::kJerk) || fire > 1U) {
    return false;
  }
  command.mode = static_cast<GimbalPacketMode>(mode);
  command.fire = fire != 0;
  if (!packet.unloadData<float>(command.pitch, 4) ||
      !packet.unloadData<float>(command.yaw, 8) ||
      !packet.unloadData<float>(command.yaw_jerk, 12) ||
      !packet.unloadData<float>(command.pitch_jerk, 16) ||
      !packet.unloadData<float>(command.command_dt, 20) ||
      !packet.unloadData<std::uint32_t>(command.sequence, 24)) {
    return false;
  }
  return std::isfinite(command.pitch) && std::isfinite(command.yaw) &&
         std::isfinite(command.yaw_jerk) && std::isfinite(command.pitch_jerk) &&
         std::isfinite(command.command_dt) &&
         (command.mode == GimbalPacketMode::kJerk ? command.command_dt > 0.0F
                                                  : command.command_dt >= 0.0F);
}

}  // namespace fyt::serial_driver

#endif  // SERIAL_DRIVER_GIMBAL_PACKET_V2_HPP_
