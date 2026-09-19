#ifndef SERIAL_DRIVER_GIMBAL_COMMAND_RECEIVER_HPP_
#define SERIAL_DRIVER_GIMBAL_COMMAND_RECEIVER_HPP_

#include <cmath>
#include <cstdint>

#include "rm_serial_driver/fixed_packet.hpp"
#include "rm_serial_driver/gimbal_packet_v2.hpp"

namespace fyt::serial_driver {

struct GimbalActuatorCommand {
  GimbalPacketMode mode = GimbalPacketMode::kPosition;
  bool fire = false;
  float pitch = 0.0F;
  float yaw = 0.0F;
  float distance = 0.0F;
  float yaw_jerk = 0.0F;
  float pitch_jerk = 0.0F;
  float command_dt = 0.0F;
  std::uint32_t sequence = 0;
};

class GimbalCommandReceiver {
public:
  explicit GimbalCommandReceiver(float max_command_dt = 0.5F)
  : max_command_dt_(max_command_dt > 0.0F && std::isfinite(max_command_dt)
                      ? max_command_dt
                      : 0.5F) {}

  bool receiveLegacyPosition(const FixedPacket<16> &packet,
                             GimbalActuatorCommand &command) const noexcept {
    if (!validFrame<16>(packet.buffer())) return false;
    std::uint8_t fire = 0;
    float pitch = 0.0F;
    float yaw = 0.0F;
    float distance = 0.0F;
    if (!packet.unloadData<std::uint8_t>(fire, 1) || fire > 1U ||
        !packet.unloadData<float>(pitch, 2) || !packet.unloadData<float>(yaw, 6) ||
        !packet.unloadData<float>(distance, 10) || !std::isfinite(pitch) ||
        !std::isfinite(yaw) || !std::isfinite(distance)) {
      return false;
    }
    command = {};
    command.mode = GimbalPacketMode::kPosition;
    command.fire = fire != 0U;
    command.pitch = pitch;
    command.yaw = yaw;
    command.distance = distance;
    return true;
  }

  bool receiveVersioned(const FixedPacket<32> &packet,
                        GimbalActuatorCommand &command) noexcept {
    if (!validFrame<32>(packet.buffer())) return false;
    GimbalPacketCommand decoded;
    if (!decodeGimbalPacketV2(packet, decoded)) return false;
    if (decoded.mode == GimbalPacketMode::kJerk) {
      if (decoded.command_dt > max_command_dt_ || !acceptSequence(decoded.sequence)) {
        return false;
      }
    }
    command = {};
    command.mode = decoded.mode;
    command.fire = decoded.fire;
    command.pitch = decoded.pitch;
    command.yaw = decoded.yaw;
    command.yaw_jerk = decoded.mode == GimbalPacketMode::kJerk ? decoded.yaw_jerk : 0.0F;
    command.pitch_jerk = decoded.mode == GimbalPacketMode::kJerk ? decoded.pitch_jerk : 0.0F;
    command.command_dt = decoded.command_dt;
    command.sequence = decoded.sequence;
    return true;
  }

  void resetSequence() noexcept {
    has_sequence_ = false;
    last_sequence_ = 0;
  }

private:
  template <int capacity>
  static bool validFrame(const std::uint8_t *buffer) noexcept {
    return buffer != nullptr && buffer[0] == 0xFFU &&
           buffer[capacity - 1] == 0x0DU;
  }

  bool acceptSequence(const std::uint32_t sequence) noexcept {
    if (!has_sequence_) {
      last_sequence_ = sequence;
      has_sequence_ = true;
      return true;
    }
    const auto delta = sequence - last_sequence_;
    if (delta == 0U || delta >= 0x80000000U) return false;
    last_sequence_ = sequence;
    return true;
  }

  float max_command_dt_;
  bool has_sequence_ = false;
  std::uint32_t last_sequence_ = 0;
};

}  // namespace fyt::serial_driver

#endif  // SERIAL_DRIVER_GIMBAL_COMMAND_RECEIVER_HPP_
