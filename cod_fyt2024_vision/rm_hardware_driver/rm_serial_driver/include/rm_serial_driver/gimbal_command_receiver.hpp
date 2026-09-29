// Upper-computer-side model of what the lower controller receives. Used by the
// protocol tests; the real parser lives in the MCU firmware and must agree with
// gimbal_packet_v3.hpp byte for byte.

#ifndef SERIAL_DRIVER_GIMBAL_COMMAND_RECEIVER_HPP_
#define SERIAL_DRIVER_GIMBAL_COMMAND_RECEIVER_HPP_

#include <cmath>
#include <cstdint>

#include "rm_serial_driver/fixed_packet.hpp"
#include "rm_serial_driver/gimbal_packet_v3.hpp"

namespace fyt::serial_driver {

struct GimbalActuatorCommand {
  GimbalPacketMode mode = GimbalPacketMode::kIdle;
  bool fire = false;
  bool control = false;
  float pitch = 0.0F;      // degrees
  float yaw = 0.0F;        // degrees
  float pitch_vel = 0.0F;  // degrees/s
  float pitch_acc = 0.0F;  // degrees/s^2
  float yaw_vel = 0.0F;    // degrees/s
  float yaw_acc = 0.0F;    // degrees/s^2
  float distance = 0.0F;   // legacy packet only
};

class GimbalCommandReceiver {
public:
  GimbalCommandReceiver() = default;

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
    command.mode = fire != 0U ? GimbalPacketMode::kFire : GimbalPacketMode::kControl;
    command.fire = fire != 0U;
    command.control = true;
    command.pitch = pitch;
    command.yaw = yaw;
    command.distance = distance;
    return true;
  }

  bool receiveVersioned(const FixedPacket<32> &packet,
                        GimbalActuatorCommand &command) noexcept {
    GimbalPacketCommand decoded;
    if (!decodeGimbalPacketV3(packet, decoded)) return false;

    command = {};
    command.mode = decoded.mode;
    // An idle frame explicitly means "stop driving the gimbal".
    command.control = decoded.mode != GimbalPacketMode::kIdle;
    command.fire = decoded.mode == GimbalPacketMode::kFire;
    command.pitch = decoded.pitch;
    command.yaw = decoded.yaw;
    command.pitch_vel = decoded.pitch_vel;
    command.pitch_acc = decoded.pitch_acc;
    command.yaw_vel = decoded.yaw_vel;
    command.yaw_acc = decoded.yaw_acc;
    return true;
  }

private:
  template <int capacity>
  static bool validFrame(const std::uint8_t *buffer) noexcept {
    return buffer != nullptr && buffer[0] == 0xFFU &&
           buffer[capacity - 1] == 0x0DU;
  }
};

}  // namespace fyt::serial_driver

#endif  // SERIAL_DRIVER_GIMBAL_COMMAND_RECEIVER_HPP_
