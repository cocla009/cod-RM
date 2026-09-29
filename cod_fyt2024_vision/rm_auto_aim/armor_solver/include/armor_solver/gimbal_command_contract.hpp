// Sanity contract applied to a GimbalCmd before it is published.

#ifndef ARMOR_SOLVER_GIMBAL_COMMAND_CONTRACT_HPP_
#define ARMOR_SOLVER_GIMBAL_COMMAND_CONTRACT_HPP_

#include <cmath>
#include <cstdint>

#include "rm_interfaces/msg/gimbal_cmd.hpp"

namespace fyt::auto_aim {

// Values carried in GimbalCmd::control_mode to select the wire format. Must
// stay in sync with fyt::serial_driver::GimbalCommandFormat in rm_serial_driver;
// armor_solver does not depend on that package.
enum class GimbalCommandFormat : std::uint8_t {
  kPosition = 0,     // legacy 16-byte position packet
  kFeedforward = 1,  // 32-byte position + velocity + acceleration packet
};

inline bool isValidGimbalCommand(const rm_interfaces::msg::GimbalCmd &command) noexcept {
  const auto finite = [](const double value) { return std::isfinite(value); };
  if (command.control_mode > static_cast<std::uint8_t>(GimbalCommandFormat::kFeedforward)) {
    return false;
  }
  // The six motion fields all travel on the wire; the former jerk and
  // command_dt fields are no longer transmitted by either format.
  return finite(command.yaw) && finite(command.pitch) && finite(command.distance) &&
         finite(command.yaw_velocity) && finite(command.pitch_velocity) &&
         finite(command.yaw_acceleration) && finite(command.pitch_acceleration);
}

}  // namespace fyt::auto_aim

#endif  // ARMOR_SOLVER_GIMBAL_COMMAND_CONTRACT_HPP_
