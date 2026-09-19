#ifndef ARMOR_SOLVER_GIMBAL_COMMAND_CONTRACT_HPP_
#define ARMOR_SOLVER_GIMBAL_COMMAND_CONTRACT_HPP_

#include <cmath>
#include <cstdint>

#include "rm_interfaces/msg/gimbal_cmd.hpp"

namespace fyt::auto_aim {

enum class GimbalControlMode : std::uint8_t {
  kPosition = 0,
  kJerk = 1,
};

inline bool isValidGimbalCommand(const rm_interfaces::msg::GimbalCmd &command) noexcept {
  const auto finite = [](const double value) { return std::isfinite(value); };
  if (command.control_mode > static_cast<std::uint8_t>(GimbalControlMode::kJerk)) return false;
  if (!finite(command.yaw) || !finite(command.pitch) || !finite(command.distance) ||
      !finite(command.yaw_velocity) || !finite(command.pitch_velocity) ||
      !finite(command.yaw_acceleration) || !finite(command.pitch_acceleration) ||
      !finite(command.yaw_jerk) || !finite(command.pitch_jerk) ||
      !finite(command.command_dt)) {
    return false;
  }
  if (command.control_mode == static_cast<std::uint8_t>(GimbalControlMode::kJerk)) {
    return command.mpc_valid && command.command_dt > 0.0;
  }
  return command.command_dt >= 0.0;
}

}  // namespace fyt::auto_aim

#endif  // ARMOR_SOLVER_GIMBAL_COMMAND_CONTRACT_HPP_
