#include "armor_solver/gimbal_command_contract.hpp"

#include <cassert>
#include <limits>

int main() {
  rm_interfaces::msg::GimbalCmd command;
  command.control_mode = 0;
  command.command_dt = 0.0;
  assert(fyt::auto_aim::isValidGimbalCommand(command));

  command.control_mode = 1;
  command.mpc_valid = true;
  command.command_dt = 0.01;
  assert(fyt::auto_aim::isValidGimbalCommand(command));

  command.command_dt = 0.0;
  assert(!fyt::auto_aim::isValidGimbalCommand(command));
  command.command_dt = 0.01;
  command.control_mode = 2;
  assert(!fyt::auto_aim::isValidGimbalCommand(command));
  command.control_mode = 0;
  command.command_dt = -1.0;
  assert(!fyt::auto_aim::isValidGimbalCommand(command));
  command.command_dt = 0.0;
  command.yaw = std::numeric_limits<double>::quiet_NaN();
  assert(!fyt::auto_aim::isValidGimbalCommand(command));
  return 0;
}
