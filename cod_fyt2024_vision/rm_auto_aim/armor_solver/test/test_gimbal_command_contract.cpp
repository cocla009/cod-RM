#include "armor_solver/gimbal_command_contract.hpp"

#include <cassert>
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
  using fyt::auto_aim::GimbalCommandFormat;
  using fyt::auto_aim::isValidGimbalCommand;

  // The selector values must match fyt::serial_driver::GimbalCommandFormat.
  check(static_cast<std::uint8_t>(GimbalCommandFormat::kPosition) == 0U,
        "position format selector is 0");
  check(static_cast<std::uint8_t>(GimbalCommandFormat::kFeedforward) == 1U,
        "feedforward format selector is 1");

  rm_interfaces::msg::GimbalCmd command;

  // A feed-forward command: position plus velocity and acceleration.
  command.control_mode = static_cast<std::uint8_t>(GimbalCommandFormat::kFeedforward);
  command.mpc_valid = true;
  command.yaw = 12.5;
  command.pitch = -3.0;
  command.distance = 4.2;
  command.yaw_velocity = 180.0;
  command.pitch_velocity = -20.0;
  command.yaw_acceleration = 2500.0;
  command.pitch_acceleration = -900.0;
  check(isValidGimbalCommand(command), "a well-formed feed-forward command is valid");

  // The legacy format only carries position, so the higher-order fields staying
  // at zero must not disqualify it.
  command.control_mode = static_cast<std::uint8_t>(GimbalCommandFormat::kPosition);
  command.yaw_velocity = 0.0;
  command.pitch_velocity = 0.0;
  command.yaw_acceleration = 0.0;
  command.pitch_acceleration = 0.0;
  check(isValidGimbalCommand(command), "a position-only command is valid");

  // Idle frames are zero everywhere and must still pass.
  rm_interfaces::msg::GimbalCmd idle;
  idle.control_mode = static_cast<std::uint8_t>(GimbalCommandFormat::kFeedforward);
  idle.mpc_valid = false;
  idle.distance = -1.0;
  check(isValidGimbalCommand(idle), "an idle command is valid");

  // ── Rejects ─────────────────────────────────────────────────────────────
  command.control_mode = 2;
  check(!isValidGimbalCommand(command), "an unknown format selector is rejected");

  command.control_mode = static_cast<std::uint8_t>(GimbalCommandFormat::kFeedforward);
  command.yaw = std::numeric_limits<double>::quiet_NaN();
  check(!isValidGimbalCommand(command), "a NaN yaw is rejected");
  command.yaw = 0.0;

  command.yaw_acceleration = std::numeric_limits<double>::infinity();
  check(!isValidGimbalCommand(command), "an infinite acceleration is rejected");
  command.yaw_acceleration = 0.0;

  command.pitch = std::numeric_limits<double>::quiet_NaN();
  check(!isValidGimbalCommand(command), "a NaN pitch is rejected");
  command.pitch = 0.0;

  command.distance = std::numeric_limits<double>::infinity();
  check(!isValidGimbalCommand(command), "an infinite distance is rejected");

  std::printf("%s (%d failure(s))\n", failures == 0 ? "PASS" : "FAIL", failures);
  return failures == 0 ? 0 : 1;
}
