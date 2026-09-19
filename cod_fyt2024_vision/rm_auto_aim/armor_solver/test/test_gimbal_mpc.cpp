#include "armor_solver/gimbal_mpc.hpp"

#include <cassert>
#include <cmath>
#include <limits>

using fyt::auto_aim::GimbalAxisReference;
using fyt::auto_aim::GimbalAxisState;
using fyt::auto_aim::GimbalMpc;
using fyt::auto_aim::GimbalMpcConfig;
using fyt::auto_aim::GimbalMpcInput;

int main() {
  GimbalMpcConfig config;
  config.horizon = 30;
  config.max_velocity = 4.0;
  config.max_acceleration = 20.0;
  config.max_jerk = 300.0;
  config.max_iterations = 12;
  config.step_size = 0.002;

  GimbalAxisReference reference;
  reference.position.resize(config.horizon);
  reference.velocity.resize(config.horizon);
  reference.acceleration.resize(config.horizon);
  for (std::size_t i = 0; i < config.horizon; ++i) {
    const double t = (i + 1) * config.dt;
    reference.position[i] = 0.8 * (1.0 - std::exp(-4.0 * t));
    reference.velocity[i] = 3.2 * std::exp(-4.0 * t);
    reference.acceleration[i] = -12.8 * std::exp(-4.0 * t);
  }

  GimbalMpcInput input;
  input.yaw_reference = reference;
  input.pitch_reference = reference;
  input.yaw = {};
  input.pitch = {};
  GimbalMpc mpc(config);
  const auto first = mpc.solve(input);
  assert(first.valid); assert(first.converged);
  assert(first.yaw.command.position > 0.0);
  assert(std::abs(first.yaw.command.velocity) <= config.max_velocity + 1e-9);
  assert(std::abs(first.yaw.command.acceleration) <= config.max_acceleration + 1e-9);
  assert(std::abs(first.yaw.control) <= config.max_jerk + 1e-9);
  assert(first.yaw.max_constraint_violation < 1e-9);

  const auto second = mpc.solve(input);
  assert(second.valid);
  assert(std::isfinite(second.yaw.objective));

  input.yaw_reference.position[0] = std::numeric_limits<double>::quiet_NaN();
  const auto invalid_reference = mpc.solve(input);
  assert(!invalid_reference.valid);
  assert(!invalid_reference.converged);

  input.yaw_reference.position[0] = reference.position[0];
  input.yaw.velocity = config.max_velocity + 1.0;
  const auto invalid_state = mpc.solve(input);
  assert(!invalid_state.valid);
  assert(!invalid_state.converged);

  input.yaw.velocity = 0.0;
  input.yaw_reference.velocity[0] = config.max_velocity * 2.0;
  const auto infeasible_reference = mpc.solve(input);
  assert(!infeasible_reference.valid);
  assert(!infeasible_reference.converged);
  return 0;
}
