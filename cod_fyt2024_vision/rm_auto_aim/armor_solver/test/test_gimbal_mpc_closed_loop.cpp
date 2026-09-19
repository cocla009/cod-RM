#include "armor_solver/aim_reference.hpp"
#include "armor_solver/gimbal_mpc.hpp"
#include "armor_solver/gimbal_simulator.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

using fyt::auto_aim::AimReferenceGenerator;
using fyt::auto_aim::ArmorPlannerConfig;
using fyt::auto_aim::ArmorPlannerInput;
using fyt::auto_aim::GimbalAxisReference;
using fyt::auto_aim::GimbalAxisState;
using fyt::auto_aim::GimbalCommandDelay;
using fyt::auto_aim::GimbalMpc;
using fyt::auto_aim::GimbalMpcConfig;
using fyt::auto_aim::GimbalMpcInput;
using fyt::auto_aim::GimbalSimulator;

namespace {

constexpr double kPi = 3.14159265358979323846;

GimbalAxisReference makeReference(double start,
                                  double velocity,
                                  std::size_t horizon,
                                  double dt,
                                  int preview_delay_steps = 0) {
  GimbalAxisReference reference;
  reference.position.resize(horizon);
  reference.velocity.resize(horizon, velocity);
  reference.acceleration.resize(horizon, 0.0);
  start += velocity * static_cast<double>(preview_delay_steps) * dt;
  for (std::size_t i = 0; i < horizon; ++i) {
    reference.position[i] = start + velocity * static_cast<double>(i + 1) * dt;
  }
  return reference;
}

void assertFiniteState(const GimbalAxisState &state) {
  assert(std::isfinite(state.position));
  assert(std::isfinite(state.velocity));
  assert(std::isfinite(state.acceleration));
}

void testPlantLimits() {
  GimbalMpcConfig config;
  config.dt = 0.01;
  config.max_velocity = 2.0;
  config.max_acceleration = 5.0;
  config.max_jerk = 20.0;
  fyt::auto_aim::GimbalAxisSimulator plant(config);
  for (int i = 0; i < 1000; ++i) {
    const auto state = plant.step(1e6);
    assertFiniteState(state);
    assert(std::abs(state.velocity) <= config.max_velocity + 1e-9);
    assert(std::abs(state.acceleration) <= config.max_acceleration + 1e-9);
    assert(std::abs(plant.lastJerk()) <= config.max_jerk + 1e-9);
  }
}

struct ControlMetrics {
  double error = 0.0;
  double jerk = 0.0;
};

void testCommandDelayContract() {
  const GimbalAxisState command{1.0, 2.0, 3.0};
  GimbalCommandDelay immediate(0);
  assertFiniteState(immediate.push(command));
  assert(immediate.push(command).acceleration == command.acceleration);

  GimbalCommandDelay delayed(2);
  const auto first = delayed.push(command);
  const auto second = delayed.push({4.0, 5.0, 6.0});
  const auto third = delayed.push({7.0, 8.0, 9.0});
  assert(first.acceleration == 0.0);
  assert(second.acceleration == 0.0);
  assert(third.acceleration == command.acceleration);
}

ControlMetrics testMpcClosedLoop(double target_velocity,
                                 double initial_yaw,
                                 int command_delay_steps,
                                 double measurement_noise) {
  GimbalMpcConfig config;
  config.horizon = 20;
  config.max_velocity = 8.0;
  config.max_acceleration = 20.0;
  config.max_jerk = 120.0;
  config.max_iterations = 12;
  config.step_size = 0.001;
  GimbalMpc mpc(config);
  GimbalSimulator simulator(config);
  simulator.reset({initial_yaw, 0.0, 0.0}, {});

  double accumulated_error = 0.0;
  double accumulated_jerk = 0.0;
  GimbalCommandDelay yaw_command_delay(static_cast<std::size_t>(command_delay_steps));
  GimbalCommandDelay pitch_command_delay(static_cast<std::size_t>(command_delay_steps));
  for (int frame = 0; frame < 100; ++frame) {
    const double now = frame * config.dt;
    const double desired_yaw = initial_yaw + target_velocity * now;
    GimbalMpcInput input;
    input.yaw = simulator.yaw().state();
    input.yaw.position += measurement_noise * std::sin(0.37 * frame);
    input.pitch = simulator.pitch().state();
    input.yaw_reference = makeReference(
      desired_yaw, target_velocity, config.horizon, config.dt, command_delay_steps);
    input.pitch_reference = makeReference(0.0, 0.0, config.horizon, config.dt);
    const auto result = mpc.solve(input);
    assert(result.valid);
    assert(result.converged);
    assert(result.yaw.max_constraint_violation < 1e-8);
    assert(result.pitch.max_constraint_violation < 1e-8);
    assert(!result.yaw.predicted.empty());
    assertFiniteState(result.yaw.command);
    assertFiniteState(result.pitch.command);
    accumulated_error += std::abs(simulator.yaw().state().position - desired_yaw);
    const auto applied_yaw = yaw_command_delay.push({0.0, 0.0, result.yaw.control});
    const auto applied_pitch = pitch_command_delay.push({0.0, 0.0, result.pitch.control});
    simulator.stepJerk(applied_yaw.acceleration, applied_pitch.acceleration);
    accumulated_jerk += std::abs(simulator.yaw().lastJerk());
    assertFiniteState(simulator.yaw().state());
    assert(std::abs(simulator.yaw().state().velocity) <= config.max_velocity + 1e-8);
    assert(std::abs(simulator.yaw().state().acceleration) <= config.max_acceleration + 1e-8);
  }
  assert(std::isfinite(accumulated_error));
  assert(std::isfinite(accumulated_jerk));
  return {accumulated_error, accumulated_jerk};
}

ControlMetrics testAnalyticClosedLoop(double target_velocity,
                                      double initial_yaw,
                                      int command_delay_steps,
                                      double measurement_noise) {
  GimbalMpcConfig config;
  config.max_velocity = 8.0;
  config.max_acceleration = 20.0;
  config.max_jerk = 120.0;
  fyt::auto_aim::GimbalAxisSimulator simulator(config);
  simulator.reset({initial_yaw, 0.0, 0.0});
  GimbalCommandDelay command_delay(static_cast<std::size_t>(command_delay_steps));
  ControlMetrics metrics;
  for (int frame = 0; frame < 100; ++frame) {
    const double desired = initial_yaw + target_velocity * frame * config.dt;
    const double control_target = desired +
      target_velocity * static_cast<double>(command_delay_steps) * config.dt;
    const auto state = simulator.state();
    auto measured_state = state;
    measured_state.position += measurement_noise * std::sin(0.37 * frame);
    const double desired_acceleration = std::clamp(
      30.0 * (control_target - measured_state.position) +
        6.0 * (target_velocity - measured_state.velocity),
      -config.max_acceleration, config.max_acceleration);
    const double jerk = std::clamp(
      (desired_acceleration - state.acceleration) / config.dt,
      -config.max_jerk, config.max_jerk);
    metrics.error += std::abs(state.position - desired);
    const auto applied = command_delay.push({0.0, 0.0, jerk});
    simulator.step(applied.acceleration);
    metrics.jerk += std::abs(simulator.lastJerk());
  }
  return metrics;
}

void testReferenceScenarios() {
  ArmorPlannerConfig planner_config;
  ArmorPlannerInput input;
  input.center = Eigen::Vector3d(3.0, 0.0, 0.5);
  input.velocity = Eigen::Vector3d(0.1, 0.0, 0.0);
  input.yaw = kPi - 0.01;
  input.v_yaw = 8.0;
  input.radius_1 = 0.22;
  input.radius_2 = 0.26;
  input.dz = 0.1;
  input.armors_num = 4;
  const auto result = AimReferenceGenerator::generate(
    input, planner_config, 0.01, 20,
    [](const Eigen::Vector3d &position) { return position.norm() / 20.0; },
    [](const Eigen::Vector3d &position, double &pitch) {
      pitch = std::atan2(position.z(), position.head<2>().norm());
      return std::isfinite(pitch);
    });
  const auto delayed_result = AimReferenceGenerator::generate(
    input, planner_config, 0.01, 20,
    [](const Eigen::Vector3d &position) { return position.norm() / 20.0; },
    [](const Eigen::Vector3d &position, double &pitch) {
      pitch = std::atan2(position.z(), position.head<2>().norm());
      return std::isfinite(pitch);
    }, -1, 0.02);
  assert(result.valid);
  assert(delayed_result.valid);
  assert(result.samples.size() == 20);
  assert(delayed_result.samples.size() == 20);
  assert(std::abs(delayed_result.samples.front().yaw - result.samples.front().yaw) > 1e-9);
  for (std::size_t i = 0; i < result.samples.size(); ++i) {
    assert(std::isfinite(result.samples[i].yaw));
    assert(std::isfinite(result.samples[i].yaw_velocity));
    assert(std::isfinite(result.samples[i].yaw_acceleration));
    if (i > 0) assert(std::abs(result.samples[i].yaw - result.samples[i - 1].yaw) < kPi);
  }
  assert(std::abs(AimReferenceGenerator::unwrapNear(-kPi + 0.01, kPi - 0.01) -
                  (kPi + 0.01)) < 1e-6);

  ArmorPlannerInput switch_input = input;
  switch_input.yaw = kPi;
  switch_input.v_yaw = 0.0;
  const auto switched = AimReferenceGenerator::generate(
    switch_input, planner_config, 0.01, 8,
    [](const Eigen::Vector3d &position) { return position.norm() / 20.0; },
    [](const Eigen::Vector3d &position, double &pitch) {
      pitch = std::atan2(position.z(), position.head<2>().norm());
      return true;
    },
    0);
  assert(switched.valid);
  assert(switched.selection_discontinuous);
  assert(!switched.safe);

  ArmorPlannerInput expired = input;
  expired.target_age_seconds = planner_config.max_target_age + 0.1;
  const auto expired_plan = fyt::auto_aim::ArmorTrajectoryPlanner::plan(
    expired, planner_config,
    [](const Eigen::Vector3d &position) { return position.norm() / 20.0; });
  assert(!expired_plan.converged);
  assert(expired_plan.target_expired);
  assert(!expired_plan.safe);
}

}  // namespace

int main() {
  testCommandDelayContract();
  testPlantLimits();
  const auto static_mpc = testMpcClosedLoop(0.0, 0.0, 0, 0.0);
  const auto low_mpc = testMpcClosedLoop(1.0, 0.0, 1, 0.001);
  const auto high_mpc = testMpcClosedLoop(3.5, kPi - 0.01, 1, 0.001);
  const auto static_analytic = testAnalyticClosedLoop(0.0, 0.0, 0, 0.0);
  const auto low_analytic = testAnalyticClosedLoop(1.0, 0.0, 1, 0.001);
  const auto high_analytic = testAnalyticClosedLoop(3.5, kPi - 0.01, 1, 0.001);
  assert(std::isfinite(static_mpc.error) && std::isfinite(static_mpc.jerk));
  assert(std::isfinite(low_mpc.error) && std::isfinite(low_mpc.jerk));
  assert(std::isfinite(high_mpc.error) && std::isfinite(high_mpc.jerk));
  assert(low_mpc.error <= 1.02 * low_analytic.error);
  assert(high_mpc.error <= 1.02 * high_analytic.error);
  assert(low_mpc.jerk <= low_analytic.jerk);
  assert(high_mpc.jerk <= high_analytic.jerk);
  std::cout << "scenario=static mpc_error=" << static_mpc.error
            << " analytic_error=" << static_analytic.error
            << " mpc_jerk=" << static_mpc.jerk
            << " analytic_jerk=" << static_analytic.jerk << "\n";
  std::cout << "scenario=low_rotation mpc_error=" << low_mpc.error
            << " analytic_error=" << low_analytic.error
            << " mpc_jerk=" << low_mpc.jerk
            << " analytic_jerk=" << low_analytic.jerk << "\n";
  std::cout << "scenario=high_rotation_wrap mpc_error=" << high_mpc.error
            << " analytic_error=" << high_analytic.error
            << " mpc_jerk=" << high_mpc.jerk
            << " analytic_jerk=" << high_analytic.jerk << "\n";
  testReferenceScenarios();
  std::cout << "gimbal_mpc_closed_loop: PASS\n";
  return 0;
}
