#include "armor_solver/gimbal_mpc.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace fyt::auto_aim {
namespace {
constexpr double kEpsilon = 1e-12;
using StateVector = std::array<double, 3>;

bool finiteState(const GimbalAxisState &state) noexcept {
  return std::isfinite(state.position) && std::isfinite(state.velocity) &&
         std::isfinite(state.acceleration);
}

bool finiteReference(const GimbalAxisReference &reference) noexcept {
  if (reference.position.empty() || reference.position.size() != reference.velocity.size() ||
      reference.position.size() != reference.acceleration.size()) {
    return false;
  }
  for (std::size_t i = 0; i < reference.position.size(); ++i) {
    if (!std::isfinite(reference.position[i]) || !std::isfinite(reference.velocity[i]) ||
        !std::isfinite(reference.acceleration[i])) {
      return false;
    }
  }
  return true;
}

bool feasibleReference(const GimbalAxisReference &reference,
                       const GimbalMpcConfig &config) noexcept {
  const double dt = std::max(config.dt, kEpsilon);
  for (std::size_t i = 0; i < reference.position.size(); ++i) {
    if (std::abs(reference.velocity[i]) > config.max_velocity + 1e-9 ||
        std::abs(reference.acceleration[i]) > config.max_acceleration + 1e-9) {
      return false;
    }
    if (i > 0 && std::abs(reference.acceleration[i] - reference.acceleration[i - 1]) / dt >
                   config.max_jerk + 1e-9) {
      return false;
    }
  }
  return true;
}

StateVector propagate(const StateVector &state, const double jerk, const double dt) noexcept {
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  return {
    state[0] + dt * state[1] + 0.5 * dt2 * state[2] + dt3 * jerk / 6.0,
    state[1] + dt * state[2] + 0.5 * dt2 * jerk,
    state[2] + dt * jerk,
  };
}

double clampFinite(const double value, const double lower, const double upper) noexcept {
  if (!std::isfinite(value)) return 0.0;
  return std::clamp(value, lower, upper);
}

void projectControls(const GimbalAxisState &initial,
                     const GimbalMpcConfig &config,
                     std::vector<double> &controls) noexcept {
  StateVector state{initial.position, initial.velocity, initial.acceleration};
  for (double &control : controls) {
    const double dt = std::max(config.dt, kEpsilon);
    double lower_jerk = -config.max_jerk;
    double upper_jerk = config.max_jerk;
    if (config.max_acceleration > 0.0) {
      lower_jerk = std::max(lower_jerk, (-config.max_acceleration - state[2]) / dt);
      upper_jerk = std::min(upper_jerk, (config.max_acceleration - state[2]) / dt);
    }
    if (config.max_velocity > 0.0) {
      const double half_dt_squared = 0.5 * dt * dt;
      lower_jerk = std::max(
        lower_jerk, (-config.max_velocity - state[1] - dt * state[2]) / half_dt_squared);
      upper_jerk = std::min(
        upper_jerk, (config.max_velocity - state[1] - dt * state[2]) / half_dt_squared);
    }
    if (lower_jerk > upper_jerk) {
      // An already infeasible state cannot be repaired without violating a bound.
      control = 0.0;
      return;
    }
    control = clampFinite(control, lower_jerk, upper_jerk);
    state = propagate(state, control, dt);
  }
}

void rollout(const GimbalAxisState &initial,
             const GimbalMpcConfig &config,
             const std::vector<double> &controls,
             std::vector<StateVector> &states) noexcept {
  states.clear();
  states.reserve(controls.size() + 1);
  StateVector state{initial.position, initial.velocity, initial.acceleration};
  states.push_back(state);
  for (const double control : controls) {
    state = propagate(state, control, config.dt);
    states.push_back(state);
  }
}

void seedControls(const GimbalAxisState &initial,
                  const GimbalAxisReference &reference,
                  const GimbalMpcConfig &config,
                  std::vector<double> &controls) noexcept {
  StateVector state{initial.position, initial.velocity, initial.acceleration};
  const double position_gain = 2.2 * std::sqrt(
    config.position_weight / std::max(config.acceleration_weight, kEpsilon));
  const double velocity_gain = 1.5 * std::sqrt(
    config.velocity_weight / std::max(config.acceleration_weight, kEpsilon));
  for (std::size_t i = 0; i < controls.size(); ++i) {
    const double position_error = reference.position[i] - state[0];
    const double velocity_error = reference.velocity[i] - state[1];
    const double desired_acceleration = reference.acceleration[i] +
      position_gain * position_error + velocity_gain * velocity_error;
    controls[i] = (desired_acceleration - state[2]) / std::max(config.dt, kEpsilon);
    controls[i] = clampFinite(controls[i], -config.max_jerk, config.max_jerk);
    state = propagate(state, controls[i], config.dt);
  }
}

double objective(const std::vector<StateVector> &states,
                 const std::vector<double> &controls,
                 const GimbalAxisReference &reference,
                 const GimbalMpcConfig &config) noexcept {
  if (states.size() != controls.size() + 1 || reference.position.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  double result = 0.0;
  const std::size_t count = std::min(controls.size(), reference.position.size());
  for (std::size_t i = 0; i < count; ++i) {
    const double position_error = states[i + 1][0] - reference.position[i];
    const double velocity_error = states[i + 1][1] - reference.velocity[i];
    const double acceleration_error = states[i + 1][2] - reference.acceleration[i];
    result += config.position_weight * position_error * position_error;
    result += config.velocity_weight * velocity_error * velocity_error;
    result += config.acceleration_weight * acceleration_error * acceleration_error;
    result += config.jerk_weight * controls[i] * controls[i];
  }
  const std::size_t terminal = std::min(count, states.size() - 1);
  if (terminal > 0) {
    const double position_error = states[terminal][0] - reference.position[terminal - 1];
    const double velocity_error = states[terminal][1] - reference.velocity[terminal - 1];
    result += config.terminal_weight *
      (position_error * position_error + velocity_error * velocity_error);
  }
  return result;
}

std::vector<double> analyticGradient(const std::vector<StateVector> &states,
                                     const std::vector<double> &controls,
                                     const GimbalAxisReference &reference,
                                     const GimbalMpcConfig &config) noexcept {
  const std::size_t count = controls.size();
  std::vector<double> gradient(count, 0.0);
  if (states.size() != count + 1 || reference.position.size() < count) return gradient;

  const double dt = config.dt;
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  StateVector costate_next{0.0, 0.0, 0.0};
  for (std::size_t reverse = count; reverse > 0; --reverse) {
    const std::size_t state_index = reverse;
    const std::size_t reference_index = state_index - 1;
    const auto &state = states[state_index];
    StateVector costate{
      2.0 * config.position_weight * (state[0] - reference.position[reference_index]),
      2.0 * config.velocity_weight * (state[1] - reference.velocity[reference_index]),
      2.0 * config.acceleration_weight *
        (state[2] - reference.acceleration[reference_index])};
    if (state_index == count) {
      costate[0] += 2.0 * config.terminal_weight *
        (state[0] - reference.position[reference_index]);
      costate[1] += 2.0 * config.terminal_weight *
        (state[1] - reference.velocity[reference_index]);
    }
    costate[0] += costate_next[0];
    costate[1] += costate_next[1];
    costate[2] += costate_next[2];

    const std::size_t control_index = state_index - 1;
    gradient[control_index] = 2.0 * config.jerk_weight * controls[control_index] +
      (dt3 / 6.0) * costate[0] + 0.5 * dt2 * costate[1] + dt * costate[2];

    costate_next = {costate[0], dt * costate[0] + costate[1],
                    0.5 * dt2 * costate[0] + dt * costate[1] + costate[2]};
  }
  return gradient;
}

double constraintViolation(const std::vector<StateVector> &states,
                           const std::vector<double> &controls,
                           const GimbalMpcConfig &config) noexcept {
  double violation = 0.0;
  for (const auto &state : states) {
    if (config.max_velocity > 0.0) {
      violation = std::max(violation, std::max(0.0, std::abs(state[1]) - config.max_velocity));
    }
    if (config.max_acceleration > 0.0) {
      violation = std::max(
        violation, std::max(0.0, std::abs(state[2]) - config.max_acceleration));
    }
  }
  for (const double control : controls) {
    if (config.max_jerk > 0.0) {
      violation = std::max(violation, std::max(0.0, std::abs(control) - config.max_jerk));
    }
  }
  return violation;
}

}  // namespace

GimbalMpc::GimbalMpc(GimbalMpcConfig config) : config_(config) {
  setConfig(config);
}

void GimbalMpc::setConfig(const GimbalMpcConfig &config) noexcept {
  config_ = config;
  if (!std::isfinite(config_.dt) || config_.dt <= 0.0) config_.dt = 0.01;
  if (config_.horizon < 2) config_.horizon = 2;
  if (!std::isfinite(config_.max_iterations) || config_.max_iterations < 1) {
    config_.max_iterations = 1;
  }
  config_.step_size = std::isfinite(config_.step_size) && config_.step_size > 0.0
    ? config_.step_size : 0.01;
  config_.convergence_tolerance = std::isfinite(config_.convergence_tolerance) &&
      config_.convergence_tolerance > 0.0 ? config_.convergence_tolerance : 1e-5;
  config_.max_velocity = std::max(0.0, config_.max_velocity);
  config_.max_acceleration = std::max(0.0, config_.max_acceleration);
  config_.max_jerk = std::max(0.0, config_.max_jerk);
  yaw_controls_.clear();
  pitch_controls_.clear();
}

void GimbalMpc::reset() noexcept {
  yaw_controls_.clear();
  pitch_controls_.clear();
}

GimbalAxisResult GimbalMpc::solveAxis(const GimbalAxisState &initial,
                                      const GimbalAxisReference &reference,
                                      std::vector<double> &warm_controls) const noexcept {
  GimbalAxisResult result;
  if (!finiteState(initial) || !finiteReference(reference) ||
      !feasibleReference(reference, config_) || reference.position.size() < 2 ||
      config_.horizon < 2 || config_.max_jerk <= 0.0 || config_.max_acceleration <= 0.0 ||
      config_.max_velocity <= 0.0) {
    return result;
  }
  if (std::abs(initial.velocity) > config_.max_velocity + 1e-9 ||
      std::abs(initial.acceleration) > config_.max_acceleration + 1e-9) {
    return result;
  }

  const std::size_t horizon = config_.horizon;
  std::vector<double> controls(horizon, 0.0);
  seedControls(initial, reference, config_, controls);
  if (warm_controls.size() >= horizon) {
    for (std::size_t i = 0; i < horizon; ++i) {
      controls[i] = 0.5 * (controls[i] + warm_controls[i]);
    }
  }
  projectControls(initial, config_, controls);

  std::vector<StateVector> states;
  rollout(initial, config_, controls, states);
  double current_objective = objective(states, controls, reference, config_);
  if (!std::isfinite(current_objective)) return result;

  std::size_t completed_iterations = 0;
  double max_delta = std::numeric_limits<double>::infinity();
  double step_size = config_.step_size;
  for (int iteration = 0; iteration < config_.max_iterations; ++iteration) {
    const auto gradient = analyticGradient(states, controls, reference, config_);

    std::vector<double> candidate = controls;
    max_delta = 0.0;
    for (std::size_t i = 0; i < horizon; ++i) {
      const double updated = controls[i] - step_size * gradient[i];
      max_delta = std::max(max_delta, std::abs(updated - controls[i]));
      candidate[i] = updated;
    }
    projectControls(initial, config_, candidate);
    std::vector<StateVector> candidate_states;
    rollout(initial, config_, candidate, candidate_states);
    const double candidate_objective = objective(candidate_states, candidate, reference, config_);
    const bool accepted = std::isfinite(candidate_objective) &&
      candidate_objective <= current_objective;
    if (accepted) {
      controls = std::move(candidate);
      states = std::move(candidate_states);
      current_objective = candidate_objective;
    } else {
      step_size *= 0.5;
    }
    completed_iterations = static_cast<std::size_t>(iteration + 1);
    if (max_delta < config_.convergence_tolerance) {
      result.converged = true;
      break;
    }
  }

  if (states.size() < 2 || !std::isfinite(current_objective)) return result;
  result.valid = true;
  result.iterations = completed_iterations;
  result.objective = current_objective;
  result.max_constraint_violation = constraintViolation(states, controls, config_);
  result.converged = result.converged ||
    (completed_iterations == static_cast<std::size_t>(config_.max_iterations) &&
     result.max_constraint_violation < 1e-8);
  result.predicted.reserve(states.size() - 1);
  for (std::size_t i = 1; i < states.size(); ++i) {
    const GimbalAxisState predicted{states[i][0], states[i][1], states[i][2]};
    if (!finiteState(predicted)) return GimbalAxisResult{};
    result.predicted.push_back(predicted);
  }
  result.command = result.predicted.front();
  result.control = controls.front();
  if (!finiteState(result.command) || !std::isfinite(result.control) ||
      !std::isfinite(result.objective) ||
      !std::isfinite(result.max_constraint_violation)) {
    return GimbalAxisResult{};
  }

  warm_controls.assign(controls.begin() + 1, controls.end());
  warm_controls.push_back(controls.back());
  return result;
}

GimbalMpcResult GimbalMpc::solve(const GimbalMpcInput &input) noexcept {
  GimbalMpcResult result;
  if (input.yaw_reference.position.size() != config_.horizon ||
      input.pitch_reference.position.size() != config_.horizon) {
    return result;
  }
  result.yaw = solveAxis(input.yaw, input.yaw_reference, yaw_controls_);
  result.pitch = solveAxis(input.pitch, input.pitch_reference, pitch_controls_);
  result.valid = result.yaw.valid && result.pitch.valid &&
    result.yaw.max_constraint_violation < 1e-8 && result.pitch.max_constraint_violation < 1e-8;
  result.converged = result.yaw.converged && result.pitch.converged;
  return result;
}

}  // namespace fyt::auto_aim
