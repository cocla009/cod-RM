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
    control = clampFinite(control, -config.max_jerk, config.max_jerk);
    double next_acceleration = state[2] + config.dt * control;
    next_acceleration = clampFinite(
      next_acceleration, -config.max_acceleration, config.max_acceleration);

    if (config.max_velocity > 0.0) {
      const double predicted_velocity = state[1] +
        config.dt * (state[2] + next_acceleration) * 0.5;
      if (std::abs(predicted_velocity) > config.max_velocity) {
        const double limited_velocity = std::copysign(config.max_velocity, predicted_velocity);
        next_acceleration = clampFinite(
          2.0 * (limited_velocity - state[1]) / std::max(config.dt, kEpsilon) - state[2],
          -config.max_acceleration,
          config.max_acceleration);
      }
    }

    control = clampFinite(
      (next_acceleration - state[2]) / std::max(config.dt, kEpsilon),
      -config.max_jerk,
      config.max_jerk);
    state = propagate(state, control, config.dt);
    state[1] = clampFinite(state[1], -config.max_velocity, config.max_velocity);
    state[2] = clampFinite(state[2], -config.max_acceleration, config.max_acceleration);
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
  yaw_controls_.assign(config_.horizon, 0.0);
  pitch_controls_.assign(config_.horizon, 0.0);
}

void GimbalMpc::reset() noexcept {
  std::fill(yaw_controls_.begin(), yaw_controls_.end(), 0.0);
  std::fill(pitch_controls_.begin(), pitch_controls_.end(), 0.0);
}

GimbalAxisResult GimbalMpc::solveAxis(const GimbalAxisState &initial,
                                      const GimbalAxisReference &reference,
                                      std::vector<double> &warm_controls) const noexcept {
  GimbalAxisResult result;
  if (!finiteState(initial) || !finiteReference(reference) || reference.position.size() < 2 ||
      config_.horizon < 2 || config_.max_jerk <= 0.0 || config_.max_acceleration <= 0.0 ||
      config_.max_velocity <= 0.0) {
    return result;
  }

  const std::size_t horizon = config_.horizon;
  std::vector<double> controls(horizon, 0.0);
  for (std::size_t i = 0; i < horizon && i < warm_controls.size(); ++i) controls[i] = warm_controls[i];
  projectControls(initial, config_, controls);

  std::vector<StateVector> states;
  rollout(initial, config_, controls, states);
  double current_objective = objective(states, controls, reference, config_);
  if (!std::isfinite(current_objective)) return result;

  std::size_t completed_iterations = 0;
  double max_delta = std::numeric_limits<double>::infinity();
  double step_size = config_.step_size;
  for (int iteration = 0; iteration < config_.max_iterations; ++iteration) {
    std::vector<double> gradient(horizon, 0.0);
    for (std::size_t control_index = 0; control_index < horizon; ++control_index) {
      const double delta = std::max(1e-6, 1e-4 * std::abs(controls[control_index]));
      std::vector<double> perturbed = controls;
      perturbed[control_index] += delta;
      projectControls(initial, config_, perturbed);
      std::vector<StateVector> perturbed_states;
      rollout(initial, config_, perturbed, perturbed_states);
      const double perturbed_objective = objective(perturbed_states, perturbed, reference, config_);
      gradient[control_index] = (perturbed_objective - current_objective) / delta;
      if (!std::isfinite(gradient[control_index])) gradient[control_index] = 0.0;
      gradient[control_index] += 2.0 * config_.jerk_weight * controls[control_index];
    }

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
    if (candidate_objective <= current_objective || max_delta < config_.convergence_tolerance) {
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
    result.predicted.push_back({states[i][0], states[i][1], states[i][2]});
  }
  result.command = result.predicted.front();

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
