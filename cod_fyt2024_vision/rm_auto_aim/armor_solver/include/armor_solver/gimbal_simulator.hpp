#ifndef ARMOR_SOLVER_GIMBAL_SIMULATOR_HPP_
#define ARMOR_SOLVER_GIMBAL_SIMULATOR_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>

#include "armor_solver/gimbal_mpc.hpp"

namespace fyt::auto_aim {

// Deterministic jerk-limited plant used by the ROS-free controller tests.
class GimbalAxisSimulator {
public:
  explicit GimbalAxisSimulator(GimbalMpcConfig config = {}) : config_(config) {
    config_.dt = std::isfinite(config_.dt) && config_.dt > 0.0 ? config_.dt : 0.01;
    config_.max_velocity = std::max(0.0, config_.max_velocity);
    config_.max_acceleration = std::max(0.0, config_.max_acceleration);
    config_.max_jerk = std::max(0.0, config_.max_jerk);
  }

  void reset(const GimbalAxisState &state = {}) noexcept { state_ = state; }

  const GimbalAxisState &state() const noexcept { return state_; }

  GimbalAxisState step(double jerk) noexcept {
    if (!std::isfinite(jerk)) jerk = 0.0;
    if (config_.max_jerk > 0.0) {
      jerk = std::clamp(jerk, -config_.max_jerk, config_.max_jerk);
    }

    const double dt = config_.dt;
    double next_acceleration = state_.acceleration + dt * jerk;
    if (config_.max_acceleration > 0.0) {
      next_acceleration = std::clamp(
        next_acceleration, -config_.max_acceleration, config_.max_acceleration);
    }

    if (config_.max_velocity > 0.0) {
      const double predicted_velocity =
        state_.velocity + dt * (state_.acceleration + next_acceleration) * 0.5;
      if (std::abs(predicted_velocity) > config_.max_velocity) {
        const double limited_velocity =
          std::copysign(config_.max_velocity, predicted_velocity);
        next_acceleration = std::clamp(
          2.0 * (limited_velocity - state_.velocity) / dt - state_.acceleration,
          -config_.max_acceleration, config_.max_acceleration);
      }
    }

    double applied_jerk = (next_acceleration - state_.acceleration) / dt;
    if (config_.max_jerk > 0.0) {
      applied_jerk = std::clamp(applied_jerk, -config_.max_jerk, config_.max_jerk);
      next_acceleration = state_.acceleration + dt * applied_jerk;
    }
    state_.position += dt * state_.velocity + 0.5 * dt * dt * state_.acceleration +
      dt * dt * dt * applied_jerk / 6.0;
    state_.velocity += dt * state_.acceleration + 0.5 * dt * dt * applied_jerk;
    state_.acceleration = next_acceleration;
    if (config_.max_velocity > 0.0) {
      state_.velocity = std::clamp(state_.velocity, -config_.max_velocity, config_.max_velocity);
    }
    if (config_.max_acceleration > 0.0) {
      state_.acceleration = std::clamp(
        state_.acceleration, -config_.max_acceleration, config_.max_acceleration);
    }
    last_jerk_ = applied_jerk;
    return state_;
  }

  double lastJerk() const noexcept { return last_jerk_; }

private:
  GimbalMpcConfig config_;
  GimbalAxisState state_{};
  double last_jerk_ = 0.0;
};

// Discrete command transport used by closed-loop tests. A delay of N means
// the command submitted at step k is first applied at step k + N.
class GimbalCommandDelay {
public:
  explicit GimbalCommandDelay(const std::size_t delay_steps = 0)
  : delay_steps_(delay_steps) {}

  void reset() noexcept { commands_.clear(); }

  GimbalAxisState push(const GimbalAxisState &command) {
    commands_.push_back(command);
    if (commands_.size() <= delay_steps_) return {};
    const auto applied = commands_.front();
    commands_.pop_front();
    return applied;
  }

  std::size_t delaySteps() const noexcept { return delay_steps_; }

private:
  std::size_t delay_steps_;
  std::deque<GimbalAxisState> commands_;
};

class GimbalSimulator {
public:
  explicit GimbalSimulator(GimbalMpcConfig config = {})
  : yaw_(config), pitch_(config), config_dt_(std::isfinite(config.dt) && config.dt > 0.0 ? config.dt : 0.01) {}

  void reset(const GimbalAxisState &yaw = {}, const GimbalAxisState &pitch = {}) noexcept {
    yaw_.reset(yaw);
    pitch_.reset(pitch);
  }

  void step(const GimbalAxisState &yaw_command,
            const GimbalAxisState &pitch_command) noexcept {
    yaw_.step((yaw_command.acceleration - yaw_.state().acceleration) /
              std::max(1e-12, dt()));
    pitch_.step((pitch_command.acceleration - pitch_.state().acceleration) /
              std::max(1e-12, dt()));
  }

  void stepJerk(const double yaw_jerk, const double pitch_jerk) noexcept {
    yaw_.step(yaw_jerk);
    pitch_.step(pitch_jerk);
  }

  const GimbalAxisSimulator &yaw() const noexcept { return yaw_; }
  const GimbalAxisSimulator &pitch() const noexcept { return pitch_; }
  double dt() const noexcept { return config_dt_; }

private:
  GimbalAxisSimulator yaw_;
  GimbalAxisSimulator pitch_;
  double config_dt_ = 0.01;
};

}  // namespace fyt::auto_aim

#endif  // ARMOR_SOLVER_GIMBAL_SIMULATOR_HPP_
