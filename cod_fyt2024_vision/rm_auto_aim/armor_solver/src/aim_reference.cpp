#include "armor_solver/aim_reference.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fyt::auto_aim {
namespace {
constexpr double kTwoPi = 2.0 * M_PI;
constexpr double kMaxReferenceStep = 0.5 * M_PI;

bool finiteSample(const AimReferenceSample &sample) noexcept {
  return std::isfinite(sample.yaw) && std::isfinite(sample.pitch) &&
         std::isfinite(sample.yaw_velocity) && std::isfinite(sample.pitch_velocity) &&
         std::isfinite(sample.yaw_acceleration) && std::isfinite(sample.pitch_acceleration);
}

// Clamp one derivative into [-limit, limit] and report how far it overshot.
// A non-positive limit leaves the value untouched.
double clampWithExcess(double &value, const double limit, double &max_excess) noexcept {
  if (!(limit > 0.0) || !std::isfinite(value)) return value;
  const double magnitude = std::abs(value);
  if (magnitude > limit) {
    max_excess = std::max(max_excess, magnitude - limit);
    value = std::copysign(limit, value);
  }
  return value;
}

// Project the yaw/pitch derivative chain of a generated reference into the
// gimbal envelope. Velocity and acceleration are bounded directly; jerk is
// bounded by limiting how fast acceleration may change between samples.
// Positions are then re-integrated from the clamped velocities so the caller
// never sees a position series that disagrees with its own derivatives.
void applyLimits(std::vector<AimReferenceSample> &samples,
                 const AimReferenceLimits &limits,
                 const double dt,
                 const double initial_yaw,
                 const double initial_pitch,
                 bool &exceeded,
                 double &max_excess) noexcept {
  if (samples.empty() || !limits.active()) return;

  double excess = 0.0;
  for (auto &sample : samples) {
    clampWithExcess(sample.yaw_velocity, limits.max_velocity, excess);
    clampWithExcess(sample.pitch_velocity, limits.max_velocity, excess);
    clampWithExcess(sample.yaw_acceleration, limits.max_acceleration, excess);
    clampWithExcess(sample.pitch_acceleration, limits.max_acceleration, excess);
  }

  if (limits.max_jerk > 0.0) {
    const double max_step = limits.max_jerk * dt;
    double previous_yaw_acceleration = samples.front().yaw_acceleration;
    double previous_pitch_acceleration = samples.front().pitch_acceleration;
    for (std::size_t i = 1; i < samples.size(); ++i) {
      auto &sample = samples[i];
      const double yaw_step = sample.yaw_acceleration - previous_yaw_acceleration;
      if (std::abs(yaw_step) > max_step) {
        excess = std::max(excess, std::abs(yaw_step) - max_step);
        sample.yaw_acceleration =
          previous_yaw_acceleration + std::copysign(max_step, yaw_step);
      }
      const double pitch_step = sample.pitch_acceleration - previous_pitch_acceleration;
      if (std::abs(pitch_step) > max_step) {
        excess = std::max(excess, std::abs(pitch_step) - max_step);
        sample.pitch_acceleration =
          previous_pitch_acceleration + std::copysign(max_step, pitch_step);
      }
      previous_yaw_acceleration = sample.yaw_acceleration;
      previous_pitch_acceleration = sample.pitch_acceleration;
    }
  }

  double yaw = initial_yaw;
  double pitch = initial_pitch;
  for (auto &sample : samples) {
    yaw += sample.yaw_velocity * dt;
    pitch += sample.pitch_velocity * dt;
    sample.yaw = yaw;
    sample.pitch = pitch;
  }

  if (excess > 0.0) {
    exceeded = true;
    max_excess = std::max(max_excess, excess);
  }
}
}

double AimReferenceGenerator::unwrapNear(const double angle, const double reference) noexcept {
  if (!std::isfinite(angle) || !std::isfinite(reference)) return std::numeric_limits<double>::quiet_NaN();
  return reference + std::remainder(angle - reference, kTwoPi);
}

AimReferenceResult AimReferenceGenerator::generate(
  const ArmorPlannerInput &input,
  const ArmorPlannerConfig &config,
  const double dt,
  const std::size_t horizon,
  const ArmorTrajectoryPlanner::FlightTimeFunction &flight_time,
  const PitchFunction &pitch,
  const int previous_selected_index,
  const double start_delay,
  const AimReferenceLimits &limits) noexcept {
  AimReferenceResult result;
  if (!std::isfinite(dt) || dt <= 0.0 || horizon < 2 || !pitch ||
      !std::isfinite(start_delay) || start_delay < 0.0) {
    result.reason = "invalid_reference_config";
    return result;
  }

  result.samples.reserve(horizon);
  int selected_index = previous_selected_index;
  const double initial_yaw = input.yaw;
  double previous_yaw = initial_yaw;
  for (std::size_t step = 0; step < horizon; ++step) {
    ArmorPlannerInput sample_input = input;
    const double elapsed = start_delay + static_cast<double>(step + 1) * dt;
    sample_input.center = input.center + elapsed * input.velocity;
    sample_input.yaw = input.yaw + elapsed * input.v_yaw;
    sample_input.target_age_seconds = std::numeric_limits<double>::quiet_NaN();

    const auto planned = ArmorTrajectoryPlanner::plan(
      sample_input, config, flight_time, selected_index);
    if (!planned.converged || planned.armors.empty()) {
      result.reason = planned.reason.empty() ? "planner_failed" : planned.reason;
      return result;
    }
    if (!planned.safe) {
      result.selection_discontinuous = true;
    }

    selected_index = static_cast<int>(planned.selected_index);
    const auto &position = planned.armors.at(planned.selected_index).position;
    AimReferenceSample sample;
    sample.yaw = unwrapNear(std::atan2(position.y(), position.x()), previous_yaw);
    if (!pitch(position, sample.pitch) || !finiteSample(sample)) {
      result.reason = "pitch_compensation_failed";
      return result;
    }
    sample.armor_index = planned.selected_index;
    if (!result.samples.empty()) {
      const auto &previous = result.samples.back();
      if (std::abs(sample.yaw - previous.yaw) > kMaxReferenceStep ||
          std::abs(sample.pitch - previous.pitch) > kMaxReferenceStep) {
        result.selection_discontinuous = true;
      }
    }
    result.samples.push_back(sample);
    previous_yaw = sample.yaw;
  }

  for (std::size_t i = 0; i < result.samples.size(); ++i) {
    const auto previous = i == 0 ? initial_yaw : result.samples[i - 1].yaw;
    const auto next = i + 1 < result.samples.size() ? result.samples[i + 1].yaw : result.samples[i].yaw;
    const auto previous_pitch = i == 0 ? result.samples[i].pitch : result.samples[i - 1].pitch;
    const auto next_pitch = i + 1 < result.samples.size() ? result.samples[i + 1].pitch : result.samples[i].pitch;
    result.samples[i].yaw_velocity = (next - previous) / (i == 0 || i + 1 == result.samples.size() ? dt : 2.0 * dt);
    result.samples[i].pitch_velocity =
      (next_pitch - previous_pitch) / (i == 0 || i + 1 == result.samples.size() ? dt : 2.0 * dt);
    if (!finiteSample(result.samples[i])) {
      result.reason = "nonfinite_reference";
      return result;
    }
  }

  for (std::size_t i = 0; i < result.samples.size(); ++i) {
    const double previous_yaw_velocity = i == 0 ? result.samples[i].yaw_velocity
                                                 : result.samples[i - 1].yaw_velocity;
    const double next_yaw_velocity = i + 1 < result.samples.size()
                                       ? result.samples[i + 1].yaw_velocity
                                       : result.samples[i].yaw_velocity;
    const double previous_pitch_velocity = i == 0 ? result.samples[i].pitch_velocity
                                                   : result.samples[i - 1].pitch_velocity;
    const double next_pitch_velocity = i + 1 < result.samples.size()
                                         ? result.samples[i + 1].pitch_velocity
                                         : result.samples[i].pitch_velocity;
    const double denominator = (i == 0 || i + 1 == result.samples.size()) ? dt : 2.0 * dt;
    result.samples[i].yaw_acceleration =
      (next_yaw_velocity - previous_yaw_velocity) / denominator;
    result.samples[i].pitch_acceleration =
      (next_pitch_velocity - previous_pitch_velocity) / denominator;
    if (!finiteSample(result.samples[i])) {
      result.reason = "nonfinite_reference";
      return result;
    }
  }

  // The first sample's pitch is the closest available stand-in for the current
  // pitch, which the planner input does not carry.
  const double initial_pitch = result.samples.front().pitch -
    result.samples.front().pitch_velocity * dt;
  applyLimits(result.samples, limits, dt, initial_yaw, initial_pitch,
              result.exceeded_limits, result.max_limit_excess);
  for (const auto &sample : result.samples) {
    if (!finiteSample(sample)) {
      result.reason = "nonfinite_reference";
      return result;
    }
  }

  result.valid = true;
  // A clamped reference is trackable but no longer aims at the armour, so it
  // must not be treated as a firing solution.
  result.safe = !result.selection_discontinuous && !result.exceeded_limits;
  result.reason = result.selection_discontinuous ? "selection_discontinuity"
    : (result.exceeded_limits ? "reference_exceeds_gimbal_limits" : "ok");
  return result;
}

}  // namespace fyt::auto_aim
