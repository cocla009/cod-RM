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
  const double start_delay) noexcept {
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

  result.valid = true;
  result.safe = !result.selection_discontinuous;
  result.reason = result.safe ? "ok" : "selection_discontinuity";
  return result;
}

}  // namespace fyt::auto_aim
