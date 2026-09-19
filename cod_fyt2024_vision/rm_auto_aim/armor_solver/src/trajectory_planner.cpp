#include "armor_solver/trajectory_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fyt::auto_aim {
namespace {
constexpr double kTwoPi = 2.0 * 3.14159265358979323846;

bool finitePositive(const double value) noexcept {
  return std::isfinite(value) && value > 0.0;
}
}  // namespace

double ArmorTrajectoryPlanner::normalizeAngle(const double angle) noexcept {
  return std::remainder(angle, kTwoPi);
}

bool ArmorTrajectoryPlanner::validInput(const ArmorPlannerInput &input,
                                        const ArmorPlannerConfig &config) noexcept {
  if (!input.center.allFinite() || !input.velocity.allFinite() ||
      !std::isfinite(input.yaw) || !std::isfinite(input.v_yaw) ||
      !std::isfinite(input.radius_1) || !std::isfinite(input.radius_2) ||
      !std::isfinite(input.dz) || input.radius_1 <= 0.0 ||
      (input.armors_num == 4 && input.radius_2 <= 0.0) || input.armors_num < 2 ||
      input.armors_num > 4 ||
      !finitePositive(config.bullet_speed) || !std::isfinite(config.processing_delay) ||
      config.processing_delay < 0.0 || !std::isfinite(config.max_target_age) ||
      config.max_target_age < 0.0 || !std::isfinite(config.max_tracking_v_yaw) ||
      config.max_tracking_v_yaw < 0.0 || !std::isfinite(config.min_switching_v_yaw) ||
      config.min_switching_v_yaw < 0.0 || !std::isfinite(config.coming_angle) ||
      !std::isfinite(config.leaving_angle) || config.leaving_angle < 0.0 ||
      config.leaving_angle > config.coming_angle || config.coming_angle > 3.14159265358979323846 ||
      config.convergence_tolerance <= 0.0 || config.max_iterations <= 0 ||
      !std::isfinite(config.max_selection_jump_rad) || config.max_selection_jump_rad < 0.0) {
    return false;
  }
  return !std::isfinite(input.target_age_seconds) ||
         (input.target_age_seconds >= 0.0 && input.target_age_seconds <= config.max_target_age);
}

std::vector<ArmorCandidate> ArmorTrajectoryPlanner::makeArmors(
  const Eigen::Vector3d &center,
  const double yaw,
  const ArmorPlannerInput &input) noexcept {
  std::vector<ArmorCandidate> armors;
  armors.reserve(input.armors_num);
  const double step = kTwoPi / static_cast<double>(input.armors_num);
  for (std::size_t index = 0; index < input.armors_num; ++index) {
    const double armor_yaw = yaw + static_cast<double>(index) * step;
    const bool long_armor = input.armors_num == 4 && index % 2 == 1;
    const double radius = long_armor ? input.radius_2 : input.radius_1;
    const double height = input.armors_num == 4 ? (long_armor ? input.dz : 0.0) : input.dz;
    ArmorCandidate candidate;
    candidate.index = index;
    candidate.position = center + Eigen::Vector3d(-radius * std::cos(armor_yaw),
                                                    -radius * std::sin(armor_yaw), height);
    if (!candidate.position.allFinite()) {
      return {};
    }
    armors.push_back(candidate);
  }
  return armors;
}

ArmorPlannerResult ArmorTrajectoryPlanner::plan(const ArmorPlannerInput &input,
                                                const ArmorPlannerConfig &config,
                                                const FlightTimeFunction &flight_time,
                                                const int previous_selected_index) noexcept {
  ArmorPlannerResult result;
  if (!validInput(input, config)) {
    result.target_expired = std::isfinite(input.target_age_seconds) &&
                            (input.target_age_seconds < 0.0 ||
                             input.target_age_seconds > config.max_target_age);
    result.reason = result.target_expired ? "target_expired" : "invalid_input";
    return result;
  }

  const auto get_time = flight_time ? flight_time : [speed = config.bullet_speed](
                                                        const Eigen::Vector3d &position) {
    return position.norm() / speed;
  };
  double flying_time = get_time(input.center);
  if (!finitePositive(flying_time)) {
    result.reason = "invalid_flight_time";
    return result;
  }

  for (int iteration = 0; iteration < config.max_iterations; ++iteration) {
    const double total_delay = config.processing_delay + flying_time;
    if (!std::isfinite(total_delay) || total_delay < 0.0) {
      result.reason = "invalid_delay";
      return result;
    }
    result.predicted_center = input.center + total_delay * input.velocity;
    result.predicted_yaw = normalizeAngle(input.yaw + total_delay * input.v_yaw);
    result.armors = makeArmors(result.predicted_center, result.predicted_yaw, input);
    if (result.armors.empty()) {
      result.reason = "no_valid_armors";
      return result;
    }

    const auto nearest = std::min_element(
      result.armors.begin(), result.armors.end(), [](const ArmorCandidate &lhs, const ArmorCandidate &rhs) {
        return lhs.position.head<2>().squaredNorm() < rhs.position.head<2>().squaredNorm();
      });
    const double updated_time = get_time(nearest->position);
    result.iterations = iteration + 1;
    if (!finitePositive(updated_time)) {
      result.reason = "invalid_flight_time";
      return result;
    }
    if (std::abs(updated_time - flying_time) <= config.convergence_tolerance) {
      flying_time = updated_time;
      result.converged = true;
      break;
    }
    flying_time = updated_time;
  }

  result.flight_time = flying_time;
  result.total_delay = config.processing_delay + flying_time;
  result.predicted_center = input.center + result.total_delay * input.velocity;
  result.predicted_yaw = normalizeAngle(input.yaw + result.total_delay * input.v_yaw);
  result.armors = makeArmors(result.predicted_center, result.predicted_yaw, input);
  if (!result.converged) {
    result.reason = "prediction_not_converged";
    return result;
  }

  const double line_of_sight = std::atan2(result.predicted_center.y(), result.predicted_center.x());
  for (auto &armor : result.armors) {
    const double armor_yaw = result.predicted_yaw + armor.index * kTwoPi / input.armors_num;
    armor.delta_angle = normalizeAngle(armor_yaw - line_of_sight);
  }

  std::size_t selected = 0;
  if (std::abs(input.v_yaw) < config.min_switching_v_yaw) {
    for (std::size_t index = 1; index < result.armors.size(); ++index) {
      if (std::abs(result.armors[index].delta_angle) <
          std::abs(result.armors[selected].delta_angle)) {
        selected = index;
      }
    }
    if (previous_selected_index >= 0 &&
        previous_selected_index < static_cast<int>(result.armors.size())) {
      const auto previous = static_cast<std::size_t>(previous_selected_index);
      if (std::abs(result.armors[previous].delta_angle) < 3.14159265358979323846 / 3.0 &&
          std::abs(result.armors[previous].delta_angle) -
              std::abs(result.armors[selected].delta_angle) < 3.14159265358979323846 / 6.0) {
        selected = previous;
      }
    }
  } else {
    const bool positive = input.v_yaw > 0.0;
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < result.armors.size(); ++index) {
      const double delta = result.armors[index].delta_angle;
      const bool in_zone = positive ? (delta > -config.coming_angle && delta < config.leaving_angle)
                                    : (delta > -config.leaving_angle && delta < config.coming_angle);
      if (in_zone && std::abs(delta) < best) {
        best = std::abs(delta);
        selected = index;
      }
    }
    if (!std::isfinite(best)) {
      for (std::size_t index = 1; index < result.armors.size(); ++index) {
        if (std::abs(result.armors[index].delta_angle) <
            std::abs(result.armors[selected].delta_angle)) {
          selected = index;
        }
      }
    }
  }

  result.selected_index = selected;
  result.selected_delta_angle = result.armors[selected].delta_angle;
  if (previous_selected_index >= 0 &&
      previous_selected_index < static_cast<int>(result.armors.size()) &&
      previous_selected_index != static_cast<int>(selected)) {
    const double jump = std::abs(normalizeAngle(
      result.armors[selected].delta_angle -
      result.armors[static_cast<std::size_t>(previous_selected_index)].delta_angle));
    result.selection_discontinuous = jump > config.max_selection_jump_rad;
  }
  result.safe = !result.selection_discontinuous;
  result.reason = result.safe ? "ok" : "selection_discontinuity";
  return result;
}

}  // namespace fyt::auto_aim
