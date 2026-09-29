// Second, independent fire gate: is the *measured* gimbal actually pointing at
// the armour?
//
// The planner's own decision (Plan::fire) only knows whether the planned
// trajectory is still tracking the reference. It is computed in feed-forward
// from the reference itself, so it cannot detect a lower controller that is
// lagging, saturated, or ignoring the acceleration feed-forward. The reference
// implementation ships the same weakness in its MPC entry point, while its
// analytic path additionally checks the measured gimbal angle. We keep that
// check as an AND term.
//
// Extracted from the previous Solver::isOnTarget so the tolerance behaviour is
// unchanged.

#ifndef ARMOR_SOLVER_FIRE_GATE_HPP_
#define ARMOR_SOLVER_FIRE_GATE_HPP_

#include <algorithm>
#include <cmath>

#include <angles/angles.h>

namespace fyt::auto_aim {

constexpr double SMALL_ARMOR_HALF_WIDTH = 0.133 / 2.0;
constexpr double LARGE_ARMOR_HALF_WIDTH = 0.225 / 2.0;

struct FireGateParams {
  double shooting_range_h = 0.135;
  double fire_margin = 0.8;
  double min_fire_tolerance_rad = 1.0 * M_PI / 180.0;
  double max_fire_tolerance_rad = 4.0 * M_PI / 180.0;
};

// Angular tolerance within which a shot still strikes the plate. The plate's
// apparent width shrinks as it turns away, so the tolerance scales with
// distance and is clamped at both ends.
inline double yawTolerance(const double distance,
                           const size_t armors_num,
                           const FireGateParams &params) noexcept {
  const double half_width =
    armors_num == 2 ? LARGE_ARMOR_HALF_WIDTH : SMALL_ARMOR_HALF_WIDTH;
  const double margin = std::max(0.1, params.fire_margin);
  return std::clamp(std::atan2(half_width, distance) * margin,
                    params.min_fire_tolerance_rad,
                    params.max_fire_tolerance_rad);
}

inline double pitchTolerance(const double distance, const FireGateParams &params) noexcept {
  const double margin = std::max(0.1, params.fire_margin);
  return std::clamp(
    std::atan2(std::max(0.001, params.shooting_range_h * 0.5), distance) * margin,
    params.min_fire_tolerance_rad,
    params.max_fire_tolerance_rad);
}

// cur_*: measured gimbal angles. target_*: angles the plan wants to be at.
inline bool isAimOnTarget(const double cur_yaw,
                          const double cur_pitch,
                          const double target_yaw,
                          const double target_pitch,
                          const double distance,
                          const size_t armors_num,
                          const FireGateParams &params) noexcept {
  if (!std::isfinite(distance) || distance <= 0.0) {
    return false;
  }
  if (!std::isfinite(cur_yaw) || !std::isfinite(cur_pitch) ||
      !std::isfinite(target_yaw) || !std::isfinite(target_pitch)) {
    return false;
  }

  return std::abs(angles::shortest_angular_distance(cur_yaw, target_yaw)) <
           yawTolerance(distance, armors_num, params) &&
         std::abs(cur_pitch - target_pitch) < pitchTolerance(distance, params);
}

}  // namespace fyt::auto_aim

#endif  // ARMOR_SOLVER_FIRE_GATE_HPP_
