#ifndef ARMOR_SOLVER_AIM_REFERENCE_HPP_
#define ARMOR_SOLVER_AIM_REFERENCE_HPP_

#include <functional>
#include <string>
#include <vector>

#include "armor_solver/trajectory_planner.hpp"

namespace fyt::auto_aim {

struct AimReferenceSample {
  double yaw = 0.0;
  double pitch = 0.0;
  double yaw_velocity = 0.0;
  double pitch_velocity = 0.0;
  double yaw_acceleration = 0.0;
  double pitch_acceleration = 0.0;
  std::size_t armor_index = 0;
};

// Gimbal motion envelope used to keep a generated reference trackable. A
// non-positive limit disables clamping for that derivative, which reproduces
// the historical unclamped behaviour.
struct AimReferenceLimits {
  double max_velocity = 0.0;
  double max_acceleration = 0.0;
  double max_jerk = 0.0;

  bool active() const noexcept {
    return max_velocity > 0.0 || max_acceleration > 0.0 || max_jerk > 0.0;
  }
};

struct AimReferenceResult {
  bool valid = false;
  bool safe = false;
  bool selection_discontinuous = false;
  // Set when the raw reference derivatives left the supplied envelope. The
  // samples are clamped back into it, so the gimbal cannot follow the target
  // exactly on these frames and fire control must not trust them.
  bool exceeded_limits = false;
  // Worst raw yaw/pitch derivative overshoot, in the unit of the derivative
  // that saturated. Zero when the reference already fit the envelope.
  double max_limit_excess = 0.0;
  std::string reason;
  std::vector<AimReferenceSample> samples;
};

class AimReferenceGenerator {
public:
  using PitchFunction = std::function<bool(const Eigen::Vector3d &, double &)>;

  static AimReferenceResult generate(const ArmorPlannerInput &input,
                                     const ArmorPlannerConfig &config,
                                     double dt,
                                     std::size_t horizon,
                                     const ArmorTrajectoryPlanner::FlightTimeFunction &flight_time,
                                     const PitchFunction &pitch,
                                     int previous_selected_index = -1,
                                     double start_delay = 0.0,
                                     const AimReferenceLimits &limits = {}) noexcept;

  static double unwrapNear(double angle, double reference) noexcept;
};

}  // namespace fyt::auto_aim

#endif  // ARMOR_SOLVER_AIM_REFERENCE_HPP_
