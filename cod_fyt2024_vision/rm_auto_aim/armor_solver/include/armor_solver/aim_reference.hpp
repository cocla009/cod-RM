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

struct AimReferenceResult {
  bool valid = false;
  bool safe = false;
  bool selection_discontinuous = false;
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
                                     int previous_selected_index = -1) noexcept;

  static double unwrapNear(double angle, double reference) noexcept;
};

}  // namespace fyt::auto_aim

#endif  // ARMOR_SOLVER_AIM_REFERENCE_HPP_
