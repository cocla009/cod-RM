#ifndef ARMOR_SOLVER_TRAJECTORY_PLANNER_HPP_
#define ARMOR_SOLVER_TRAJECTORY_PLANNER_HPP_

#include <cstddef>
#include <functional>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace fyt::auto_aim {

struct ArmorPlannerInput {
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  double yaw = 0.0;
  double v_yaw = 0.0;
  double radius_1 = 0.0;
  double radius_2 = 0.0;
  double dz = 0.0;
  std::size_t armors_num = 0;
  // NaN means that the producer has no trustworthy timestamp.
  double target_age_seconds = std::numeric_limits<double>::quiet_NaN();
};

struct ArmorPlannerConfig {
  double bullet_speed = 20.0;
  double processing_delay = 0.0;
  double max_target_age = 1.0;
  double max_tracking_v_yaw = 6.0;
  double min_switching_v_yaw = 1.0;
  double coming_angle = 55.0 * 3.14159265358979323846 / 180.0;
  double leaving_angle = 20.0 * 3.14159265358979323846 / 180.0;
  double convergence_tolerance = 1e-3;
  int max_iterations = 10;
  double max_selection_jump_rad = 1.5707963267948966;
};

struct ArmorCandidate {
  std::size_t index = 0;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  double delta_angle = 0.0;
};

struct ArmorPlannerResult {
  bool safe = false;
  bool converged = false;
  bool target_expired = false;
  bool selection_discontinuous = false;
  std::string reason;
  int iterations = 0;
  double flight_time = 0.0;
  double total_delay = 0.0;
  Eigen::Vector3d predicted_center = Eigen::Vector3d::Zero();
  double predicted_yaw = 0.0;
  std::vector<ArmorCandidate> armors;
  std::size_t selected_index = 0;
  double selected_delta_angle = 0.0;
};

class ArmorTrajectoryPlanner {
public:
  using FlightTimeFunction = std::function<double(const Eigen::Vector3d &)>;

  static ArmorPlannerResult plan(const ArmorPlannerInput &input,
                                 const ArmorPlannerConfig &config,
                                 const FlightTimeFunction &flight_time = {} ,
                                 int previous_selected_index = -1) noexcept;

  static double normalizeAngle(double angle) noexcept;

private:
  static bool validInput(const ArmorPlannerInput &input,
                         const ArmorPlannerConfig &config) noexcept;
  static std::vector<ArmorCandidate> makeArmors(const Eigen::Vector3d &center,
                                                double yaw,
                                                const ArmorPlannerInput &input) noexcept;
};

}  // namespace fyt::auto_aim

#endif  // ARMOR_SOLVER_TRAJECTORY_PLANNER_HPP_
