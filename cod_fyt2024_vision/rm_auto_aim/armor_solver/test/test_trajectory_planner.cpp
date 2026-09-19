#include "armor_solver/trajectory_planner.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <limits>

using fyt::auto_aim::ArmorPlannerConfig;
using fyt::auto_aim::ArmorPlannerInput;
using fyt::auto_aim::ArmorTrajectoryPlanner;

namespace {
ArmorPlannerInput input(std::size_t count) {
  ArmorPlannerInput value;
  value.center = Eigen::Vector3d(3.0, 0.2, 0.5);
  value.velocity = Eigen::Vector3d(0.1, -0.05, 0.0);
  value.yaw = 3.13;
  value.v_yaw = 0.4;
  value.radius_1 = 0.22;
  value.radius_2 = 0.26;
  value.dz = 0.12;
  value.armors_num = count;
  return value;
}
}

TEST(TrajectoryPlanner, Regression) {
  ArmorPlannerConfig config;
  auto result = ArmorTrajectoryPlanner::plan(input(4), config);
  assert(result.safe && result.converged);
  assert(result.armors.size() == 4 && result.selected_index < 4);
  assert(std::abs(ArmorTrajectoryPlanner::normalizeAngle(2.0 * M_PI + 0.02)) < 0.02);

  for (std::size_t count : {2U, 3U, 4U}) {
    auto geometry = ArmorTrajectoryPlanner::plan(input(count), config);
    assert(geometry.armors.size() == count);
    for (std::size_t index = 0; index < count; ++index) {
      assert(geometry.armors[index].index == index);
      assert(geometry.armors[index].position.allFinite());
    }
  }

  auto expired = input(4);
  expired.target_age_seconds = 2.0;
  auto expired_result = ArmorTrajectoryPlanner::plan(expired, config);
  assert(!expired_result.safe && expired_result.target_expired);

  auto future = input(4);
  future.target_age_seconds = -0.01;
  auto future_result = ArmorTrajectoryPlanner::plan(future, config);
  assert(!future_result.safe && future_result.target_expired);

  auto invalid = input(5);
  auto invalid_result = ArmorTrajectoryPlanner::plan(invalid, config);
  assert(!invalid_result.safe && invalid_result.reason == "invalid_input");

  auto nonfinite = input(4);
  nonfinite.center.x() = std::numeric_limits<double>::quiet_NaN();
  auto nonfinite_result = ArmorTrajectoryPlanner::plan(nonfinite, config);
  assert(!nonfinite_result.safe && nonfinite_result.reason == "invalid_input");

  auto bad_flight = ArmorTrajectoryPlanner::plan(
    input(4), config, [](const Eigen::Vector3d &) {
      return std::numeric_limits<double>::quiet_NaN();
    });
  assert(!bad_flight.safe && bad_flight.reason == "invalid_flight_time");

  auto non_convergent = input(4);
  ArmorPlannerConfig strict = config;
  strict.max_iterations = 1;
  strict.convergence_tolerance = 1e-12;
  auto non_convergent_result = ArmorTrajectoryPlanner::plan(non_convergent, strict);
  assert(!non_convergent_result.safe && !non_convergent_result.converged);

  auto discontinuous = input(2);
  discontinuous.v_yaw = 4.0;
  ArmorPlannerConfig discontinuity_config = config;
  discontinuity_config.max_selection_jump_rad = 0.0;
  auto first = ArmorTrajectoryPlanner::plan(discontinuous, discontinuity_config);
  auto second = ArmorTrajectoryPlanner::plan(discontinuous, discontinuity_config, {},
                                             static_cast<int>(1U - first.selected_index));
  assert(second.selected_index != (1U - first.selected_index));
  assert(second.selection_discontinuous);

}
