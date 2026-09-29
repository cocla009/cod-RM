// Gimbal trajectory planner.
//
// Algorithm ported from sp_vision_25 tasks/auto_aim/planner/planner.{hpp,cpp}
// https://github.com/TongjiSuperPower/sp_vision_25 (MIT, see thirdparty/LICENSE-TinyMPC)
// with two deliberate deviations, marked [deviation] below.

#ifndef ARMOR_SOLVER_PLANNER_HPP_
#define ARMOR_SOLVER_PLANNER_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "rm_interfaces/msg/target.hpp"

namespace fyt::auto_aim {

// All angles in radians, all rates per second. Pitch follows the vehicle
// convention used throughout this package: positive is upward.
struct Plan {
  bool control = false;
  bool fire = false;
  double target_yaw = 0.0;
  double target_pitch = 0.0;
  double yaw = 0.0;
  double yaw_vel = 0.0;
  double yaw_acc = 0.0;
  double pitch = 0.0;
  double pitch_vel = 0.0;
  double pitch_acc = 0.0;
  // Horizontal distance to the armour the plan is aimed at, for diagnostics.
  double distance = 0.0;
};

class Planner {
public:
  // Takes a reference rather than a SharedPtr so the owning node can build the
  // planner from its own constructor, where shared_from_this() is not yet valid.
  explicit Planner(rclcpp::Node &node);
  ~Planner();

  Planner(const Planner &) = delete;
  Planner &operator=(const Planner &) = delete;

  // target_age: seconds between the target's stamp and the current time.
  // [deviation] The reference implementation's Target always carries "now"
  // state, so it needs no age. Ours arrives as a stamped message, so the state
  // must be advanced by the age plus the configured pipeline delay before the
  // prediction window is built. Omitting it introduces a systematic lead error
  // proportional to the target's angular rate.
  Plan plan(const rm_interfaces::msg::Target &target,
            double target_age,
            double bullet_speed);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace fyt::auto_aim

#endif  // ARMOR_SOLVER_PLANNER_HPP_
