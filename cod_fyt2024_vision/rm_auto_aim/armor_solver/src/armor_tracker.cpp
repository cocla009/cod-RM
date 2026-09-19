// Copyright Chen Jun 2023. Licensed under the MIT License.
//
// Additional modifications and features by Chengfu Zou, Labor. Licensed under Apache License 2.0.
//
// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "armor_solver/armor_tracker.hpp"
// std
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
// ros2
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// project
#include "rm_utils/logger/log.hpp"
#include "armor_solver/tracker_logic.hpp"

namespace fyt::auto_aim {
Tracker::Tracker(double max_match_distance, double max_match_yaw_diff)
: tracker_state(LOST)
, tracked_id(std::string(""))
, tracked_armors_num(ArmorsNum::NORMAL_4)
, measurement(Eigen::VectorXd::Zero(4))
, target_state(Eigen::VectorXd::Zero(TargetEkfModel::kStateSize))
, info_position_diff(std::numeric_limits<double>::infinity())
, info_yaw_diff(std::numeric_limits<double>::infinity())
, dz(0.0)
, another_r(0.0)
, max_match_distance_(max_match_distance)
, max_match_yaw_diff_(max_match_yaw_diff)
, detect_count_(0)
, lost_count_(0)
, active_armor_index_(0) {}

void Tracker::init(const Armors::SharedPtr &armors_msg) noexcept {
  if (armors_msg->armors.empty()) {
    return;
  }

  // Simply choose the armor that is closest to image center
  double min_distance = DBL_MAX;
  tracked_armor = armors_msg->armors[0];
  for (const auto &armor : armors_msg->armors) {
    if (armor.distance_to_image_center < min_distance) {
      min_distance = armor.distance_to_image_center;
      tracked_armor = armor;
    }
  }

  tracked_id = tracked_armor.number;
  updateArmorCount(tracked_armor);
  active_armor_index_ = 0;
  initEKF(tracked_armor);
  detect_count_ = 0;
  lost_count_ = 0;
  ekf.resetInnovationHistory();
  FYT_INFO("armor_solver", "Init EKF!");

  tracker_state = DETECTING;
}

void Tracker::update(const Armors::SharedPtr &armors_msg) noexcept {
  // KF predict
  Eigen::VectorXd ekf_prediction = ekf.predict();

  bool matched = false;
  // Use KF prediction as default target state if no matched armor is found
  target_state = ekf_prediction;

  if (!armors_msg->armors.empty()) {
    Armor matched_armor;
    int matched_index = 0;
    double matched_yaw = 0.0;
    double min_position_diff = std::numeric_limits<double>::infinity();
    double yaw_diff = std::numeric_limits<double>::infinity();
    const bool has_candidate = findBestMatch(
      armors_msg, matched_armor, matched_index, matched_yaw, min_position_diff, yaw_diff);

    info_position_diff = min_position_diff;
    info_yaw_diff = yaw_diff;

    if (has_candidate && min_position_diff < max_match_distance_ &&
        yaw_diff < max_match_yaw_diff_) {
      const int previous_count = armorCount();
      const int observed_count = tracker_logic::armorCountFor(
        matched_armor.number, matched_armor.type);
      const bool count_changed = observed_count != previous_count;
      tracked_armor = matched_armor;
      if (count_changed) {
        updateArmorCount(matched_armor);
        active_armor_index_ = tracker_logic::remapArmorIndex(
          matched_yaw, target_state(YAW), observed_count);
      } else {
        active_armor_index_ = matched_index;
      }
      auto p = tracked_armor.pose.position;
      if (setSphericalMeasurement(p, matched_yaw)) {
        matched = true;
        if (count_changed) {
          // A geometry change invalidates the old index and velocity mapping.
          handleArmorJump(matched_armor, active_armor_index_, matched_yaw);
          ekf.resetInnovationHistory();
        } else {
          target_state = ekf.update(measurement);
        }
        if (ekf.isDiverged()) {
          FYT_WARN("armor_solver", "EKF innovation statistics diverged, resetting tracker");
          tracker_state = LOST;
          ekf.resetInnovationHistory();
          return;
        }
      } else {
        FYT_WARN("armor_solver", "Rejecting singular spherical armor measurement");
      }
    } else if (has_candidate && yaw_diff > max_match_yaw_diff_) {
      const int previous_count = armorCount();
      const int observed_count = tracker_logic::armorCountFor(
        matched_armor.number, matched_armor.type);
      const bool count_changed = observed_count != previous_count;
      const auto p = matched_armor.pose.position;
      if (setSphericalMeasurement(p, matched_yaw)) {
        tracked_armor = matched_armor;
        if (count_changed) {
          updateArmorCount(matched_armor);
          active_armor_index_ = tracker_logic::remapArmorIndex(
            matched_yaw, target_state(YAW), observed_count);
        } else {
          active_armor_index_ = matched_index;
        }
        handleArmorJump(matched_armor, active_armor_index_, matched_yaw);
        ekf.resetInnovationHistory();
        matched = true;
      } else {
        FYT_WARN("armor_solver", "Rejecting singular spherical armor measurement");
      }
    } else {
      FYT_WARN("armor_solver", "No matched armor found!");
    }
  }

  // Keep the geometric parameters within physical bounds.
  bool state_clamped = false;
  if (target_state(R1) < 0.12) {
    target_state(R1) = 0.12;
    state_clamped = true;
  } else if (target_state(R1) > 0.27) {
    target_state(R1) = 0.27;
    state_clamped = true;
  }
  if (target_state(DELTA_R) < -0.08) {
    target_state(DELTA_R) = -0.08;
    state_clamped = true;
  } else if (target_state(DELTA_R) > 0.08) {
    target_state(DELTA_R) = 0.08;
    state_clamped = true;
  }
  if (target_state(DZ) < -0.20) {
    target_state(DZ) = -0.20;
    state_clamped = true;
  } else if (target_state(DZ) > 0.20) {
    target_state(DZ) = 0.20;
    state_clamped = true;
  }
  if (state_clamped) {
    ekf.setState(target_state);
  }
  another_r = target_state(R1) + target_state(DELTA_R);
  dz = target_state(DZ);

  // Tracking state machine
  if (tracker_state == DETECTING) {
    if (matched) {
      detect_count_++;
      if (detect_count_ >= std::max(1, tracking_thres)) {
        detect_count_ = 0;
        tracker_state = TRACKING;
        FYT_DEBUG("armor_solver", "Tracker state: TRACKING {}", tracked_id);
      }
    } else {
      detect_count_ = 0;
      tracker_state = LOST;
      FYT_DEBUG("armor_solver", "Tracker state: LOST {}", tracked_id);
    }
  } else if (tracker_state == TRACKING) {
    if (matched) {
      lost_count_ = 0;
    } else {
      tracker_state = TEMP_LOST;
      lost_count_++;
      FYT_DEBUG("armor_solver", "Tracker state: TEMP_LOST {}", tracked_id);
    }
  } else if (tracker_state == TEMP_LOST) {
    if (!matched) {
      lost_count_++;
      if (lost_count_ >= std::max(1, lost_thres)) {
        lost_count_ = 0;
        tracker_state = LOST;
        FYT_DEBUG("armor_solver", "Tracker state: LOST {}", tracked_id);
      }
    } else {
      tracker_state = TRACKING;
      lost_count_ = 0;
      FYT_DEBUG("armor_solver", "Tracker state: TRACKING {}", tracked_id);
    }
  }
}

void Tracker::initEKF(const Armor &a) noexcept {
  double xa = a.pose.position.x;
  double ya = a.pose.position.y;
  double za = a.pose.position.z;
  double yaw = orientationToYaw(a.pose.orientation);

  target_state = Eigen::VectorXd::Zero(TargetEkfModel::kStateSize);
  double r = 0.26;
  double xc = xa + r * cos(yaw);
  double yc = ya + r * sin(yaw);
  dz = 0, another_r = r;
  target_state << xc, 0, yc, 0, za, 0, yaw, 0, r, 0, 0;

  ekf.resetState(target_state);
}

void Tracker::handleArmorJump(const Armor &current_armor,
                              int armor_index,
                              double armor_yaw) noexcept {
  const int count = armorCount();
  if (count < 2 || armor_index < 0 || armor_index >= count) {
    return;
  }
  const bool long_armor = count == 4 && armor_index % 2 == 1;
  const double radius = target_state(R1) + (long_armor ? target_state(DELTA_R) : 0.0);
  const double height = long_armor ? target_state(DZ) : 0.0;
  const double base_yaw = TargetEkfModel::normalizeAngle(
    armor_yaw - armor_index * 2.0 * M_PI / static_cast<double>(count));
  auto p = current_armor.pose.position;
  target_state(XC) = p.x + radius * std::cos(armor_yaw);
  target_state(VXC) = 0;
  target_state(YC) = p.y + radius * std::sin(armor_yaw);
  target_state(VYC) = 0;
  target_state(Z) = p.z - height;
  target_state(VZ) = 0;
  target_state(YAW) = base_yaw;
  target_state(VYAW) = 0;
  active_armor_index_ = armor_index;
  ekf.setState(target_state);
  FYT_DEBUG("armor_solver", "Armor jump re-anchored at index {}", armor_index);
}

double Tracker::orientationToYaw(const geometry_msgs::msg::Quaternion &q) noexcept {
  // Get armor yaw
  tf2::Quaternion tf_q;
  tf2::fromMsg(q, tf_q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  return TargetEkfModel::normalizeAngle(yaw);
}

Eigen::Vector3d Tracker::getArmorPositionFromState(const Eigen::VectorXd &x,
                                                  int armor_index,
                                                  int armor_count) noexcept {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  TargetEkfModel::armorPosition(x, armor_index, armor_count, position);
  return position;
}

int Tracker::activeArmorIndex() const noexcept { return active_armor_index_; }

int Tracker::armorCount() const noexcept {
  return static_cast<int>(tracked_armors_num);
}

void Tracker::updateArmorCount(const Armor &armor) noexcept {
  if (armor.type == "large" &&
      (armor.number == "3" || armor.number == "4" || armor.number == "5")) {
    tracked_armors_num = ArmorsNum::BALANCE_2;
  } else if (armor.number == "outpost") {
    tracked_armors_num = ArmorsNum::OUTPOST_3;
  } else {
    tracked_armors_num = ArmorsNum::NORMAL_4;
  }
}

bool Tracker::setSphericalMeasurement(const geometry_msgs::msg::Point &position,
                                      double armor_yaw) noexcept {
  const Eigen::Vector3d cartesian(position.x, position.y, position.z);
  Eigen::Vector3d spherical;
  if (!TargetEkfModel::cartesianToSpherical(cartesian, spherical) || !std::isfinite(armor_yaw)) {
    return false;
  }

  measurement << spherical, TargetEkfModel::normalizeAngle(armor_yaw);
  return measurement.allFinite();
}

bool Tracker::findBestMatch(const Armors::SharedPtr &armors_msg,
                            Armor &matched_armor,
                            int &matched_index,
                            double &matched_yaw,
                            double &position_diff,
                            double &yaw_diff) noexcept {
  if (!armors_msg || armors_msg->armors.empty()) {
    return false;
  }

  const int count = armorCount();
  bool found = false;
  std::size_t observation_index = 0;
  tracker_logic::MatchCandidate best_candidate;
  for (const auto &armor : armors_msg->armors) {
    if (armor.number != tracked_id) {
      ++observation_index;
      continue;
    }

    const auto p = armor.pose.position;
    const Eigen::Vector3d measured_position(p.x, p.y, p.z);
    const double measured_yaw = orientationToYaw(armor.pose.orientation);
    for (int index = 0; index < count; ++index) {
      const Eigen::Vector3d predicted_position =
        getArmorPositionFromState(target_state, index, count);
      const double predicted_yaw = TargetEkfModel::normalizeAngle(
        target_state(YAW) + index * 2.0 * M_PI / static_cast<double>(count));
      const double candidate_position_diff = (predicted_position - measured_position).norm();
      const double candidate_yaw_diff =
        tracker_logic::normalizedYawResidual(measured_yaw, predicted_yaw);
      const double candidate_cost = candidate_position_diff / std::max(max_match_distance_, 1e-6) +
                                    candidate_yaw_diff / std::max(max_match_yaw_diff_, 1e-6);
      const tracker_logic::MatchCandidate candidate{
        observation_index, index, candidate_cost, candidate_position_diff, candidate_yaw_diff};

      if (tracker_logic::betterMatch(candidate, best_candidate)) {
        best_candidate = candidate;
        matched_armor = armor;
        matched_index = index;
        matched_yaw = measured_yaw;
        position_diff = candidate_position_diff;
        yaw_diff = candidate_yaw_diff;
        found = true;
      }
    }
    ++observation_index;
  }
  return found;
}

}  // namespace fyt::auto_aim
