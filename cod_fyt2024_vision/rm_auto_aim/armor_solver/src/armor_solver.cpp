// Created by Chengfu Zou
// Maintained by Chengfu Zou, Labor
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

#include "armor_solver/armor_solver.hpp"

#include <algorithm>
#include <angles/angles.h>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "armor_solver/aim_correction.hpp"
#include "armor_solver/armor_solver_node.hpp"
#include "rm_utils/logger/log.hpp"

namespace fyt::auto_aim {
namespace {

constexpr int kMaxPredictionIterations = 10;
constexpr double kPredictionConvergence = 1e-3;
constexpr double kMinValidDistance = 0.1;

double normalizeAngle(const double angle) noexcept {
  return std::remainder(angle, 2.0 * M_PI);
}

}  // namespace

Solver::Solver(std::weak_ptr<rclcpp::Node> n) : node_(n) {
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Armor solver node is unavailable");
  }

  shooting_range_h_ = node->declare_parameter("solver.shooting_range_height", 0.135);
  max_tracking_v_yaw_ = node->declare_parameter("solver.max_tracking_v_yaw", 6.0);
  prediction_delay_ = node->declare_parameter("solver.prediction_delay", 0.0);
  controller_delay_ = node->declare_parameter("solver.controller_delay", 0.0);
  min_switching_v_yaw_ = node->declare_parameter("solver.min_switching_v_yaw", 1.0);
  coming_angle_ = node->declare_parameter("solver.coming_angle", 55.0);
  leaving_angle_ = node->declare_parameter("solver.leaving_angle", 20.0);
  center_tracking_distance_ = node->declare_parameter("solver.center_tracking_distance", 1.5);
  yaw_offset_deg_ = node->declare_parameter("solver.yaw_offset", 0.0);
  pitch_offset_deg_ = node->declare_parameter("solver.pitch_offset", 0.0);
  high_yaw_compensation_threshold_ =
    node->declare_parameter("solver.high_yaw_pitch_compensation_threshold", 6.0);
  high_yaw_compensation_reference_ =
    node->declare_parameter("solver.high_yaw_pitch_compensation_reference", 10.0);
  max_high_yaw_pitch_offset_deg_ =
    node->declare_parameter("solver.max_high_yaw_pitch_offset", 1.2);

  fire_margin_ = node->declare_parameter("solver.fire_margin", 0.8);
  const double min_tolerance_deg =
    node->declare_parameter("solver.min_fire_tolerance", 1.0);
  const double max_tolerance_deg =
    node->declare_parameter("solver.max_fire_tolerance", 4.0);
  min_fire_tolerance_rad_ = min_tolerance_deg * M_PI / 180.0;
  max_fire_tolerance_rad_ = std::max(min_fire_tolerance_deg, max_tolerance_deg) * M_PI / 180.0;

  const auto compensator_type = node->declare_parameter("solver.compensator_type", "ideal");
  trajectory_compensator_ = CompensatorFactory::createCompensator(compensator_type);
  if (!trajectory_compensator_) {
    FYT_WARN("armor_solver", "Unknown compensator '{}', falling back to ideal", compensator_type);
    trajectory_compensator_ = std::make_unique<IdealCompensator>();
  }
  trajectory_compensator_->iteration_times = node->declare_parameter("solver.iteration_times", 20);
  trajectory_compensator_->velocity = node->declare_parameter("solver.bullet_speed", 20.0);
  trajectory_compensator_->gravity = node->declare_parameter("solver.gravity", 9.8);
  trajectory_compensator_->resistance = node->declare_parameter("solver.resistance", 0.001);

  state = State::TRACKING_ARMOR;
  overflow_count_ = 0;
  transfer_thresh_ = 5;
}

rm_interfaces::msg::GimbalCmd Solver::solve(const rm_interfaces::msg::Target &target,
                                            const rclcpp::Time &current_time,
                                            std::shared_ptr<tf2_ros::Buffer> tf2_buffer) {
  if (target.armors_num <= 0 || target.armors_num > 8) {
    throw std::runtime_error("Target has an invalid armor count");
  }

  try {
    auto node = node_.lock();
    if (node) {
      max_tracking_v_yaw_ = node->get_parameter("solver.max_tracking_v_yaw").as_double();
      prediction_delay_ = node->get_parameter("solver.prediction_delay").as_double();
      controller_delay_ = node->get_parameter("solver.controller_delay").as_double();
      min_switching_v_yaw_ = node->get_parameter("solver.min_switching_v_yaw").as_double();
      coming_angle_ = node->get_parameter("solver.coming_angle").as_double();
      leaving_angle_ = node->get_parameter("solver.leaving_angle").as_double();
      center_tracking_distance_ =
        node->get_parameter("solver.center_tracking_distance").as_double();
      yaw_offset_deg_ = node->get_parameter("solver.yaw_offset").as_double();
      pitch_offset_deg_ = node->get_parameter("solver.pitch_offset").as_double();
      high_yaw_compensation_threshold_ =
        node->get_parameter("solver.high_yaw_pitch_compensation_threshold").as_double();
      high_yaw_compensation_reference_ =
        node->get_parameter("solver.high_yaw_pitch_compensation_reference").as_double();
      max_high_yaw_pitch_offset_deg_ =
        node->get_parameter("solver.max_high_yaw_pitch_offset").as_double();
      fire_margin_ = node->get_parameter("solver.fire_margin").as_double();
    }
  } catch (const std::runtime_error &e) {
    FYT_ERROR("armor_solver", "{}", e.what());
  }

  std::array<double, 3> rpy{};
  try {
    const auto gimbal_tf =
      tf2_buffer->lookupTransform(target.header.frame_id, "gimbal_link", tf2::TimePointZero);
    tf2::Quaternion tf_q;
    tf2::fromMsg(gimbal_tf.transform.rotation, tf_q);
    tf2::Matrix3x3(tf_q).getRPY(rpy[0], rpy[1], rpy[2]);
    rpy[0] = -rpy[1];
  } catch (const tf2::TransformException &ex) {
    FYT_ERROR("armor_solver", "{}", ex.what());
    throw;
  }

  const Eigen::Vector3d initial_position(target.position.x, target.position.y, target.position.z);
  const Eigen::Vector3d target_velocity(target.velocity.x,
                                        target.velocity.y,
                                        target.velocity.z);
  const double processing_delay =
    std::max(0.0, (current_time - rclcpp::Time(target.header.stamp)).seconds()) +
    std::max(0.0, prediction_delay_) + std::max(0.0, controller_delay_);

  // The selected plate changes the distance, so recompute flight time using the
  // nearest predicted plate until the estimate converges.
  double flying_time = trajectory_compensator_->getFlyingTime(initial_position);
  for (int iteration = 0; iteration < kMaxPredictionIterations; ++iteration) {
    const double total_delay = processing_delay + flying_time;
    const Eigen::Vector3d predicted_center = initial_position + total_delay * target_velocity;
    const double predicted_yaw = target.yaw + total_delay * target.v_yaw;
    const auto predicted_armors = getArmorPositions(predicted_center,
                                                    predicted_yaw,
                                                    target.radius_1,
                                                    target.radius_2,
                                                    target.dz,
                                                    target.armors_num);
    if (predicted_armors.empty()) {
      throw std::runtime_error("Target has no armor positions");
    }

    auto nearest = predicted_armors.front();
    for (const auto &armor : predicted_armors) {
      if (armor.head<2>().squaredNorm() < nearest.head<2>().squaredNorm()) {
        nearest = armor;
      }
    }
    const double new_flying_time = trajectory_compensator_->getFlyingTime(nearest);
    if (std::abs(new_flying_time - flying_time) < kPredictionConvergence) {
      flying_time = new_flying_time;
      break;
    }
    flying_time = new_flying_time;
  }

  const double total_delay = processing_delay + flying_time;
  Eigen::Vector3d target_position = initial_position + total_delay * target_velocity;
  double target_yaw = target.yaw + total_delay * target.v_yaw;
  auto armor_positions = getArmorPositions(
    target_position, target_yaw, target.radius_1, target.radius_2, target.dz, target.armors_num);
  if (armor_positions.empty()) {
    throw std::runtime_error("Target has no armor positions");
  }

  double selected_delta_angle = 0.0;
  const int selected_idx = selectBestArmor(armor_positions,
                                           target_position,
                                           target_yaw,
                                           target.v_yaw,
                                           target.armors_num,
                                           selected_delta_angle);
  Eigen::Vector3d aim_position = armor_positions.at(static_cast<size_t>(selected_idx));
  double yaw = 0.0;
  double pitch = 0.0;
  double distance = aim_position.norm();

  const bool fast_and_near = std::abs(target.v_yaw) > max_tracking_v_yaw_ &&
                             distance < center_tracking_distance_;
  const bool slow_or_far = std::abs(target.v_yaw) <= max_tracking_v_yaw_ ||
                           distance >= center_tracking_distance_;
  if (state == State::TRACKING_ARMOR) {
    overflow_count_ = fast_and_near ? overflow_count_ + 1 : 0;
    if (overflow_count_ > transfer_thresh_) {
      state = State::TRACKING_CENTER;
      overflow_count_ = 0;
    }
  } else {
    overflow_count_ = slow_or_far ? overflow_count_ + 1 : 0;
    if (overflow_count_ > transfer_thresh_) {
      state = State::TRACKING_ARMOR;
      overflow_count_ = 0;
    }
  }

  if (state == State::TRACKING_CENTER) {
    aim_position = target_position;
    selected_delta_angle = 0.0;
  }
  if (aim_position.norm() < kMinValidDistance) {
    throw std::runtime_error("No valid aim position");
  }
  calcYawAndPitch(aim_position, rpy, yaw, pitch);
  applyAimCorrections(target.v_yaw, yaw, pitch);
  distance = aim_position.norm();

  rm_interfaces::msg::GimbalCmd gimbal_cmd;
  gimbal_cmd.header = target.header;
  gimbal_cmd.distance = distance;
  gimbal_cmd.yaw = yaw * 180.0 / M_PI;
  gimbal_cmd.pitch = pitch * 180.0 / M_PI;
  gimbal_cmd.yaw_diff = angles::shortest_angular_distance(rpy[2], yaw) * 180.0 / M_PI;
  gimbal_cmd.pitch_diff = (pitch - rpy[1]) * 180.0 / M_PI;
  gimbal_cmd.fire_advice = isOnTarget(rpy[2],
                                      rpy[1],
                                      yaw,
                                      pitch,
                                      distance,
                                      target.armors_num,
                                      selected_delta_angle);
  if (state == State::TRACKING_CENTER) {
    // Center tracking keeps the gimbal motion continuous but does not identify
    // an actual armor plate that is safe to fire at.
    gimbal_cmd.fire_advice = false;
  }
  return gimbal_cmd;
}

bool Solver::isOnTarget(const double cur_yaw,
                        const double cur_pitch,
                        const double target_yaw,
                        const double target_pitch,
                        const double distance,
                        const size_t armors_num,
                        const double armor_delta_angle) const noexcept {
  if (!std::isfinite(distance) || distance <= 0.0) {
    return false;
  }

  const double half_width = armors_num == 2 ? LARGE_ARMOR_HALF_WIDTH : SMALL_ARMOR_HALF_WIDTH;
  const double projected_half_width = half_width * std::abs(std::cos(armor_delta_angle));
  const double margin = std::max(0.1, fire_margin_);
  const double yaw_tolerance = std::clamp(std::atan2(projected_half_width, distance) * margin,
                                          min_fire_tolerance_rad_,
                                          max_fire_tolerance_rad_);
  const double pitch_tolerance = std::clamp(
    std::atan2(std::max(0.001, shooting_range_h_ * 0.5), distance) * margin,
    min_fire_tolerance_rad_,
    max_fire_tolerance_rad_);

  return std::abs(angles::shortest_angular_distance(cur_yaw, target_yaw)) < yaw_tolerance &&
         std::abs(cur_pitch - target_pitch) < pitch_tolerance;
}

std::vector<Eigen::Vector3d> Solver::getArmorPositions(const Eigen::Vector3d &target_center,
                                                       const double target_yaw,
                                                       const double r1,
                                                       const double r2,
                                                       const double dz,
                                                       const size_t armors_num) const noexcept {
  if (armors_num == 0) {
    return {};
  }

  std::vector<Eigen::Vector3d> positions;
  positions.reserve(armors_num);
  for (size_t i = 0; i < armors_num; ++i) {
    const double armor_yaw = target_yaw + i * (2.0 * M_PI / armors_num);
    const bool first_pair = i % 2 == 0;
    const double radius = armors_num == 4 ? (first_pair ? r1 : r2) : r1;
    const double height = armors_num == 4 ? (first_pair ? 0.0 : dz) : dz;
    positions.emplace_back(target_center + Eigen::Vector3d(-radius * std::cos(armor_yaw),
                                                            -radius * std::sin(armor_yaw),
                                                            height));
  }
  return positions;
}

int Solver::selectBestArmor(const std::vector<Eigen::Vector3d> &armor_positions,
                            const Eigen::Vector3d &target_center,
                            const double target_yaw,
                            const double target_v_yaw,
                            const size_t armors_num,
                            double &selected_delta_angle) noexcept {
  const size_t count = std::min(armors_num, armor_positions.size());
  if (count == 0) {
    selected_delta_angle = 0.0;
    return 0;
  }

  const double line_of_sight = std::atan2(target_center.y(), target_center.x());
  std::vector<double> delta_angles(count);
  for (size_t i = 0; i < count; ++i) {
    delta_angles[i] = normalizeAngle(target_yaw + i * (2.0 * M_PI / count) - line_of_sight);
  }

  int selected = 0;
  if (std::abs(target_v_yaw) < min_switching_v_yaw_) {
    auto best = std::min_element(delta_angles.begin(), delta_angles.end(), [](double lhs, double rhs) {
      return std::abs(lhs) < std::abs(rhs);
    });
    selected = static_cast<int>(std::distance(delta_angles.begin(), best));

    if (lock_id_ >= 0 && lock_id_ < static_cast<int>(count)) {
      const double lock_delta = std::abs(delta_angles[static_cast<size_t>(lock_id_)]);
      const double best_delta = std::abs(delta_angles[static_cast<size_t>(selected)]);
      if (lock_delta < M_PI / 3.0 && lock_delta - best_delta < M_PI / 6.0) {
        selected = lock_id_;
      }
    }
  } else {
    const double coming = coming_angle_ * M_PI / 180.0;
    const double leaving = leaving_angle_ * M_PI / 180.0;
    double best_score = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < count; ++i) {
      const double delta = delta_angles[i];
      const bool in_coming_zone = target_v_yaw > 0.0
                                     ? (delta > -coming && delta < leaving)
                                     : (delta > -leaving && delta < coming);
      if (in_coming_zone && std::abs(delta) < best_score) {
        best_score = std::abs(delta);
        selected = static_cast<int>(i);
      }
    }
    if (!std::isfinite(best_score)) {
      auto best = std::min_element(delta_angles.begin(), delta_angles.end(), [](double lhs, double rhs) {
        return std::abs(lhs) < std::abs(rhs);
      });
      selected = static_cast<int>(std::distance(delta_angles.begin(), best));
    }
  }

  lock_id_ = selected;
  selected_delta_angle = delta_angles[static_cast<size_t>(selected)];
  return selected;
}

void Solver::calcYawAndPitch(const Eigen::Vector3d &p,
                             const std::array<double, 3>,
                             double &yaw,
                             double &pitch) const noexcept {
  yaw = std::atan2(p.y(), p.x());
  pitch = std::atan2(p.z(), p.head<2>().norm());
  double compensated_pitch = pitch;
  if (trajectory_compensator_->compensate(p, compensated_pitch)) {
    pitch = compensated_pitch;
  }
}

void Solver::applyAimCorrections(const double target_v_yaw,
                                 double &yaw,
                                 double &pitch) const noexcept {
  const AimCorrectionParameters parameters{
    yaw_offset_deg_,
    pitch_offset_deg_,
    high_yaw_compensation_threshold_,
    high_yaw_compensation_reference_,
    max_high_yaw_pitch_offset_deg_};
  applyAimCorrection(target_v_yaw, parameters, yaw, pitch);
}

}  // namespace fyt::auto_aim
