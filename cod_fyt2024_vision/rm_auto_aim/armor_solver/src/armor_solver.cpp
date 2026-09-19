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
#include "armor_solver/gimbal_command_contract.hpp"
#include "armor_solver/trajectory_planner.hpp"
#include "rm_utils/logger/log.hpp"

namespace fyt::auto_aim {
namespace {

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
  max_fire_tolerance_rad_ = std::max(min_tolerance_deg, max_tolerance_deg) * M_PI / 180.0;

  mpc_enabled_ = node->declare_parameter("solver.mpc.enabled", true);
  mpc_dt_ = node->declare_parameter("solver.mpc.dt", 0.01);
  const int mpc_horizon = node->declare_parameter<int>("solver.mpc.horizon", 40);
  // With no explicit actuator delay, apply the first predicted state. A later
  // preview index must be justified by a measured command transport delay.
  const int mpc_preview_steps = node->declare_parameter<int>("solver.mpc.preview_steps", 0);
  mpc_actuator_delay_ = node->declare_parameter("solver.mpc.actuator_delay", 0.0);
  mpc_horizon_ = static_cast<std::size_t>(std::max(2, mpc_horizon));
  mpc_preview_steps_ = static_cast<std::size_t>(std::max(0, mpc_preview_steps));
  GimbalMpcConfig mpc_config;
  mpc_config.dt = mpc_dt_;
  mpc_config.horizon = mpc_horizon_;
  mpc_config.position_weight = node->declare_parameter("solver.mpc.position_weight", 40.0);
  mpc_config.velocity_weight = node->declare_parameter("solver.mpc.velocity_weight", 4.0);
  mpc_config.acceleration_weight =
    node->declare_parameter("solver.mpc.acceleration_weight", 0.2);
  mpc_config.jerk_weight = node->declare_parameter("solver.mpc.jerk_weight", 0.05);
  mpc_config.terminal_weight = node->declare_parameter("solver.mpc.terminal_weight", 80.0);
  mpc_config.max_velocity = node->declare_parameter("solver.mpc.max_velocity", 12.0);
  mpc_config.max_acceleration = node->declare_parameter("solver.mpc.max_acceleration", 50.0);
  mpc_config.max_jerk = node->declare_parameter("solver.mpc.max_jerk", 500.0);
  mpc_config.step_size = node->declare_parameter("solver.mpc.step_size", 0.002);
  mpc_config.max_iterations = node->declare_parameter("solver.mpc.max_iterations", 12);
  gimbal_mpc_.setConfig(mpc_config);

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

bool Solver::buildMpcReference(const ArmorPlannerInput &planner_input,
                               const ArmorPlannerConfig &planner_config,
                               const std::array<double, 3> &rpy,
                               const Eigen::Vector3d &center_position,
                               const double target_v_yaw,
                               AimReferenceResult &reference) noexcept {
  (void)rpy;
  const auto pitch = [this, target_v_yaw](const Eigen::Vector3d &position, double &value) {
    double yaw = 0.0;
    calcYawAndPitch(position, {}, yaw, value);
    applyAimCorrections(target_v_yaw, yaw, value);
    return std::isfinite(yaw) && std::isfinite(value);
  };

  if (state == State::TRACKING_CENTER) {
    reference.samples.clear();
    reference.samples.reserve(mpc_horizon_);
    for (std::size_t i = 0; i < mpc_horizon_; ++i) {
      const double elapsed = std::max(0.0, mpc_actuator_delay_) +
        static_cast<double>(i + 1) * mpc_dt_;
      const Eigen::Vector3d position = center_position + elapsed * planner_input.velocity;
      AimReferenceSample sample;
      if (!pitch(position, sample.pitch)) {
        reference.reason = "center_pitch_compensation_failed";
        return false;
      }
      double yaw = 0.0;
      calcYawAndPitch(position, {}, yaw, sample.pitch);
      applyAimCorrections(target_v_yaw, yaw, sample.pitch);
      sample.yaw = AimReferenceGenerator::unwrapNear(yaw, i == 0 ? rpy[2] : reference.samples.back().yaw);
      sample.armor_index = 0;
      reference.samples.push_back(sample);
    }
    reference.valid = true;
    reference.safe = true;
  } else {
    reference = AimReferenceGenerator::generate(
      planner_input, planner_config, mpc_dt_, mpc_horizon_,
      [this](const Eigen::Vector3d &position) {
        return trajectory_compensator_->getFlyingTime(position);
      },
      pitch,
      lock_id_,
      std::max(0.0, mpc_actuator_delay_));
  }

  if (!reference.valid || reference.samples.size() != mpc_horizon_) return false;
  for (std::size_t i = 0; i < reference.samples.size(); ++i) {
    const auto previous_yaw = i == 0 ? rpy[2] : reference.samples[i - 1].yaw;
    const auto next_yaw = i + 1 < reference.samples.size()
                            ? reference.samples[i + 1].yaw
                            : reference.samples[i].yaw;
    const auto previous_pitch = i == 0 ? reference.samples[i].pitch : reference.samples[i - 1].pitch;
    const auto next_pitch = i + 1 < reference.samples.size()
                              ? reference.samples[i + 1].pitch
                              : reference.samples[i].pitch;
    const double denominator = (i == 0 || i + 1 == reference.samples.size()) ? mpc_dt_ : 2.0 * mpc_dt_;
    reference.samples[i].yaw_velocity = (next_yaw - previous_yaw) / denominator;
    reference.samples[i].pitch_velocity = (next_pitch - previous_pitch) / denominator;
    if (!std::isfinite(reference.samples[i].yaw) ||
        !std::isfinite(reference.samples[i].pitch) ||
        !std::isfinite(reference.samples[i].yaw_velocity) ||
        !std::isfinite(reference.samples[i].pitch_velocity) ||
        (i > 0 && (std::abs(reference.samples[i].yaw - reference.samples[i - 1].yaw) > M_PI / 2.0 ||
                   std::abs(reference.samples[i].pitch - reference.samples[i - 1].pitch) > M_PI / 2.0))) {
      reference.selection_discontinuous = true;
    }
  }
  reference.safe = reference.safe && !reference.selection_discontinuous;
  return true;
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
  const double raw_age = (current_time - rclcpp::Time(target.header.stamp)).seconds();
  const bool unknown_timestamp = target.header.stamp.sec == 0 && target.header.stamp.nanosec == 0;
  const double target_age = unknown_timestamp ? std::numeric_limits<double>::quiet_NaN() : raw_age;
  const bool invalid_delay = prediction_delay_ < 0.0 || controller_delay_ < 0.0;
  const double processing_delay =
    std::max(0.0, raw_age) + std::max(0.0, prediction_delay_) + std::max(0.0, controller_delay_);

  ArmorPlannerInput planner_input;
  planner_input.center = initial_position;
  planner_input.velocity = target_velocity;
  planner_input.yaw = target.yaw;
  planner_input.v_yaw = target.v_yaw;
  planner_input.radius_1 = target.radius_1;
  planner_input.radius_2 = target.radius_2;
  planner_input.dz = target.dz;
  planner_input.armors_num = target.armors_num;
  planner_input.target_age_seconds = target_age;

  ArmorPlannerConfig planner_config;
  planner_config.processing_delay = processing_delay;
  planner_config.max_tracking_v_yaw = max_tracking_v_yaw_;
  planner_config.min_switching_v_yaw = min_switching_v_yaw_;
  planner_config.coming_angle = coming_angle_ * M_PI / 180.0;
  planner_config.leaving_angle = leaving_angle_ * M_PI / 180.0;
  planner_config.bullet_speed = trajectory_compensator_->velocity;
  const auto planner_result = ArmorTrajectoryPlanner::plan(
    planner_input,
    planner_config,
    [this](const Eigen::Vector3d &position) { return trajectory_compensator_->getFlyingTime(position); },
    lock_id_);

  bool planning_safe = planner_result.safe && !invalid_delay;
  Eigen::Vector3d target_position = planner_result.predicted_center;
  double target_yaw = planner_result.predicted_yaw;
  std::vector<Eigen::Vector3d> armor_positions;
  double selected_delta_angle = 0.0;
  int selected_idx = 0;
  if (planner_result.converged && !planner_result.armors.empty()) {
    armor_positions.reserve(planner_result.armors.size());
    for (const auto &armor : planner_result.armors) {
      armor_positions.push_back(armor.position);
    }
    selected_idx = static_cast<int>(planner_result.selected_index);
    selected_delta_angle = planner_result.selected_delta_angle;
    lock_id_ = selected_idx;
  } else {
    // Keep a deterministic aim point for diagnostics, but never fire when
    // planning failed, the target is stale, or the iteration did not converge.
    planning_safe = false;
    target_position = initial_position;
    target_yaw = target.yaw;
    armor_positions = getArmorPositions(
      target_position, target_yaw, target.radius_1, target.radius_2, target.dz, target.armors_num);
    if (armor_positions.empty()) {
      throw std::runtime_error("Target has no armor positions");
    }
    selected_idx = selectBestArmor(armor_positions,
                                   target_position,
                                   target_yaw,
                                   target.v_yaw,
                                   target.armors_num,
                                   selected_delta_angle);
  }
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

  const double state_dt = gimbal_state_initialized_
                            ? (current_time - previous_gimbal_time_).seconds()
                            : mpc_dt_;
  const bool valid_state_dt = std::isfinite(state_dt) && state_dt > 1e-4 && state_dt < 0.5;
  const double effective_state_dt = valid_state_dt ? state_dt : mpc_dt_;
  const double yaw_velocity = gimbal_state_initialized_
                                ? angles::shortest_angular_distance(previous_gimbal_yaw_, rpy[2]) /
                                    effective_state_dt
                                : 0.0;
  const double pitch_velocity = gimbal_state_initialized_
                                  ? (rpy[1] - previous_gimbal_pitch_) / effective_state_dt
                                  : 0.0;
  const double yaw_acceleration = gimbal_state_initialized_
                                    ? (yaw_velocity - previous_gimbal_yaw_velocity_) /
                                        effective_state_dt
                                    : 0.0;
  const double pitch_acceleration = gimbal_state_initialized_
                                      ? (pitch_velocity - previous_gimbal_pitch_velocity_) /
                                          effective_state_dt
                                      : 0.0;

  double command_yaw = yaw;
  double command_pitch = pitch;
  double command_yaw_velocity = 0.0;
  double command_pitch_velocity = 0.0;
  double command_yaw_acceleration = 0.0;
  double command_pitch_acceleration = 0.0;
  double command_yaw_jerk = 0.0;
  double command_pitch_jerk = 0.0;
  bool mpc_valid = false;
  bool mpc_safe = !mpc_enabled_;
  if (mpc_enabled_) {
    AimReferenceResult reference;
    const bool reference_valid = buildMpcReference(
      planner_input,
      planner_config,
      rpy,
      target_position,
      target.v_yaw,
      reference);
    if (reference_valid) {
      GimbalMpcInput mpc_input;
      mpc_input.yaw = {rpy[2], yaw_velocity, yaw_acceleration};
      mpc_input.pitch = {rpy[1], pitch_velocity, pitch_acceleration};
      mpc_input.yaw_reference.position.reserve(reference.samples.size());
      mpc_input.yaw_reference.velocity.reserve(reference.samples.size());
      mpc_input.yaw_reference.acceleration.reserve(reference.samples.size());
      mpc_input.pitch_reference.position.reserve(reference.samples.size());
      mpc_input.pitch_reference.velocity.reserve(reference.samples.size());
      mpc_input.pitch_reference.acceleration.reserve(reference.samples.size());
      for (const auto &sample : reference.samples) {
        mpc_input.yaw_reference.position.push_back(sample.yaw);
        mpc_input.yaw_reference.velocity.push_back(sample.yaw_velocity);
        mpc_input.yaw_reference.acceleration.push_back(sample.yaw_acceleration);
        mpc_input.pitch_reference.position.push_back(sample.pitch);
        mpc_input.pitch_reference.velocity.push_back(sample.pitch_velocity);
        mpc_input.pitch_reference.acceleration.push_back(sample.pitch_acceleration);
      }
      const auto mpc_result = gimbal_mpc_.solve(mpc_input);
      mpc_valid = mpc_result.valid;
      mpc_safe = mpc_result.valid && mpc_result.converged && reference.safe;
      if (mpc_valid && !mpc_result.yaw.predicted.empty() && !mpc_result.pitch.predicted.empty()) {
        const auto index = std::min(
          mpc_preview_steps_, std::min(mpc_result.yaw.predicted.size(), mpc_result.pitch.predicted.size()) - 1);
        const auto &yaw_command = mpc_result.yaw.predicted[index];
        const auto &pitch_command = mpc_result.pitch.predicted[index];
        command_yaw = yaw_command.position;
        command_yaw_velocity = yaw_command.velocity;
        command_yaw_acceleration = yaw_command.acceleration;
        command_pitch = pitch_command.position;
        command_pitch_velocity = pitch_command.velocity;
        command_pitch_acceleration = pitch_command.acceleration;
        command_yaw_jerk = mpc_result.yaw.control;
        command_pitch_jerk = mpc_result.pitch.control;
      } else {
        mpc_valid = false;
      }
    }
  }

  const bool finite_command = std::isfinite(command_yaw) && std::isfinite(command_pitch) &&
                              std::isfinite(command_yaw_velocity) &&
                              std::isfinite(command_pitch_velocity) &&
                              std::isfinite(command_yaw_acceleration) &&
                              std::isfinite(command_pitch_acceleration) &&
                              std::isfinite(command_yaw_jerk) &&
                              std::isfinite(command_pitch_jerk);
  if (!finite_command) {
    command_yaw = std::isfinite(yaw) ? yaw : rpy[2];
    command_pitch = std::isfinite(pitch) ? pitch : rpy[1];
    command_yaw_velocity = 0.0;
    command_pitch_velocity = 0.0;
    command_yaw_acceleration = 0.0;
    command_pitch_acceleration = 0.0;
    command_yaw_jerk = 0.0;
    command_pitch_jerk = 0.0;
    mpc_valid = false;
    mpc_safe = false;
    planning_safe = false;
  }

  rm_interfaces::msg::GimbalCmd gimbal_cmd;
  gimbal_cmd.header = target.header;
  gimbal_cmd.header.stamp = current_time;
  gimbal_cmd.distance = distance;
  gimbal_cmd.yaw = command_yaw * 180.0 / M_PI;
  gimbal_cmd.pitch = command_pitch * 180.0 / M_PI;
  gimbal_cmd.yaw_diff = angles::shortest_angular_distance(rpy[2], command_yaw) * 180.0 / M_PI;
  gimbal_cmd.pitch_diff = (command_pitch - rpy[1]) * 180.0 / M_PI;
  gimbal_cmd.yaw_velocity = command_yaw_velocity * 180.0 / M_PI;
  gimbal_cmd.yaw_acceleration = command_yaw_acceleration * 180.0 / M_PI;
  gimbal_cmd.pitch_velocity = command_pitch_velocity * 180.0 / M_PI;
  gimbal_cmd.pitch_acceleration = command_pitch_acceleration * 180.0 / M_PI;
  gimbal_cmd.yaw_jerk = command_yaw_jerk * 180.0 / M_PI;
  gimbal_cmd.pitch_jerk = command_pitch_jerk * 180.0 / M_PI;
  gimbal_cmd.mpc_valid = mpc_valid;
  gimbal_cmd.control_mode = mpc_valid ? 1U : 0U;
  gimbal_cmd.command_sequence = ++command_sequence_;
  gimbal_cmd.command_dt = mpc_valid ? mpc_dt_ : 0.0;
  gimbal_cmd.fire_advice = planning_safe && mpc_safe && isOnTarget(rpy[2],
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
  if (!isValidGimbalCommand(gimbal_cmd)) {
    gimbal_cmd.control_mode = 0U;
    gimbal_cmd.mpc_valid = false;
    gimbal_cmd.yaw_jerk = 0.0;
    gimbal_cmd.pitch_jerk = 0.0;
    gimbal_cmd.command_dt = 0.0;
    gimbal_cmd.fire_advice = false;
  }
  previous_gimbal_yaw_ = rpy[2];
  previous_gimbal_pitch_ = rpy[1];
  previous_gimbal_yaw_velocity_ = yaw_velocity;
  previous_gimbal_pitch_velocity_ = pitch_velocity;
  previous_gimbal_time_ = current_time;
  gimbal_state_initialized_ = true;
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
