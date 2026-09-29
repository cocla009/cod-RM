// Created by Labor 2023.8.25
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

#include "armor_detector/ba_solver.hpp"

#include <cmath>
#include <limits>
#include <opencv2/calib3d.hpp>

#include "armor_detector/types.hpp"
#include "rm_utils/math/utils.hpp"

namespace fyt::auto_aim {

BaSolver::BaSolver(const std::array<double, 9> &camera_matrix, const std::vector<double> &dist_coeffs)
: cam_internal_k_{.fx = camera_matrix[0],
                  .fy = camera_matrix[4],
                  .cx = camera_matrix[2],
                  .cy = camera_matrix[5]},
  camera_matrix_(cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_matrix.data())).clone()),
  dist_coeffs_(cv::Mat(dist_coeffs, true)) {}

double BaSolver::computeReprojError(const Eigen::Matrix3d &camera2imu,
                                    const Eigen::Vector3d &tvec,
                                    const std::vector<cv::Point2f> &landmarks,
                                    const std::vector<Eigen::Vector3d> &object_points,
                                    const double pitch,
                                    const double yaw) const noexcept {
  if (landmarks.size() != object_points.size()) {
    return std::numeric_limits<double>::infinity();
  }

  const std::array<double, 3> euler{0.0, pitch, yaw};
  const Eigen::Matrix3d imu2armor = utils::eulerToMatrix(euler, utils::EulerOrder::XYZ);
  const Eigen::Matrix3d rotation = camera2imu * imu2armor;
  const Eigen::Matrix3d camera_matrix = cam_internal_k_.toMatrix();

  double error = 0.0;
  for (size_t i = 0; i < object_points.size(); ++i) {
    const Eigen::Vector3d point = rotation * object_points[i] + tvec;
    if (!point.allFinite() || point.z() <= 0.0) {
      return std::numeric_limits<double>::infinity();
    }

    const Eigen::Vector3d projected = camera_matrix * (point / point.z());
    const double dx = projected.x() - landmarks[i].x;
    const double dy = projected.y() - landmarks[i].y;
    error += dx * dx + dy * dy;
  }
  return error;
}

bool BaSolver::solveBa(const Armor &armor, cv::Mat &rmat) {
  if (armor.tvec.empty() || armor.tvec.total() < 3) {
    return false;
  }

  // Search in undistorted pixel coordinates, matching the pinhole projection below.
  std::vector<cv::Point2f> landmarks;
  cv::undistortPoints(armor.landmarks(), landmarks, camera_matrix_, dist_coeffs_,
                      cv::noArray(), camera_matrix_);
  const Eigen::Matrix3d camera2imu = armor.camera_to_odom.transpose();
  const Eigen::Vector3d tvec(armor.tvec.at<double>(0),
                             armor.tvec.at<double>(1),
                             armor.tvec.at<double>(2));
  const auto object_points =
    Armor::buildObjectPoints<Eigen::Vector3d>(armor.width, armor.height);
  const double pitch = armor.number == "outpost" ? -FIFTTEN_DEGREE_RAD : FIFTTEN_DEGREE_RAD;

  constexpr int kCoarseSteps = 360;
  constexpr double kCoarseStep = 2.0 * CV_PI / kCoarseSteps;
  constexpr double kFineRange = 2.0 * CV_PI / 180.0;
  constexpr double kFineStep = 0.1 * CV_PI / 180.0;

  double best_yaw = 0.0;
  double best_error = std::numeric_limits<double>::infinity();
  for (int i = 0; i < kCoarseSteps; ++i) {
    const double yaw = -CV_PI + i * kCoarseStep;
    const double error =
      computeReprojError(camera2imu, tvec, landmarks, object_points, pitch, yaw);
    if (error < best_error) {
      best_error = error;
      best_yaw = yaw;
    }
  }

  const double coarse_yaw = best_yaw;
  for (double yaw = coarse_yaw - kFineRange; yaw <= coarse_yaw + kFineRange; yaw += kFineStep) {
    const double error =
      computeReprojError(camera2imu, tvec, landmarks, object_points, pitch, yaw);
    if (error < best_error) {
      best_error = error;
      best_yaw = yaw;
    }
  }

  if (!std::isfinite(best_error)) {
    return false;
  }

  const std::array<double, 3> optimized_euler{0.0, pitch, best_yaw};
  const Eigen::Matrix3d imu2armor =
    utils::eulerToMatrix(optimized_euler, utils::EulerOrder::XYZ);
  const Eigen::Matrix3d rmat_optimized = armor.camera_to_odom.transpose() * imu2armor;
  rmat = utils::eigenToCv(rmat_optimized);
  return true;
}

}  // namespace fyt::auto_aim
