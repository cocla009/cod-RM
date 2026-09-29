// Created by Labor 2023.8.25
// Maintained by Labor, Chengfu Zou
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

#ifndef ARMOR_DETECTOR_BA_SOLVER_HPP_
#define ARMOR_DETECTOR_BA_SOLVER_HPP_

// std
#include <array>
#include <cstddef>
#include <vector>
// 3rd party
#include <Eigen/Core>
#include <opencv2/core.hpp>
// project
#include "armor_detector/types.hpp"

namespace fyt::auto_aim {

// Two-stage reprojection search for the armor yaw. It keeps the PnP translation
// and fixed pitch prior while avoiding a heavyweight graph-optimizer runtime.
class BaSolver {
public:
  BaSolver(const std::array<double, 9> &camera_matrix, const std::vector<double> &dist_coeffs);

  // Solve the armor yaw and update the camera-to-armor rotation matrix.
  bool solveBa(const Armor &armor, cv::Mat &rmat);

private:
  double computeReprojError(const Eigen::Matrix3d &camera2imu,
                            const Eigen::Vector3d &tvec,
                            const std::vector<cv::Point2f> &landmarks,
                            const std::vector<Eigen::Vector3d> &object_points,
                            double pitch,
                            double yaw) const noexcept;

  CameraInternalK cam_internal_k_;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
};

}  // namespace fyt::auto_aim
#endif // ARMOR_DETECTOR_BAS_SOLVER_HPP_
