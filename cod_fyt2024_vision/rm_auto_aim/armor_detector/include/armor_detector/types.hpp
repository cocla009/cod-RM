// Created by Chengfu Zou on 2023.10.26
// Maintained by Chengfu Zou
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

#ifndef ARMOR_DETECTOR_TYPES_HPP_
#define ARMOR_DETECTOR_TYPES_HPP_

#include <array>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <opencv2/core.hpp>

namespace fyt::auto_aim {
// Distances between the four light endpoints, in metres. Measure on your armor.
constexpr double SMALL_ARMOR_WIDTH = 0.135;
constexpr double SMALL_ARMOR_HEIGHT = 0.055;
constexpr double LARGE_ARMOR_WIDTH = 0.230;
constexpr double LARGE_ARMOR_HEIGHT = 0.055;
constexpr double FIFTTEN_DEGREE_RAD = 15 * CV_PI / 180;

enum class ArmorType { SMALL, LARGE, INVALID };
inline std::string armorTypeToString(ArmorType type) {
  switch (type) {
    case ArmorType::SMALL: return "small";
    case ArmorType::LARGE: return "large";
    default: return "invalid";
  }
}

struct CameraInternalK {
  Eigen::Matrix3d toMatrix() const noexcept {
    Eigen::Matrix3d k;
    k << fx, 0, cx, 0, fy, cy, 0, 0, 1;
    return k;
  }
  double fx, fy, cx, cy;
};

struct Armor {
  static constexpr int N_LANDMARKS = 4;
  // Image order: bottom-left, top-left, top-right, bottom-right.
  // Object axes: x normal to the plate, y left, z up.
  template <typename PointType>
  static std::vector<PointType> buildObjectPoints(double w, double h) {
    return {PointType(0, w / 2, -h / 2), PointType(0, w / 2, h / 2),
            PointType(0, -w / 2, h / 2), PointType(0, -w / 2, -h / 2)};
  }
  std::vector<cv::Point2f> landmarks() const { return {corners.begin(), corners.end()}; }
  std::array<cv::Point2f, N_LANDMARKS> corners{};
  cv::Point2f center{};
  ArmorType type = ArmorType::INVALID;
  std::string number;
  float confidence = 0.0F;
  // Assigned from geometry.* before pose estimation.
  double width = 0.0;
  double height = 0.0;
  cv::Mat rmat, tvec;
  double roll = 0.0;
  Eigen::Matrix3d camera_to_odom = Eigen::Matrix3d::Identity();
};
}  // namespace fyt::auto_aim
#endif
