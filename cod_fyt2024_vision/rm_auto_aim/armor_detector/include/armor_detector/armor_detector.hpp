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

#ifndef ARMOR_DETECTOR_DETECTOR_HPP_
#define ARMOR_DETECTOR_DETECTOR_HPP_

#include <memory>
#include <string>
#include <vector>
#include "armor_detector/types.hpp"
#include "rm_utils/common.hpp"

namespace fyt::auto_aim {
// RobotPilots 0526: RGB float32 NCHW [1,3,640,640] -> FP16 model -> FP32 output.
// Single inference request; callers must serialize detect() and configuration changes.
class Detector {
public:
  struct Params {
    double confidence_threshold = 0.7;
    double nms_threshold = 0.4;
  };
  Detector(const std::string &model_path, const std::string &device, Params params,
           EnemyColor color);
  ~Detector();
  Detector(const Detector &) = delete;
  Detector &operator=(const Detector &) = delete;
  std::vector<Armor> detect(const cv::Mat &rgb_image);
  static std::vector<Armor> decode(const cv::Mat &output, cv::Size image_size,
                                   cv::Size resized_size, const Params &params,
                                   EnemyColor color);
  static void validateParams(const Params &params);
  Params params;
  EnemyColor detect_color;
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace fyt::auto_aim
#endif
