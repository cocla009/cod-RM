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

#ifndef ARMOR_DETECTOR_DETECTOR_NODE_HPP_
#define ARMOR_DETECTOR_DETECTOR_NODE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <image_transport/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include "armor_detector/armor_detector.hpp"
#include "armor_detector/ba_solver.hpp"
#include "rm_interfaces/msg/armors.hpp"
#include "rm_interfaces/srv/set_mode.hpp"
#include "rm_utils/heartbeat.hpp"
#include "rm_utils/math/pnp_solver.hpp"

namespace fyt::auto_aim {
class ArmorDetectorNode : public rclcpp::Node {
public:
  explicit ArmorDetectorNode(const rclcpp::NodeOptions &options);

private:
  std::unique_ptr<Detector> initDetector();
  void subscribeImage();
  void cameraInfoCallback(sensor_msgs::msg::CameraInfo::ConstSharedPtr info);
  void imageCallback(sensor_msgs::msg::Image::ConstSharedPtr image);
  bool frameIsFresh(const std_msgs::msg::Header &header);
  bool solvePose(Armor &armor, rm_interfaces::msg::Armor &message);
  double reprojectionError(const Armor &armor, const cv::Mat &rvec, const cv::Mat &tvec) const;
  double rvecToRPY(const cv::Mat &rvec, int axis) const;
  void publishResult(const rm_interfaces::msg::Armors &message);
  void publishDebug(const sensor_msgs::msg::Image::ConstSharedPtr &image,
                    const std::vector<Armor> &armors);
  void setModeCallback(std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
                       std::shared_ptr<rm_interfaces::srv::SetMode::Response> response);
  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> &parameters);
  Detector::Params detectorParams() const;

  // Image, camera-info, parameter and mode callbacks share the default mutually
  // exclusive callback group, including in component_container_mt.
  std::unique_ptr<Detector> detector_;
  std::unique_ptr<PnPSolver> pnp_solver_;
  std::unique_ptr<BaSolver> ba_solver_;
  cv::Mat camera_matrix_, distortion_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr cam_info_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Publisher<rm_interfaces::msg::Armors>::SharedPtr armors_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  image_transport::Publisher result_img_pub_;
  rclcpp::Service<rm_interfaces::srv::SetMode>::SharedPtr set_mode_srv_;
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
  HeartBeatPublisher::SharedPtr heartbeat_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  Eigen::Matrix3d camera_to_odom_ = Eigen::Matrix3d::Identity();
  std::string odom_frame_;
  bool enabled_ = true;
  bool use_ba_ = true;
  bool pnp_solution_selection_ = true;
  double max_frame_age_ = 0.2;
  double max_reprojection_error_ = 5.0;
  double small_width_, small_height_, large_width_, large_height_;
};
}  // namespace fyt::auto_aim
#endif
