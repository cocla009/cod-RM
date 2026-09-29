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

#include "armor_detector/armor_detector_node.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <functional>
#include <limits>
#include <stdexcept>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/create_timer_ros.h>
#include "rm_utils/math/utils.hpp"
#include "rm_utils/url_resolver.hpp"

namespace fyt::auto_aim {
namespace {
rcl_interfaces::msg::ParameterDescriptor readOnly() {
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  return descriptor;
}
void requirePositive(double value, const char *name) {
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string(name) + " must be finite and positive");
  }
}
}  // namespace

ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions &options)
: Node("armor_detector", options) {
  detector_ = initDetector();
  const auto fixed = readOnly();
  use_ba_ = declare_parameter("use_ba", true, fixed);
  pnp_solution_selection_ = declare_parameter("pnp_solution_selection", true, fixed);
  odom_frame_ = declare_parameter("target_frame", "odom", fixed);
  
  max_frame_age_ = declare_parameter("max_frame_age", 0.2, fixed);
  max_reprojection_error_ = declare_parameter("max_reprojection_error", 5.0, fixed);
  small_width_ = declare_parameter("geometry.small_width", SMALL_ARMOR_WIDTH, fixed);
  small_height_ = declare_parameter("geometry.small_height", SMALL_ARMOR_HEIGHT, fixed);
  large_width_ = declare_parameter("geometry.large_width", LARGE_ARMOR_WIDTH, fixed);
  large_height_ = declare_parameter("geometry.large_height", LARGE_ARMOR_HEIGHT, fixed);
  requirePositive(max_frame_age_, "max_frame_age");
  requirePositive(max_reprojection_error_, "max_reprojection_error");
  requirePositive(small_width_, "geometry.small_width");
  requirePositive(small_height_, "geometry.small_height");
  requirePositive(large_width_, "geometry.large_width");
  requirePositive(large_height_, "geometry.large_height");
  if (odom_frame_.empty()) throw std::invalid_argument("target_frame must not be empty");

  declare_parameter("debug", false);
  // Create transport plugins once; toggling debug only controls publication.
  declare_parameter("armor_detector.result_img.jpeg_quality", 50);
  result_img_pub_ = image_transport::create_publisher(this, "armor_detector/result_img");
  armors_pub_ = create_publisher<rm_interfaces::msg::Armors>(
    "armor_detector/armors", rclcpp::SensorDataQoS().keep_last(1));
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("armor_detector/marker", 1);

  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf2_buffer_->setCreateTimerInterface(std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(), get_node_timers_interface()));
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  cam_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    "camera_info", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&ArmorDetectorNode::cameraInfoCallback, this, std::placeholders::_1));
  subscribeImage();
  set_mode_srv_ = create_service<rm_interfaces::srv::SetMode>(
    "armor_detector/set_mode", std::bind(&ArmorDetectorNode::setModeCallback, this,
                                        std::placeholders::_1, std::placeholders::_2));
  parameter_callback_ = add_on_set_parameters_callback(
    std::bind(&ArmorDetectorNode::onSetParameters, this, std::placeholders::_1));
  heartbeat_ = HeartBeatPublisher::create(this);
  RCLCPP_INFO(get_logger(), "Armor detector ready: RobotPilots 0526, four network corners");
}

std::unique_ptr<Detector> ArmorDetectorNode::initDetector() {
  const auto fixed = readOnly();
  const auto uri = declare_parameter("nn.model_path", "package://armor_detector/model/rp_0526.onnx", fixed);
  const auto device = declare_parameter("nn.device", "CPU", fixed);
  const auto color = declare_parameter("detect_color", 0, fixed);
  declare_parameter("nn.confidence_threshold", 0.7);
  declare_parameter("nn.nms_threshold", 0.4);
  const std::filesystem::path path = uri.find("://") == std::string::npos
    ? std::filesystem::path(uri) : utils::URLResolver::getResolvedPath(uri);
  RCLCPP_INFO(get_logger(), "Loading 0526 model %s on %s", path.c_str(), device.c_str());
  return std::make_unique<Detector>(path.string(), device, detectorParams(),
                                    static_cast<EnemyColor>(color));
}

Detector::Params ArmorDetectorNode::detectorParams() const {
  const auto values = get_parameters({"nn.confidence_threshold", "nn.nms_threshold"});
  return {values[0].as_double(), values[1].as_double()};
}

void ArmorDetectorNode::subscribeImage() {
  if (!img_sub_) {
    img_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "image_raw", rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&ArmorDetectorNode::imageCallback, this, std::placeholders::_1));
  }
}

void ArmorDetectorNode::cameraInfoCallback(sensor_msgs::msg::CameraInfo::ConstSharedPtr info) {
  if (cam_info_ && info->k == cam_info_->k && info->d == cam_info_->d &&
      info->width == cam_info_->width && info->height == cam_info_->height &&
      info->header.frame_id == cam_info_->header.frame_id &&
      info->distortion_model == cam_info_->distortion_model &&
      info->binning_x == cam_info_->binning_x && info->binning_y == cam_info_->binning_y &&
      info->roi == cam_info_->roi) return;
  // Never retain an old calibration after receiving an invalid replacement.
  cam_info_.reset();
  pnp_solver_.reset();
  ba_solver_.reset();
  const auto finite = [](double x) { return std::isfinite(x); };
  const size_t n = info->d.size();
  if (!std::all_of(info->k.begin(), info->k.end(), finite) ||
      !std::all_of(info->d.begin(), info->d.end(), finite) ||
      info->k[0] <= 0 || info->k[4] <= 0 || info->k[8] != 1.0 ||
      info->width == 0 || info->height == 0 || info->header.frame_id.empty() ||
      info->binning_x > 1 || info->binning_y > 1 ||
      info->roi.x_offset != 0 || info->roi.y_offset != 0 ||
      (info->roi.width != 0 && info->roi.width != info->width) ||
      (info->roi.height != 0 && info->roi.height != info->height) ||
      (n != 0 && n != 4 && n != 5 && n != 8 && n != 12 && n != 14) ||
      (!info->distortion_model.empty() && info->distortion_model != "plumb_bob" &&
       info->distortion_model != "rational_polynomial")) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Invalid/unsupported calibration: full-frame pinhole images required");
    return;
  }
  try {
    camera_matrix_ = cv::Mat(3, 3, CV_64F, const_cast<double *>(info->k.data())).clone();
    distortion_ = cv::Mat(info->d, true);
    auto pnp = std::make_unique<PnPSolver>(info->k, info->d);
    pnp->setObjectPoints("small", Armor::buildObjectPoints<cv::Point3f>(small_width_, small_height_));
    pnp->setObjectPoints("large", Armor::buildObjectPoints<cv::Point3f>(large_width_, large_height_));
    auto ba = std::make_unique<BaSolver>(info->k, info->d);
    pnp_solver_ = std::move(pnp);
    ba_solver_ = std::move(ba);
    cam_info_ = std::move(info);
  } catch (const std::exception &error) {
    RCLCPP_ERROR(get_logger(), "Calibration setup failed: %s", error.what());
  }
}

bool ArmorDetectorNode::frameIsFresh(const std_msgs::msg::Header &header) {
  const rclcpp::Time stamp(header.stamp, get_clock()->get_clock_type());
  const double age = (now() - stamp).seconds();
  return stamp.nanoseconds() > 0 && age >= -0.01 && age <= max_frame_age_;
}

void ArmorDetectorNode::imageCallback(sensor_msgs::msg::Image::ConstSharedPtr image) {
  if (!enabled_) return;
  rm_interfaces::msg::Armors message;
  message.header = image->header;
  std::vector<Armor> armors;
  try {
    if (!frameIsFresh(image->header)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Dropping stale/invalid image timestamp");
      publishResult(message);
      return;
    }
    if (!cam_info_ || image->width != cam_info_->width || image->height != cam_info_->height ||
        image->header.frame_id != cam_info_->header.frame_id) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for calibration matching image frame/size");
      publishResult(message);
      return;
    }
    const auto transform = tf2_buffer_->lookupTransform(
      odom_frame_, image->header.frame_id, rclcpp::Time(image->header.stamp),
      rclcpp::Duration::from_seconds(0.01));
    tf2::Quaternion q;
    tf2::fromMsg(transform.transform.rotation, q);
    if (!std::isfinite(q.length2()) || q.length2() < 1e-12) {
      throw std::runtime_error("Invalid camera TF quaternion");
    }
    const tf2::Matrix3x3 rotation(q.normalized());
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) camera_to_odom_(row, col) = rotation[row][col];
    }
    detector_->params = detectorParams();
    armors = detector_->detect(cv_bridge::toCvShare(image, "rgb8")->image);
    for (auto &armor : armors) {
      try {
        armor.width = armor.type == ArmorType::SMALL ? small_width_ : large_width_;
        armor.height = armor.type == ArmorType::SMALL ? small_height_ : large_height_;
        rm_interfaces::msg::Armor result;
        if (solvePose(armor, result)) message.armors.push_back(std::move(result));
      } catch (const std::exception &error) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Pose rejected: %s", error.what());
      }
    }
    // Inference/pose latency counts toward the frame age budget as well.
    if (!frameIsFresh(image->header)) {
      message.armors.clear();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Inference exceeded max_frame_age");
    }
  } catch (const std::exception &error) {
    message.armors.clear();
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Empty detection frame: %s", error.what());
  }
  publishResult(message);
  // Debug publication happens after the control-facing result and cannot invalidate it.
  if (get_parameter("debug").as_bool()) {
    try {
      publishDebug(image, armors);
    } catch (const std::exception &error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Debug image failed: %s", error.what());
    }
  }
}

double ArmorDetectorNode::reprojectionError(
  const Armor &armor, const cv::Mat &rvec, const cv::Mat &tvec) const {
  const double invalid = std::numeric_limits<double>::infinity();
  if (rvec.empty() || tvec.empty() || !cv::checkRange(rvec) || !cv::checkRange(tvec) ||
      tvec.at<double>(2) <= 0) return invalid;
  const auto object = Armor::buildObjectPoints<cv::Point3f>(armor.width, armor.height);
  cv::Mat rotation;
  cv::Rodrigues(rvec, rotation);
  for (const auto &p : object) {
    if (rotation.at<double>(2, 0) * p.x + rotation.at<double>(2, 1) * p.y +
        rotation.at<double>(2, 2) * p.z + tvec.at<double>(2) <= 0) return invalid;
  }
  std::vector<cv::Point2f> projected;
  cv::projectPoints(object, rvec, tvec, camera_matrix_, distortion_, projected);
  double squared = 0;
  for (size_t i = 0; i < projected.size(); ++i) {
    const auto delta = projected[i] - armor.corners[i];
    squared += delta.dot(delta);
  }
  return std::isfinite(squared) ? std::sqrt(squared / Armor::N_LANDMARKS) : invalid;
}

bool ArmorDetectorNode::solvePose(Armor &armor, rm_interfaces::msg::Armor &message) {
  cv::Mat rvec, tvec;
  if (!pnp_solver_->solvePnP(armor.landmarks(), rvec, tvec, armorTypeToString(armor.type))) return false;
  const auto solutions = pnp_solver_->getAllSolutions();
  double best = std::numeric_limits<double>::infinity();
  bool found = false;
  const double prior = armor.number == "outpost" ? -FIFTTEN_DEGREE_RAD : FIFTTEN_DEGREE_RAD;
  for (size_t i = 0; i < solutions[0].size(); ++i) {
    const auto &r = solutions[0][i];
    const auto &t = solutions[1][i];
    const double error = reprojectionError(armor, r, t);
    if (error > max_reprojection_error_) continue;
    const double rank = pnp_solution_selection_ ? std::abs(rvecToRPY(r, 1) - prior) : error;
    if (rank < best) {
      best = rank;
      rvec = r;
      tvec = t;
      found = true;
    }
  }
  if (!found) return false;
  armor.camera_to_odom = camera_to_odom_;
  armor.roll = rvecToRPY(rvec, 0) * 180 / CV_PI;
  armor.tvec = tvec;
  cv::Rodrigues(rvec, armor.rmat);
  if (use_ba_ && std::abs(armor.roll) < 10) {
    cv::Mat refined;
    if (ba_solver_->solveBa(armor, refined)) {
      cv::Mat refined_rvec;
      cv::Rodrigues(refined, refined_rvec);
      if (reprojectionError(armor, refined_rvec, tvec) <= max_reprojection_error_) {
        armor.rmat = refined;
      }
    }
  }
  const Eigen::Matrix3d rotation = utils::cvToEigen(armor.rmat);
  const Eigen::Quaterniond q(rotation);
  if (!q.coeffs().allFinite()) return false;
  message.number = armor.number;
  message.type = armorTypeToString(armor.type);
  message.pose.position.x = tvec.at<double>(0);
  message.pose.position.y = tvec.at<double>(1);
  message.pose.position.z = tvec.at<double>(2);
  message.pose.orientation.x = q.x();
  message.pose.orientation.y = q.y();
  message.pose.orientation.z = q.z();
  message.pose.orientation.w = q.w();
  message.distance_to_image_center = pnp_solver_->calculateDistanceToCenter(armor.center);
  return true;
}

double ArmorDetectorNode::rvecToRPY(const cv::Mat &rvec, int axis) const {
  cv::Mat rotation;
  cv::Rodrigues(rvec, rotation);
  const Eigen::Matrix3d rotation_in_odom = camera_to_odom_ * utils::cvToEigen(rotation);
  const Eigen::Quaterniond q(rotation_in_odom);
  std::array<double, 3> rpy;
  tf2::Matrix3x3(tf2::Quaternion(q.x(), q.y(), q.z(), q.w())).getRPY(rpy[0], rpy[1], rpy[2]);
  return rpy[axis];
}

void ArmorDetectorNode::publishResult(const rm_interfaces::msg::Armors &message) {
  armors_pub_->publish(message);
  using Marker = visualization_msgs::msg::Marker;
  visualization_msgs::msg::MarkerArray array;
  Marker clear;
  clear.header = message.header;
  clear.action = Marker::DELETEALL;
  array.markers.push_back(clear);
  int id = 0;
  for (const auto &armor : message.armors) {
    Marker box;
    box.header = message.header;
    box.ns = "armors";
    box.id = id++;
    box.type = Marker::CUBE;
    box.action = Marker::ADD;
    box.pose = armor.pose;
    box.scale.x = 0.03;
    box.scale.y = armor.type == "small" ? small_width_ : large_width_;
    box.scale.z = armor.type == "small" ? small_height_ : large_height_;
    box.color.r = 1.0;
    box.color.a = 1.0;
    box.lifetime = rclcpp::Duration::from_seconds(max_frame_age_);
    array.markers.push_back(box);
    box.ns = "classification";
    box.type = Marker::TEXT_VIEW_FACING;
    box.pose.position.y -= 0.1;
    box.scale.z = 0.1;
    box.text = armor.number;
    box.color.g = box.color.b = 1.0;
    array.markers.push_back(box);
  }
  marker_pub_->publish(array);
}

void ArmorDetectorNode::publishDebug(const sensor_msgs::msg::Image::ConstSharedPtr &image,
                                      const std::vector<Armor> &armors) {
  if (result_img_pub_.getNumSubscribers() == 0) return;
  cv::Mat result = cv_bridge::toCvShare(image, "rgb8")->image.clone();
  for (const auto &armor : armors) {
    for (int i = 0; i < Armor::N_LANDMARKS; ++i) {
      cv::line(result, armor.corners[i], armor.corners[(i + 1) % 4], {0, 255, 0}, 2);
      cv::circle(result, armor.corners[i], 3, {255, 255, 0}, -1);
      cv::putText(result, std::to_string(i), armor.corners[i], cv::FONT_HERSHEY_SIMPLEX,
                  0.4, {255, 255, 255}, 1);
    }
    cv::putText(result, armor.number + " " + armorTypeToString(armor.type) + " " +
                cv::format("%.2f", armor.confidence), armor.center,
                cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 255, 0}, 1);
  }
  const double age_ms = (now() - rclcpp::Time(image->header.stamp, get_clock()->get_clock_type())).seconds() * 1000;
  cv::putText(result, cv::format("0526 network detections | frame age %.1f ms", age_ms),
              {10, 25}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 255, 0}, 1);
  result_img_pub_.publish(cv_bridge::CvImage(image->header, "rgb8", result).toImageMsg());
}

rcl_interfaces::msg::SetParametersResult ArmorDetectorNode::onSetParameters(
  const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  try {
    auto settings = detectorParams();
    for (const auto &p : parameters) {
      if (p.get_name() == "nn.confidence_threshold") settings.confidence_threshold = p.as_double();
      if (p.get_name() == "nn.nms_threshold") settings.nms_threshold = p.as_double();
    }
    Detector::validateParams(settings);
    // Read committed parameters at the next image callback; validation has no side effects.
  } catch (const std::exception &error) {
    result.successful = false;
    result.reason = error.what();
  }
  return result;
}

void ArmorDetectorNode::setModeCallback(
  std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
  std::shared_ptr<rm_interfaces::srv::SetMode::Response> response) {
  const auto mode = static_cast<VisionMode>(request->mode);
  const auto name = visionModeToString(mode);
  if (name == "UNKNOWN") {
    response->success = false;
    response->message = "Invalid vision mode";
    return;
  }
  enabled_ = mode == VisionMode::AUTO_AIM_RED || mode == VisionMode::AUTO_AIM_BLUE;
  if (enabled_) {
    detector_->detect_color = mode == VisionMode::AUTO_AIM_RED ? EnemyColor::RED : EnemyColor::BLUE;
    subscribeImage();
  } else {
    img_sub_.reset();
  }
  if (cam_info_) {
    rm_interfaces::msg::Armors empty;
    empty.header = cam_info_->header;
    empty.header.stamp = now();
    publishResult(empty);
  }
  response->success = true;
  response->message = "0";
  RCLCPP_INFO(get_logger(), "Set mode to %s", name.c_str());
}
}  // namespace fyt::auto_aim

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fyt::auto_aim::ArmorDetectorNode)
