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

#include "armor_detector/armor_detector.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <openvino/c/openvino.h>

namespace fyt::auto_aim {
namespace {
constexpr int kInputSize = 640;
constexpr int kRows = 25200;
constexpr int kColumns = 22;
void check(ov_status_e status, const char *operation) {
  if (status != OK) {
    const char *detail = ov_get_last_err_msg();
    throw std::runtime_error(std::string("OpenVINO ") + operation + ": " +
                             (detail ? detail : ov_get_error_info(status)));
  }
}
void checkPort(ov_output_const_port_t *raw_port, const std::vector<int64_t> &expected,
               ov_element_type_e expected_type) {
  std::unique_ptr<ov_output_const_port_t, decltype(&ov_output_const_port_free)>
    port(raw_port, ov_output_const_port_free);
  ov_shape_t shape{};
  check(ov_const_port_get_shape(port.get(), &shape), "read static model shape");
  const bool valid = shape.rank == static_cast<int64_t>(expected.size()) &&
    std::equal(expected.begin(), expected.end(), shape.dims);
  ov_shape_free(&shape);
  ov_element_type_e type;
  check(ov_port_get_element_type(port.get(), &type), "read model type");
  if (!valid || type != expected_type) {
    throw std::runtime_error("Expected RobotPilots 0526 float16 [1,3,640,640] -> "
                             "float32 [1,25200,22]; generic YOLO weights are incompatible");
  }
}
}  // namespace

// The C API keeps the ROS/OpenCV C++ ABI independent of the OpenVINO distribution.
struct Detector::Impl {
  ov_core_t *core = nullptr;
  ov_model_t *model = nullptr;
  ov_compiled_model_t *compiled = nullptr;
  ov_infer_request_t *request = nullptr;
  ov_tensor_t *input = nullptr;
  ov_tensor_t *output = nullptr;
  ~Impl() {
    if (output) ov_tensor_free(output);
    if (input) ov_tensor_free(input);
    if (request) ov_infer_request_free(request);
    if (compiled) ov_compiled_model_free(compiled);
    if (model) ov_model_free(model);
    if (core) ov_core_free(core);
  }
  void init(const std::string &path, const std::string &device) {
    check(ov_core_create(&core), "create core");
    check(ov_core_read_model(core, path.c_str(), nullptr, &model), "read model");
    size_t inputs = 0, outputs = 0;
    check(ov_model_inputs_size(model, &inputs), "input count");
    check(ov_model_outputs_size(model, &outputs), "output count");
    if (inputs != 1 || outputs != 1) throw std::runtime_error("0526 requires one input/output");
    ov_output_const_port_t *port = nullptr;
    check(ov_model_const_input(model, &port), "input port");
    checkPort(port, {1, 3, kInputSize, kInputSize}, F16);
    check(ov_model_const_output(model, &port), "output port");
    checkPort(port, {1, kRows, kColumns}, F32);
    // The original ONNX consumes FP16. Expose an FP32 input to the C++ image
    // loop and let OpenVINO insert the conversion in the compiled graph.
    using Preprocessor = std::unique_ptr<ov_preprocess_prepostprocessor_t,
                                         decltype(&ov_preprocess_prepostprocessor_free)>;
    using InputInfo = std::unique_ptr<ov_preprocess_input_info_t,
                                      decltype(&ov_preprocess_input_info_free)>;
    using TensorInfo = std::unique_ptr<ov_preprocess_input_tensor_info_t,
                                       decltype(&ov_preprocess_input_tensor_info_free)>;
    ov_preprocess_prepostprocessor_t *preprocessor_raw = nullptr;
    check(ov_preprocess_prepostprocessor_create(model, &preprocessor_raw), "create preprocessing");
    Preprocessor preprocessor(preprocessor_raw, ov_preprocess_prepostprocessor_free);
    ov_preprocess_input_info_t *input_info_raw = nullptr;
    check(ov_preprocess_prepostprocessor_get_input_info(preprocessor.get(), &input_info_raw),
          "get input preprocessing");
    InputInfo input_info(input_info_raw, ov_preprocess_input_info_free);
    ov_preprocess_input_tensor_info_t *tensor_info_raw = nullptr;
    check(ov_preprocess_input_info_get_tensor_info(input_info.get(), &tensor_info_raw),
          "get input tensor information");
    TensorInfo tensor_info(tensor_info_raw, ov_preprocess_input_tensor_info_free);
    check(ov_preprocess_input_tensor_info_set_element_type(tensor_info.get(), F32),
          "set float32 input tensor");
    ov_model_t *preprocessed_model = nullptr;
    check(ov_preprocess_prepostprocessor_build(preprocessor.get(), &preprocessed_model),
          "build float32 to float16 preprocessing");
    ov_model_free(model);
    model = preprocessed_model;
    check(ov_core_compile_model(core, model, device.c_str(), 2, &compiled,
                               ov_property_key_hint_performance_mode, "LATENCY"), "compile model");
    check(ov_compiled_model_create_infer_request(compiled, &request), "create request");
    check(ov_infer_request_get_input_tensor(request, &input), "input tensor");
    check(ov_infer_request_get_output_tensor(request, &output), "output tensor");
    ov_element_type_e input_type;
    check(ov_tensor_get_element_type(input, &input_type), "check compiled input type");
    if (input_type != F32) throw std::runtime_error("Preprocessed input is not float32");
  }
};

Detector::Detector(const std::string &path, const std::string &device, Params settings,
                   EnemyColor color)
: params(settings), detect_color(color), impl_(std::make_unique<Impl>()) {
  validateParams(params);
  if (device.empty()) throw std::invalid_argument("nn.device must not be empty");
  if (color != EnemyColor::RED && color != EnemyColor::BLUE) {
    throw std::invalid_argument("detect_color must be 0 (red) or 1 (blue)");
  }
  if (!std::filesystem::is_regular_file(path)) {
    throw std::runtime_error("Missing 0526 model: " + path +
                             "; run armor_detector/model/fetch_model.py before building");
  }
  impl_->init(path, device);
}
Detector::~Detector() = default;

void Detector::validateParams(const Params &p) {
  if (!std::isfinite(p.confidence_threshold) || p.confidence_threshold <= 0.0 ||
      p.confidence_threshold >= 1.0 || !std::isfinite(p.nms_threshold) ||
      p.nms_threshold <= 0.0 || p.nms_threshold >= 1.0) {
    throw std::invalid_argument("NN confidence/NMS thresholds must be in (0,1)");
  }
}

std::vector<Armor> Detector::detect(const cv::Mat &image) {
  if (image.empty()) return {};
  if (image.type() != CV_8UC3) throw std::invalid_argument("Detector requires RGB uint8 image");
  const double scale = std::min(double(kInputSize) / image.cols, double(kInputSize) / image.rows);
  const cv::Size resized(std::max(1, int(image.cols * scale)),
                         std::max(1, int(image.rows * scale)));
  cv::Mat padded(kInputSize, kInputSize, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::resize(image, padded(cv::Rect({0, 0}, resized)), resized);
  // RGB is already supplied by cv_bridge. Black padding is on bottom/right only.
  void *input_data = nullptr;
  check(ov_tensor_data(impl_->input, &input_data), "map input");
  auto *data = static_cast<float *>(input_data);
  constexpr int plane = kInputSize * kInputSize;
  for (int y = 0; y < kInputSize; ++y) {
    const auto *row = padded.ptr<cv::Vec3b>(y);
    for (int x = 0; x < kInputSize; ++x) {
      for (int c = 0; c < 3; ++c) data[c * plane + y * kInputSize + x] = row[x][c] / 255.0F;
    }
  }
  check(ov_infer_request_infer(impl_->request), "infer");
  void *output_data = nullptr;
  check(ov_tensor_data(impl_->output, &output_data), "map output");
  return decode(cv::Mat(kRows, kColumns, CV_32F, output_data), image.size(), resized,
                params, detect_color);
}

std::vector<Armor> Detector::decode(const cv::Mat &output, cv::Size image_size,
                                    cv::Size resized, const Params &settings, EnemyColor color) {
  validateParams(settings);
  if (output.type() != CV_32F || output.cols != kColumns ||
      image_size.width <= 0 || image_size.height <= 0 ||
      resized.width <= 0 || resized.height <= 0 || resized.width > kInputSize ||
      resized.height > kInputSize) throw std::invalid_argument("Invalid 0526 output/resize metadata");
  if (color != EnemyColor::RED && color != EnemyColor::BLUE) {
    throw std::invalid_argument("Detection color must be red or blue");
  }
  const float sx = float(image_size.width) / resized.width;
  const float sy = float(image_size.height) / resized.height;
  const std::array<std::string, 9> names{"sentry", "1", "2", "3", "4", "5", "outpost", "base", "base"};
  std::vector<Armor> candidates;
  std::vector<cv::Rect2d> boxes;
  std::vector<float> scores;
  for (int row = 0; row < output.rows; ++row) {
    const float *v = output.ptr<float>(row);
    if (!std::all_of(v, v + kColumns, [](float x) { return std::isfinite(x); })) continue;
    const float score = v[8] >= 0 ? 1.0F / (1.0F + std::exp(-v[8]))
                                  : std::exp(v[8]) / (1.0F + std::exp(v[8]));
    if (score < settings.confidence_threshold) continue;
    const int color_id = std::max_element(v + 9, v + 13) - (v + 9);
    if (color_id != static_cast<int>(color)) continue;
    const int number_id = std::max_element(v + 13, v + 22) - (v + 13);
    Armor armor;
    // Network TL,BL,BR,TR -> PnP BL,TL,TR,BR. Never sort by image x/y.
    constexpr std::array<int, 4> order{1, 0, 3, 2};
    bool inside = true;
    for (int i = 0; i < 4; ++i) {
      armor.corners[i] = {v[2 * order[i]] * sx, v[2 * order[i] + 1] * sy};
      const auto &p = armor.corners[i];
      inside = inside && p.x >= 0 && p.y >= 0 && p.x < image_size.width && p.y < image_size.height;
      armor.center += p * 0.25F;
    }
    if (!inside) continue;  // Clipping corner coordinates would corrupt the PnP measurement.
    const auto points = armor.landmarks();
    if (!cv::isContourConvex(points) || cv::contourArea(points, true) <= 1.0) continue;
    const double height = std::max(cv::norm(points[0] - points[1]), cv::norm(points[2] - points[3]));
    const double width = std::max(cv::norm(points[1] - points[2]), cv::norm(points[0] - points[3]));
    if (height < 1.0 || width < 1.0) continue;
    // Project rule: only armor 1 and base are large. Both network base labels
    // (Bs and Bb) publish as "base" and use the project's large-board geometry.
    const bool large = number_id == 1 || number_id == 7 || number_id == 8;
    armor.type = large ? ArmorType::LARGE : ArmorType::SMALL;
    armor.number = names[number_id];
    armor.confidence = score;
    float min_x = points[0].x, max_x = min_x, min_y = points[0].y, max_y = min_y;
    for (const auto &p : points) {
      min_x = std::min(min_x, p.x); max_x = std::max(max_x, p.x);
      min_y = std::min(min_y, p.y); max_y = std::max(max_y, p.y);
    }
    boxes.emplace_back(min_x, min_y, max_x - min_x, max_y - min_y);
    scores.push_back(score);
    candidates.push_back(std::move(armor));
  }
  std::vector<int> indices;
  // Suppress duplicate predictions even when their number differs.
  cv::dnn::NMSBoxes(boxes, scores, settings.confidence_threshold, settings.nms_threshold,
                    indices, 1.0F, 128);
  std::vector<Armor> result;
  result.reserve(indices.size());
  for (int index : indices) result.push_back(std::move(candidates[index]));
  return result;
}
}  // namespace fyt::auto_aim
