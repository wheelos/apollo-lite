// Copyright 2026 WheelOS. All Rights Reserved.
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

#include <limits.h>
#include <stdlib.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <string>
#include <vector>

#include "gflags/gflags.h"
#include "google/protobuf/text_format.h"
#include "pcl/io/pcd_io.h"

#include "modules/lidar_object_detection/proto/lidar_object_detection.pb.h"
#include "wheelos_msgs/sensor_msgs/pointcloud.pb.h"

#include "modules/lidar_object_detection/types/model_input.h"
#include "modules/perception/lidar/common/pcl_util.h"

DEFINE_string(config, "", "Text-format CenterPointModelConfig path.");
DEFINE_string(input_pcd, "", "Input PCD path.");
DEFINE_string(input_nuscenes_bin, "", "Input nuScenes five-float binary path.");
DEFINE_string(
    input_nuscenes_model_bin, "",
    "Input nuScenes model-feature binary with time-lag in feature 5.");
DEFINE_string(output_bin, "", "Output packed five-feature binary path.");
DEFINE_string(scn_onnx, "", "Expected SCN ONNX model path.");
DEFINE_string(rpn_centerhead_engine, "",
              "Expected RPN/CenterHead TensorRT engine path.");

namespace apollo {
namespace lidar_object_detection {
namespace {

bool ReadConfig(const std::string& path, CenterPointModelConfig* config) {
  std::ifstream input(path);
  if (!input.is_open()) {
    std::cerr << "Unable to open config: " << path << std::endl;
    return false;
  }
  const std::string text((std::istreambuf_iterator<char>(input)),
                         std::istreambuf_iterator<char>());
  if (!google::protobuf::TextFormat::ParseFromString(text, config)) {
    std::cerr << "Unable to parse config: " << path << std::endl;
    return false;
  }
  return true;
}

std::string ParentPath(const std::string& path) {
  const std::size_t separator = path.find_last_of('/');
  if (separator == std::string::npos) {
    return ".";
  }
  if (separator == 0U) {
    return "/";
  }
  return path.substr(0U, separator);
}

bool ResolvePath(const std::string& path, const std::string& relative_to,
                 std::string* resolved, std::string* error) {
  if (path.empty()) {
    *error = "CenterPoint model path is not configured";
    return false;
  }
  std::string candidate = path;
  if (path.front() != '/' && !relative_to.empty()) {
    candidate = relative_to + "/" + path;
  }
  char resolved_path[PATH_MAX];
  if (realpath(candidate.c_str(), resolved_path) == nullptr) {
    *error = "Unable to resolve CenterPoint model path: " + path + " (" +
             std::strerror(errno) + ")";
    return false;
  }
  *resolved = resolved_path;
  return true;
}

bool ValidateModelPaths(const CenterPointModelConfig& config,
                        const std::string& config_path,
                        const std::string& expected_scn_onnx,
                        const std::string& expected_engine,
                        std::string* error) {
  const std::string config_dir = ParentPath(config_path);
  std::string configured_scn;
  std::string configured_engine;
  std::string actual_scn;
  std::string actual_engine;
  if (!ResolvePath(config.scn_onnx_path(), config_dir, &configured_scn,
                   error) ||
      !ResolvePath(config.rpn_centerhead_engine_path(), config_dir,
                   &configured_engine, error) ||
      !ResolvePath(expected_scn_onnx, {}, &actual_scn, error) ||
      !ResolvePath(expected_engine, {}, &actual_engine, error)) {
    return false;
  }
  if (configured_scn != actual_scn || configured_engine != actual_engine) {
    *error =
        "CenterPoint model paths in config must resolve to the same "
        "files passed via --scn_onnx and --rpn_centerhead_engine";
    return false;
  }
  return true;
}

bool LoadPcd(const std::string& path, apollo::drivers::PointCloud* cloud) {
  pcl::PointCloud<apollo::perception::lidar::PCLPointXYZIT> pcd;
  if (pcl::io::loadPCDFile(path, pcd) < 0) {
    std::cerr << "Unable to read PCD: " << path << std::endl;
    return false;
  }
  constexpr std::size_t kCenterPointMaxPoints = 300000U;
  if (pcd.size() > kCenterPointMaxPoints) {
    std::cerr << "CenterPoint accepts at most " << kCenterPointMaxPoints
              << " points per frame; PCD contains " << pcd.size() << ": "
              << path << std::endl;
    return false;
  }

  double measurement_time = 0.0;
  for (const auto& point : pcd.points) {
    if (std::isfinite(point.timestamp)) {
      measurement_time = std::max(measurement_time, point.timestamp);
    }
  }
  cloud->set_measurement_time(measurement_time);
  for (const auto& point : pcd.points) {
    auto* output = cloud->add_point();
    output->set_x(point.x);
    output->set_y(point.y);
    output->set_z(point.z);
    output->set_intensity(point.intensity);
    if (std::isfinite(point.timestamp) && point.timestamp > 0.0) {
      const long double timestamp_ns =
          static_cast<long double>(point.timestamp) * 1.0e9L;
      if (timestamp_ns >
          static_cast<long double>(std::numeric_limits<uint64_t>::max())) {
        std::cerr << "PCD timestamp is out of range: " << path << std::endl;
        return false;
      }
      output->set_timestamp(static_cast<uint64_t>(timestamp_ns));
    }
  }
  return true;
}

bool LoadNuscenesBin(const std::string& path,
                     apollo::drivers::PointCloud* cloud) {
  constexpr std::size_t kCenterPointMaxPoints = 300000U;
  constexpr std::size_t kFeatureCount = 5U;
  constexpr std::size_t kBytesPerPoint = kFeatureCount * sizeof(float);
  std::ifstream input(path, std::ios::binary | std::ios::ate);
  if (!input.is_open()) {
    std::cerr << "Unable to read nuScenes binary: " << path << std::endl;
    return false;
  }
  const std::streamoff byte_count = input.tellg();
  if (byte_count < 0 ||
      byte_count % static_cast<std::streamoff>(kBytesPerPoint) != 0) {
    std::cerr << "nuScenes binary size is not a multiple of five float32 "
                 "features: "
              << path << std::endl;
    return false;
  }
  const std::size_t point_count =
      static_cast<std::size_t>(byte_count) / kBytesPerPoint;
  if (point_count == 0U || point_count > kCenterPointMaxPoints) {
    std::cerr << "CenterPoint accepts 1 to " << kCenterPointMaxPoints
              << " points per frame; nuScenes binary contains " << point_count
              << ": " << path << std::endl;
    return false;
  }
  std::vector<float> values(point_count * kFeatureCount);
  input.seekg(0, std::ios::beg);
  if (!input.read(reinterpret_cast<char*>(values.data()), byte_count)) {
    std::cerr << "Failed reading nuScenes binary: " << path << std::endl;
    return false;
  }
  cloud->set_measurement_time(0.0);
  for (std::size_t i = 0; i < point_count; ++i) {
    const std::size_t offset = i * kFeatureCount;
    auto* point = cloud->add_point();
    point->set_x(values[offset]);
    point->set_y(values[offset + 1U]);
    point->set_z(values[offset + 2U]);
    point->set_intensity(values[offset + 3U]);
  }
  return true;
}

bool LoadNuscenesModelBin(const std::string& path,
                          apollo::drivers::PointCloud* cloud) {
  constexpr std::size_t kCenterPointMaxPoints = 300000U;
  constexpr std::size_t kFeatureCount = 5U;
  constexpr std::size_t kBytesPerPoint = kFeatureCount * sizeof(float);
  std::ifstream input(path, std::ios::binary | std::ios::ate);
  if (!input.is_open()) {
    std::cerr << "Unable to read nuScenes model-feature binary: " << path
              << std::endl;
    return false;
  }
  const std::streamoff byte_count = input.tellg();
  if (byte_count < 0 ||
      byte_count % static_cast<std::streamoff>(kBytesPerPoint) != 0) {
    std::cerr << "nuScenes model-feature binary size is not a multiple of "
                 "five float32 features: "
              << path << std::endl;
    return false;
  }
  const std::size_t point_count =
      static_cast<std::size_t>(byte_count) / kBytesPerPoint;
  if (point_count == 0U || point_count > kCenterPointMaxPoints) {
    std::cerr << "CenterPoint accepts 1 to " << kCenterPointMaxPoints
              << " points per frame; nuScenes model-feature binary contains "
              << point_count << ": " << path << std::endl;
    return false;
  }
  std::vector<float> values(point_count * kFeatureCount);
  input.seekg(0, std::ios::beg);
  if (!input.read(reinterpret_cast<char*>(values.data()), byte_count)) {
    std::cerr << "Failed reading nuScenes model-feature binary: " << path
              << std::endl;
    return false;
  }
  cloud->set_measurement_time(0.0);
  for (std::size_t i = 0; i < point_count; ++i) {
    const std::size_t offset = i * kFeatureCount;
    const float time_lag = values[offset + 4U];
    if (!std::isfinite(time_lag) || time_lag < 0.0F) {
      std::cerr << "nuScenes model-feature binary has an invalid time-lag at "
                   "point "
                << i << ": " << path << std::endl;
      return false;
    }
    auto* point = cloud->add_point();
    point->set_x(values[offset]);
    point->set_y(values[offset + 1U]);
    point->set_z(values[offset + 2U]);
    point->set_intensity(values[offset + 3U]);
    if (time_lag > 0.0F) {
      const long double timestamp_ns =
          static_cast<long double>(time_lag) * 1.0e9L;
      if (timestamp_ns >
          static_cast<long double>(std::numeric_limits<uint64_t>::max())) {
        std::cerr << "nuScenes model-feature time-lag is out of range: " << path
                  << std::endl;
        return false;
      }
      point->set_timestamp(static_cast<uint64_t>(timestamp_ns));
    }
  }
  return true;
}

bool WriteInput(const ModelInput& input, const std::string& path) {
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  if (!output.is_open()) {
    std::cerr << "Unable to open output binary: " << path << std::endl;
    return false;
  }
  if (input.points.size() >
      static_cast<std::size_t>(std::numeric_limits<std::streamsize>::max()) /
          sizeof(float)) {
    std::cerr << "Packed point data is too large: " << path << std::endl;
    return false;
  }
  output.write(
      reinterpret_cast<const char*>(input.points.data()),
      static_cast<std::streamsize>(input.points.size() * sizeof(float)));
  output.flush();
  if (!output.good()) {
    std::cerr << "Failed writing point data: " << path << std::endl;
    return false;
  }
  return true;
}

}  // namespace

bool ConvertInputToCenterPointInput(const std::string& config_path,
                                    const std::string& input_path,
                                    const std::string& output_bin,
                                    const std::string& scn_onnx,
                                    const std::string& rpn_centerhead_engine,
                                    bool input_is_nuscenes_bin,
                                    bool input_is_nuscenes_model_bin) {
  CenterPointModelConfig config;
  if (!ReadConfig(config_path, &config)) {
    return false;
  }
  std::string error;
  if (!ValidateModelPaths(config, config_path, scn_onnx, rpn_centerhead_engine,
                          &error)) {
    std::cerr << error << std::endl;
    return false;
  }
  ModelInputBuilder builder;
  if (!builder.Init(config, &error)) {
    std::cerr << error << std::endl;
    return false;
  }

  apollo::drivers::PointCloud cloud;
  const bool loaded =
      input_is_nuscenes_bin         ? LoadNuscenesBin(input_path, &cloud)
      : input_is_nuscenes_model_bin ? LoadNuscenesModelBin(input_path, &cloud)
                                    : LoadPcd(input_path, &cloud);
  if (!loaded) {
    return false;
  }
  ModelInput input;
  if (!builder.Build(cloud, &input, &error)) {
    std::cerr << error << std::endl;
    return false;
  }
  constexpr std::size_t kCenterPointMaxPoints = 300000U;
  if (input.point_count() == 0U ||
      input.point_count() > kCenterPointMaxPoints) {
    std::cerr << "CenterPoint accepts 1 to " << kCenterPointMaxPoints
              << " valid points per frame; received " << input.point_count()
              << std::endl;
    return false;
  }
  if (!WriteInput(input, output_bin)) {
    return false;
  }
  std::cout << "Wrote " << input.point_count() << " points to " << output_bin
            << std::endl;
  return true;
}

}  // namespace lidar_object_detection
}  // namespace apollo

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  const bool has_pcd = !FLAGS_input_pcd.empty();
  const bool has_nuscenes_bin = !FLAGS_input_nuscenes_bin.empty();
  const bool has_nuscenes_model_bin = !FLAGS_input_nuscenes_model_bin.empty();
  if (FLAGS_config.empty() ||
      static_cast<int>(has_pcd) + static_cast<int>(has_nuscenes_bin) +
              static_cast<int>(has_nuscenes_model_bin) !=
          1 ||
      FLAGS_output_bin.empty() || FLAGS_scn_onnx.empty() ||
      FLAGS_rpn_centerhead_engine.empty()) {
    std::cerr << "Required flags: --config, exactly one of --input_pcd, "
                 "--input_nuscenes_bin, or --input_nuscenes_model_bin, "
                 "--output_bin, --scn_onnx, --rpn_centerhead_engine"
              << std::endl;
    return 2;
  }

  const std::string input_path = has_pcd ? FLAGS_input_pcd
                                 : has_nuscenes_bin
                                     ? FLAGS_input_nuscenes_bin
                                     : FLAGS_input_nuscenes_model_bin;
  return apollo::lidar_object_detection::ConvertInputToCenterPointInput(
             FLAGS_config, input_path, FLAGS_output_bin, FLAGS_scn_onnx,
             FLAGS_rpn_centerhead_engine, has_nuscenes_bin,
             has_nuscenes_model_bin)
             ? 0
             : 1;
}
