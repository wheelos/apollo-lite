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

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <string>

#include "gflags/gflags.h"
#include "google/protobuf/text_format.h"
#include "modules/lidar_object_detection/proto/lidar_object_detection.pb.h"
#include "modules/lidar_object_detection/types/model_input.h"
#include "modules/perception/lidar/common/pcl_util.h"
#include "pcl/io/pcd_io.h"
#include "wheelos_msgs/sensor_msgs/pointcloud.pb.h"

DEFINE_string(config, "", "Text-format NvidiaCenterPointConfig path.");
DEFINE_string(input_pcd, "", "Input PCD path.");
DEFINE_string(output_bin, "", "Output packed five-feature binary path.");

namespace apollo {
namespace lidar_object_detection {
namespace {

bool ReadConfig(const std::string& path, NvidiaCenterPointConfig* config) {
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

bool LoadPcd(const std::string& path, apollo::drivers::PointCloud* cloud) {
  pcl::PointCloud<apollo::perception::lidar::PCLPointXYZIT> pcd;
  if (pcl::io::loadPCDFile(path, pcd) < 0) {
    std::cerr << "Unable to read PCD: " << path << std::endl;
    return false;
  }
  constexpr std::size_t kNvidiaMaxPoints = 300000U;
  if (pcd.size() > kNvidiaMaxPoints) {
    std::cerr << "NVIDIA CenterPoint accepts at most " << kNvidiaMaxPoints
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
  output.write(reinterpret_cast<const char*>(input.points.data()),
               static_cast<std::streamsize>(input.points.size() *
                                            sizeof(float)));
  output.flush();
  if (!output.good()) {
    std::cerr << "Failed writing point data: " << path << std::endl;
    return false;
  }
  return true;
}

}  // namespace

bool ConvertPcdToNvidiaBin(const std::string& config_path,
                           const std::string& input_pcd,
                           const std::string& output_bin) {
  NvidiaCenterPointConfig config;
  if (!ReadConfig(config_path, &config)) {
    return false;
  }
  ModelInputBuilder builder;
  std::string error;
  if (!builder.Init(config, &error)) {
    std::cerr << error << std::endl;
    return false;
  }

  apollo::drivers::PointCloud cloud;
  if (!LoadPcd(input_pcd, &cloud)) {
    return false;
  }
  ModelInput input;
  if (!builder.Build(cloud, &input, &error)) {
    std::cerr << error << std::endl;
    return false;
  }
  constexpr std::size_t kNvidiaMaxPoints = 300000U;
  if (input.point_count() == 0U || input.point_count() > kNvidiaMaxPoints) {
    std::cerr << "NVIDIA CenterPoint accepts 1 to " << kNvidiaMaxPoints
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
  if (FLAGS_config.empty() || FLAGS_input_pcd.empty() ||
      FLAGS_output_bin.empty()) {
    std::cerr << "Required flags: --config, --input_pcd, --output_bin"
              << std::endl;
    return 2;
  }

  return apollo::lidar_object_detection::ConvertPcdToNvidiaBin(
             FLAGS_config, FLAGS_input_pcd, FLAGS_output_bin)
             ? 0
             : 1;
}
