/******************************************************************************
 * Copyright 2026 WheelOS All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/perception/common/io/resource_path.h"

#include <filesystem>

#include "cyber/common/log.h"
#include "modules/common/vehicle_calibration/registry.h"

namespace apollo {
namespace perception {
namespace common {

std::string PerceptionResourcePath::ResolveCalibrationPath(
    const std::string& file_name) {
  apollo::common::vehicle_calibration::Registry registry;
  if (!registry.LoadFromEnvironment()) {
    return "";
  }
  for (const auto& sensor : registry.sensors()) {
    if (std::filesystem::path(sensor.calibration.extrinsic_file())
            .filename()
            .string() == file_name) {
      return sensor.resolved_extrinsic_file;
    }
    if (std::filesystem::path(sensor.calibration.intrinsic_file())
            .filename()
            .string() == file_name) {
      return sensor.resolved_intrinsic_file;
    }
  }
  if (!registry.resolved_lidar_height_file().empty() &&
      std::filesystem::path(registry.manifest().lidar_height_file())
              .filename()
              .string() == file_name) {
    return registry.resolved_lidar_height_file();
  }
  AERROR << "Calibration file is not listed in selected profile: "
         << file_name;
  return "";
}

std::string PerceptionResourcePath::ResolveExtrinsicPath(
    const std::string& file_name) {
  return ResolveCalibrationPath(file_name);
}

std::string PerceptionResourcePath::ResolveLidarHeightPath() {
  apollo::common::vehicle_calibration::Registry registry;
  if (!registry.LoadFromEnvironment()) {
    return "";
  }
  if (registry.resolved_lidar_height_file().empty()) {
    AERROR << "No LiDAR height calibration in selected vehicle profile.";
    return "";
  }
  return registry.resolved_lidar_height_file();
}

}  // namespace common
}  // namespace perception
}  // namespace apollo
