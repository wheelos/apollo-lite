// Copyright 2026 WheelOS. All Rights Reserved.

#pragma once

#include <string>

#include "modules/lidar_fusion/proto/lidar_fusion.pb.h"
#include "wheelos_msgs/sensor_msgs/pointcloud.pb.h"

namespace apollo {
namespace lidar_fusion {

bool ProjectFunctionalOutput(const FusionConfig& config,
                             const drivers::PointCloud& cloud,
                             const FusionOutput& full, FusionOutput* output,
                             std::string* error);

}  // namespace lidar_fusion
}  // namespace apollo
