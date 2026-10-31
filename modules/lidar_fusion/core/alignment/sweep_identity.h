// Copyright 2026 WheelOS. All Rights Reserved.

#pragma once

#include "wheelos_msgs/sensor_msgs/pointcloud.pb.h"

namespace apollo {
namespace lidar_fusion {

bool SameSweep(const drivers::PointCloud& cloud, const common::Header& header);

}  // namespace lidar_fusion
}  // namespace apollo
