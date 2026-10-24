// Copyright 2026 WheelOS. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <string>
#include <vector>

#include "wheelos_msgs/sensor_msgs/pointcloud.pb.h"

namespace apollo::lidar_object_detection {
bool BuildPointPillarsInput(const apollo::drivers::PointCloud& cloud,
                            const std::string& frame, float intensity_scale,
                            std::vector<float>* points, std::string* error);
}
