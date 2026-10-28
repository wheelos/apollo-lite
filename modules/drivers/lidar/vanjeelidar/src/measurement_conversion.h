// Copyright 2026 The WheelOS Authors. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <string>

#include <vanjee_driver/msg/imu_packet.hpp>
#include <vanjee_driver/msg/point_cloud_msg.hpp>

#include "modules/drivers/lidar/vanjeelidar/proto/vanjeelidar.pb.h"
#include "wheelos_msgs/sensor_msgs/pointcloud.pb.h"

namespace apollo {
namespace drivers {
namespace lidar {

using VanjeePoint = ::vanjee::lidar::PointXYZIRT;
using VanjeePointCloud = ::vanjee::lidar::PointCloudT<VanjeePoint>;

// Rejected inputs leave output unchanged; callers must report error.
bool ConvertVanjeePointCloud(const VanjeePointCloud& input,
                             bool use_offset_timestamp,
                             const std::string& frame_id, PointCloud* output,
                             std::string* error);

bool ConvertVanjeeImu(const ::vanjee::lidar::ImuPacket& input,
                      const std::string& frame_id, bool use_lidar_clock,
                      double publish_time, vanjee::VanjeeImu* output,
                      std::string* error);

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
