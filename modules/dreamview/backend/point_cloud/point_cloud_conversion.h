// Copyright 2026 WheelOS. All Rights Reserved.
#pragma once

#include <memory>

#include "Eigen/Geometry"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "wheelos_msgs/sensor_msgs/pointcloud.pb.h"

namespace apollo {
namespace dreamview {

pcl::PointCloud<pcl::PointXYZ>::Ptr ConvertPCLPointCloud(
    const drivers::PointCloud& cloud,
    const Eigen::Affine3d& target_from_source);

}  // namespace dreamview
}  // namespace apollo
