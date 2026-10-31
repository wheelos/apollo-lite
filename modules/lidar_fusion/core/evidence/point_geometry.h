// Copyright 2026 WheelOS. All Rights Reserved.

#pragma once

#include <utility>

#include "Eigen/Core"

#include "modules/lidar_fusion/core/association/candidates.h"
#include "modules/lidar_fusion/core/runtime/single_frame_fusion.h"

namespace apollo {
namespace lidar_fusion {
namespace fusion_detail {

using CellKey = std::pair<int, int>;
using SpatialIndex = PointIndex;

struct OrientedBox {
  Eigen::Vector3d center;
  Eigen::Vector3d size;
  Eigen::Matrix3d rotation;
};

Eigen::Matrix3d Rotation(double roll, double pitch, double yaw);
void SetVector(const Eigen::Vector3d& value, Vec3* output);
Eigen::Vector3d Vector(const Vec3& value);
void SetBox(const OrientedBox& box, Box* output);
bool InBox(const Eigen::Vector3d& point, const OrientedBox& box, double margin);
CellKey Key(double x, double y, double size);
bool InRoi(const Eigen::Vector3d& point, const FusionConfig& config);
bool InCorridor(const Eigen::Vector3d& point, const FusionConfig& config);
std::pair<CellKey, CellKey> BoxRange(const OrientedBox& box,
                                     const FusionConfig& config, double margin);
bool ObstacleHeight(const PointEvidence& point, const FusionConfig& config);

}  // namespace fusion_detail
}  // namespace lidar_fusion
}  // namespace apollo
