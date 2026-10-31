// Copyright 2026 WheelOS. All Rights Reserved.

#pragma once

#include <cstdint>
#include <limits>
#include <set>
#include <vector>

#include "Eigen/Geometry"

#include "modules/lidar_fusion/core/association/candidates.h"

namespace apollo {
namespace lidar_fusion {
namespace candidate_detail {

struct PointBounds {
  Eigen::Vector3d low =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d high = -low;
  bool valid = false;
};

struct PointBlock {
  int begin = 0;
  int end = 0;
  PointBounds bounds;
};

struct ObjectSupport {
  explicit ObjectSupport(const ObjectCandidate& object);

  bool Separated(const PointBounds& points, double adjacent) const;

  Eigen::Quaterniond rotation;
  std::set<uint32_t> ids;
  PointBounds bounds;
  double scale = 0;
};

PointBounds ActualBounds(
    const google::protobuf::RepeatedPtrField<PointEvidence>& evidence,
    const google::protobuf::RepeatedField<uint32_t>& ids);
std::vector<PointBlock> PointBlocks(
    const google::protobuf::RepeatedPtrField<PointEvidence>& evidence,
    const UnknownCandidate& unknown);

}  // namespace candidate_detail
}  // namespace lidar_fusion
}  // namespace apollo
