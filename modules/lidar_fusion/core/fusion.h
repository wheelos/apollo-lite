// Copyright 2026 WheelOS. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "modules/lidar_fusion/proto/lidar_fusion.pb.h"
#include "modules/lidar_semantic_segmentation/proto/lidar_semantic_segmentation.pb.h"
#include "wheelos_msgs/sensor_msgs/pointcloud.pb.h"

namespace apollo {
namespace lidar_fusion {

using DetectionFrame = lidar_object_detection::LidarObjectDetectionFrameResult;
namespace segmentation = lidar_semantic_segmentation;
using SemanticFrame = segmentation::LidarSemanticSegmentationResult;

bool SameSweep(const drivers::PointCloud& cloud, const common::Header& header);

using GroundPrior = std::vector<Vec3>;
using ObjectPrior = std::vector<ObjectCandidate>;

struct FusionWork {
  // Evidence, components, association, candidates, relations, space,
  // visibility, and output. Opt-in timing is separate from the published
  // evidence.
  std::array<double, 8> stage_ms{};
  uint64_t voxels = 0;
  uint64_t neighbor_edges = 0;
  uint64_t relation_pairs = 0;
  uint64_t relation_point_transforms = 0;
};

class SingleFrameFusion {
 public:
  bool Init(const FusionConfig& config, std::string* error);
  bool Process(const drivers::PointCloud& cloud,
               const DetectionFrame* detection,
               const SemanticFrame* segmentation, FusionOutput* output,
               std::string* error, const GroundPrior* ground_prior = nullptr,
               const ObjectPrior* object_prior = nullptr,
               FusionWork* work = nullptr) const;

 private:
  FusionConfig config_;
  bool initialized_ = false;
};

}  // namespace lidar_fusion
}  // namespace apollo
