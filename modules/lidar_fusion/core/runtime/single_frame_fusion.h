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
