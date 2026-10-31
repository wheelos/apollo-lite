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

#include <memory>
#include <string>

#include "modules/lidar_fusion/core/runtime/single_frame_fusion.h"

namespace apollo {
namespace lidar_fusion {

// Owns serialized cross-frame state. SingleFrameFusion remains stateless.
class StatefulFusion {
 public:
  StatefulFusion();
  ~StatefulFusion();
  bool Init(const FusionConfig& config, std::string* error);
  bool Process(const drivers::PointCloud& cloud,
               const DetectionFrame* detection,
               const SemanticFrame* segmentation, const LocalPose* pose,
               FusionOutput* output, std::string* error);
  void Reset();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace lidar_fusion
}  // namespace apollo
