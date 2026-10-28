// Copyright 2026 WheelOS. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <memory>
#include <string>

#include "modules/lidar_fusion/core/fusion.h"

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
