// Copyright 2026 WheelOS. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>
#include <deque>
#include <memory>

#include "modules/lidar_fusion/core/fusion.h"

namespace apollo {
namespace lidar_fusion {

struct ReadyFrame {
  std::shared_ptr<const drivers::PointCloud> cloud;
  std::shared_ptr<const DetectionFrame> detection;
  std::shared_ptr<const SemanticFrame> segmentation;
};

// Externally serialized. Deadlines use receiver steady time, not sensor clocks.
class FrameBuffer {
 public:
  explicit FrameBuffer(const FusionConfig& config) : config_(config) {}
  bool PushCloud(std::shared_ptr<const drivers::PointCloud> cloud,
                 double received_sec);
  bool PushDetection(std::shared_ptr<const DetectionFrame> detection);
  bool PushSemantic(std::shared_ptr<const SemanticFrame> segmentation);
  bool Pop(double now_sec, ReadyFrame* ready);
  uint64_t dropped_clouds() const { return dropped_clouds_; }
  uint64_t dropped_results() const { return dropped_results_; }
  uint64_t queue_overflows() const { return queue_overflows_; }

 private:
  struct PendingCloud {
    std::shared_ptr<const drivers::PointCloud> cloud;
    double received_sec = 0;
  };
  FusionConfig config_;
  std::deque<PendingCloud> clouds_;
  std::deque<std::shared_ptr<const DetectionFrame>> detections_;
  std::deque<std::shared_ptr<const SemanticFrame>> semantics_;
  double last_cloud_stamp_ = 0;
  uint64_t dropped_clouds_ = 0;
  uint64_t dropped_results_ = 0;
  uint64_t queue_overflows_ = 0;
};

}  // namespace lidar_fusion
}  // namespace apollo
