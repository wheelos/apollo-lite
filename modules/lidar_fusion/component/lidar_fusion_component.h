// Copyright 2026 WheelOS. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <deque>
#include <memory>
#include <mutex>

#include "cyber/component/component.h"
#include "cyber/component/timer_component.h"
#include "modules/lidar_fusion/core/frame_buffer.h"
#include "modules/lidar_fusion/core/temporal.h"

namespace apollo {
namespace lidar_fusion {

class LidarFusionComponent final : public cyber::TimerComponent {
 public:
  bool Init() override;
  bool Proc() override;

 private:
  void Clear() override;
  FusionConfig config_;
  StatefulFusion fusion_;
  std::unique_ptr<FrameBuffer> buffer_;
  std::mutex buffer_mutex_;
  std::mutex process_mutex_;
  std::shared_ptr<cyber::Reader<drivers::PointCloud>> cloud_reader_;
  std::shared_ptr<cyber::Reader<DetectionFrame>> detection_reader_;
  std::shared_ptr<cyber::Reader<SemanticFrame>> segmentation_reader_;
  std::shared_ptr<cyber::Reader<LocalPose>> pose_reader_;
  std::deque<std::shared_ptr<const LocalPose>> poses_;
  std::shared_ptr<cyber::Writer<FusionOutput>> writer_;
};

CYBER_REGISTER_COMPONENT(LidarFusionComponent)

}  // namespace lidar_fusion
}  // namespace apollo
