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

#include <deque>
#include <memory>
#include <mutex>

#include "cyber/component/component.h"
#include "cyber/component/timer_component.h"
#include "modules/lidar_fusion/core/alignment/frame_buffer.h"
#include "modules/lidar_fusion/core/tracking/temporal.h"
#include "modules/lidar_fusion/publication/pod.h"

namespace apollo {
namespace lidar_fusion {

class LidarFusionComponent final : public cyber::TimerComponent {
 public:
  bool Init() override;
  bool Proc() override;

 private:
  void Clear() override;
  bool PublishMain(const std::shared_ptr<FusionOutput>& output);
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
  std::shared_ptr<cyber::Writer<FusionOutput>> diagnostic_writer_;
  std::unique_ptr<FusionPodPublisher> pod_writer_;
  std::unique_ptr<FusionPodPublisher> pod_diagnostic_writer_;
};

CYBER_REGISTER_COMPONENT(LidarFusionComponent)

}  // namespace lidar_fusion
}  // namespace apollo
