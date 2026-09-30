/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

#include <memory>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "modules/perception/onboard/proto/fusion_component_config.pb.h"
#include "modules/perception/pipeline/proto/pipeline_config.pb.h"

#include "cyber/component/component.h"
#include "cyber/timer/timer.h"
#include "modules/perception/base/object.h"
#include "modules/perception/fusion/app/fusion_runtime.h"
#include "modules/perception/fusion/lib/interface/base_fusion_system.h"
#include "modules/perception/fusion/lib/interface/base_multisensor_fusion.h"
#include "modules/perception/onboard/inner_component_messages/inner_component_messages.h"

namespace apollo {
namespace perception {
namespace onboard {

class MultiSensorFusionComponent : public cyber::Component<SensorFrameMessage> {
 public:
  MultiSensorFusionComponent() = default;
  ~MultiSensorFusionComponent() override;
  bool Init() override;
  bool Proc(const std::shared_ptr<SensorFrameMessage>& message) override;

 private:
  bool InitAlgorithmPlugin();
  void PublishTick();

 private:
  std::mutex mutex_;
  uint32_t sequence_ = 0;
  uint64_t lidar_timestamp_ = 0;
  std::map<double, uint64_t> pending_lidar_timestamps_;
  fusion::FusionRuntime runtime_;
  std::unique_ptr<cyber::Timer> timer_;

  std::string fusion_name_;
  std::string fusion_method_;
  std::string fusion_main_sensor_;

  pipeline::PipelineConfig multi_sensor_fusion_config_;

  std::unique_ptr<fusion::BaseMultiSensorFusion> fusion_;
  std::shared_ptr<apollo::cyber::Writer<PerceptionObstacles>> writer_;
  std::shared_ptr<apollo::cyber::Writer<SensorFrameMessage>> inner_writer_;
};

CYBER_REGISTER_COMPONENT(MultiSensorFusionComponent);

}  // namespace onboard
}  // namespace perception
}  // namespace apollo
