/******************************************************************************
 * Copyright 2026 The Wheel.OS Authors. All Rights Reserved.
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
#include <string>

#include "modules/perception/camera/common/camera_frame.h"
#include "modules/perception/camera/lib/interface/base_camera_perception.h"
#include "modules/perception/traffic_light/domain/traffic_light_runtime_state.h"
#include "modules/perception/traffic_light/application/ports/traffic_light_detector_port.h"
#include "modules/perception/traffic_light/application/ports/traffic_light_tracker_port.h"
#include "modules/perception/pipeline/data_frame.h"
#include "modules/perception/pipeline/proto/pipeline_config.pb.h"

namespace apollo {
namespace perception {
namespace traffic_light {

class TrafficLightPerceptionPipeline {
 public:
  using DataFrame = pipeline::DataFrame;

  TrafficLightPerceptionPipeline() = default;
  ~TrafficLightPerceptionPipeline();

  bool Init(const pipeline::PipelineConfig& pipeline_config);
  bool Stop();
  bool Perception(const camera::CameraPerceptionOptions& options,
                  camera::CameraFrame* frame);
  bool Process(DataFrame* data_frame);
  std::string Name() const;

 private:
  std::unique_ptr<application::TrafficLightDetectorPort> detector_;
  std::unique_ptr<application::TrafficLightTrackerPort> tracker_;
  domain::TrafficLightRuntimeState runtime_state_;
};

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
