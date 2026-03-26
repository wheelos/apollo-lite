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
#include "modules/perception/traffic_light/application/traffic_light_perception_pipeline.h"

#include <utility>

#include "cyber/common/log.h"
#include "modules/perception/traffic_light/domain/traffic_light_pipeline_config.h"
#include "modules/perception/traffic_light/infra/runtime_stage_factory.h"

namespace apollo {
namespace perception {
namespace traffic_light {

TrafficLightPerceptionPipeline::~TrafficLightPerceptionPipeline() {
  Stop();
}

bool TrafficLightPerceptionPipeline::Stop() {
  bool ok = true;
  if (tracker_ != nullptr) {
    ok = tracker_->Stop() && ok;
  }
  if (detector_ != nullptr) {
    ok = detector_->Stop() && ok;
  }
  runtime_state_.initialized = false;
  return ok;
}

bool TrafficLightPerceptionPipeline::Init(
    const pipeline::PipelineConfig& pipeline_config) {
  Stop();

  const domain::TrafficLightStageBundle stage_bundle =
      domain::ResolveTrafficLightStageBundle(pipeline_config);
  if (!stage_bundle.IsValid()) {
    AERROR << "Traffic light pipeline config is incomplete.";
    return false;
  }

  detector_ =
      infra::CreateRuntimeTrafficLightDetector(*stage_bundle.detector_stage);
  if (detector_ == nullptr) {
    AERROR << "Failed to create traffic light detector runtime stage.";
    return false;
  }
  if (!detector_->Configure(*stage_bundle.detector_stage)) {
    AERROR << "Failed to configure traffic light detector: "
           << detector_->Name();
    return false;
  }
  if (!detector_->Start()) {
    AERROR << "Failed to start traffic light detector: "
           << detector_->Name();
    return false;
  }

  tracker_ =
      infra::CreateRuntimeTrafficLightTracker(*stage_bundle.tracker_stage);
  if (tracker_ == nullptr) {
    AERROR << "Failed to create traffic light tracker runtime stage.";
    return false;
  }
  if (!tracker_->Configure(*stage_bundle.tracker_stage)) {
    AERROR << "Failed to configure traffic light tracker: "
           << tracker_->Name();
    return false;
  }
  if (!tracker_->Start()) {
    AERROR << "Failed to start traffic light tracker: "
           << tracker_->Name();
    detector_->Stop();
    return false;
  }

  runtime_state_.initialized = true;
  runtime_state_.detector_name = detector_->Name();
  runtime_state_.tracker_name = tracker_->Name();
  runtime_state_.last_processed_timestamp = -1.0;
  runtime_state_.processed_frame_count = 0;
  return true;
}

bool TrafficLightPerceptionPipeline::Perception(
    const camera::CameraPerceptionOptions& options,
    camera::CameraFrame* frame) {
  (void)options;
  if (!runtime_state_.initialized || frame == nullptr) {
    return false;
  }

  application::TrafficLightDetectorExecutionContext detector_context;
  detector_context.timestamp_sec = frame->timestamp;
  if (frame->data_provider != nullptr) {
    detector_context.camera_name = frame->data_provider->sensor_name();
  }
  if (!detector_->Run(detector_context, frame)) {
    AERROR << "Traffic light detector failed: " << detector_->Name();
    return false;
  }

  application::TrafficLightTrackerExecutionContext tracker_context;
  tracker_context.timestamp_sec = frame->timestamp;
  tracker_context.camera_name = detector_context.camera_name;
  if (!tracker_->Run(tracker_context, frame)) {
    AERROR << "Traffic light tracker failed: " << tracker_->Name();
    return false;
  }

  runtime_state_.last_processed_timestamp = frame->timestamp;
  ++runtime_state_.processed_frame_count;
  return true;
}

bool TrafficLightPerceptionPipeline::Process(DataFrame* data_frame) {
  if (data_frame == nullptr || data_frame->camera_frame == nullptr) {
    return false;
  }
  camera::CameraPerceptionOptions options;
  return Perception(options, data_frame->camera_frame);
}

std::string TrafficLightPerceptionPipeline::Name() const {
  return "TrafficLightPerceptionPipeline";
}

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
