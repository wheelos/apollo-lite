/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

#include <string>

#include "cyber/common/macros.h"
#include "modules/perception/camera/common/camera_frame.h"
#include "modules/perception/camera/lib/interface/base_init_options.h"
#include "modules/perception/lib/registerer/registerer.h"
#include "modules/perception/pipeline/stage.h"

namespace apollo {
namespace perception {
namespace camera {

struct TrafficLightTrackerInitOptions : public BaseInitOptions {};

struct TrafficLightTrackerOptions {
  double time_stamp;
};

struct TrafficLightTrackerRuntimeContext {
  double timestamp_sec = -1.0;
  std::string camera_name;
  bool enable_debug = false;
};

class BaseTrafficLightTracker : public pipeline::Stage {
 public:
  using StageConfig = pipeline::StageConfig;
  using DataFrame = pipeline::DataFrame;

 public:
  BaseTrafficLightTracker() = default;

  virtual ~BaseTrafficLightTracker() = default;

  virtual bool Init(const TrafficLightTrackerInitOptions& options =
                        TrafficLightTrackerInitOptions()) = 0;

  // @brief: track detected traffic_light.
  // @param [in]: options
  // @param [in/out]: frame
  // traffic_light type and 2D bbox should be filled, required,
  virtual bool Track(const TrafficLightTrackerOptions& options,
                     CameraFrame* frame) = 0;

  // Migration-friendly entry for module-owned stages.
  virtual bool Track(const TrafficLightTrackerRuntimeContext& context,
                     CameraFrame* frame) {
    TrafficLightTrackerOptions options;
    options.time_stamp = context.timestamp_sec;
    return Track(options, frame);
  }

  virtual bool Init(const StageConfig& stage_config) {
    if (!Initialize(stage_config)) {
      return false;
    }
    return Init(TrafficLightTrackerInitOptions());
  }

  virtual bool Process(DataFrame* data_frame) {
    if (data_frame == nullptr || data_frame->camera_frame == nullptr) {
      return false;
    }
    TrafficLightTrackerOptions options;
    options.time_stamp = data_frame->camera_frame->timestamp;
    return Track(options, data_frame->camera_frame);
  }

  virtual bool ValidateStageConfig(const StageConfig& stage_config) const {
    (void)stage_config;
    return true;
  }

  virtual bool Shutdown() { return true; }

  virtual bool IsEnabled() const { return enable_; }

  virtual std::string Name() const = 0;

  DISALLOW_COPY_AND_ASSIGN(BaseTrafficLightTracker);
};  // class BaseTrafficLightTracker

PERCEPTION_REGISTER_REGISTERER(BaseTrafficLightTracker);
#define REGISTER_TRAFFIC_LIGHT_TRACKER(name) \
  PERCEPTION_REGISTER_CLASS(BaseTrafficLightTracker, name)

}  // namespace camera
}  // namespace perception
}  // namespace apollo
