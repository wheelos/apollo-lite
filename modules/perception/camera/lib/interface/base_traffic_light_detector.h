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

#include <memory>
#include <string>

#include "cyber/common/macros.h"
#include "modules/perception/camera/common/camera_frame.h"
#include "modules/perception/lib/registerer/registerer.h"

#include "modules/perception/base/camera.h"
#include "modules/perception/camera/lib/interface/base_init_options.h"
#include "modules/perception/pipeline/stage.h"

namespace apollo {
namespace perception {
namespace camera {

struct TrafficLightDetectorInitOptions : public BaseInitOptions {
  std::shared_ptr<base::BaseCameraModel> base_camera_model = nullptr;
};

struct TrafficLightDetectorOptions {};

struct TrafficLightDetectorRuntimeContext {
  double timestamp_sec = -1.0;
  std::string camera_name;
  bool enable_debug = false;
};

class BaseTrafficLightDetector : public pipeline::Stage {
 public:
  using StageConfig = pipeline::StageConfig;
  using DataFrame = pipeline::DataFrame;

 public:
  BaseTrafficLightDetector() = default;

  virtual ~BaseTrafficLightDetector() = default;

  virtual bool Init(const TrafficLightDetectorInitOptions& options =
                        TrafficLightDetectorInitOptions()) = 0;

  // @brief: detect traffic_light from image.
  // @param [in]: options
  // @param [in/out]: frame
  // traffic_light type and 2D bbox should be filled, required,
  virtual bool Detect(const TrafficLightDetectorOptions& options,
                      CameraFrame* frame) = 0;

  // Migration-friendly entry for module-owned stages.
  virtual bool Detect(const TrafficLightDetectorRuntimeContext& context,
                      CameraFrame* frame) {
    (void)context;
    TrafficLightDetectorOptions options;
    return Detect(options, frame);
  }

  virtual bool Init(const StageConfig& stage_config) {
    if (!Initialize(stage_config)) {
      return false;
    }
    return Init(TrafficLightDetectorInitOptions());
  }

  virtual bool Process(DataFrame* data_frame) {
    if (data_frame == nullptr || data_frame->camera_frame == nullptr) {
      return false;
    }
    TrafficLightDetectorOptions options;
    return Detect(options, data_frame->camera_frame);
  }

  virtual bool ValidateStageConfig(const StageConfig& stage_config) const {
    (void)stage_config;
    return true;
  }

  virtual bool Shutdown() { return true; }

  virtual bool IsEnabled() const { return enable_; }

  virtual std::string Name() const = 0;

  DISALLOW_COPY_AND_ASSIGN(BaseTrafficLightDetector);
};  // class BaseTrafficLightDetector

PERCEPTION_REGISTER_REGISTERER(BaseTrafficLightDetector);
#define REGISTER_TRAFFIC_LIGHT_DETECTOR(name) \
  PERCEPTION_REGISTER_CLASS(BaseTrafficLightDetector, name)

}  // namespace camera
}  // namespace perception
}  // namespace apollo
