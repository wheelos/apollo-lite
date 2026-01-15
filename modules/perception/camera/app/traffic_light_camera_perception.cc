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
#include "modules/perception/camera/app/traffic_light_camera_perception.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/common/util/perf_util.h"
#include "modules/perception/camera/common/util.h"
#include "modules/perception/pipeline/pipeline.h"

namespace apollo {
namespace perception {
namespace camera {

using cyber::common::GetAbsolutePath;

bool TrafficLightCameraPerception::Init(const PipelineConfig &pipeline_config) {
  return Initialize(pipeline_config);
}

bool TrafficLightCameraPerception::Init(
    const CameraPerceptionInitOptions &options) {
  std::string work_root = options.use_cyber_work_root ? GetCyberWorkRoot() : "";
  std::string proto_path = GetAbsolutePath(options.root_dir, options.conf_file);
  proto_path = GetAbsolutePath(work_root, proto_path);

  if (!cyber::common::GetProtoFromFile(proto_path, &tl_param_)) {
    AERROR << "Load proto param failed, path: " << proto_path;
    return false;
  }

  // 1. Init Detector (One-Stage)
  TrafficLightDetectorInitOptions init_options;
  // 假设 proto 中 detector_param(0) 是检测器配置
  if (tl_param_.detector_param_size() == 0) {
    AERROR << "No detector param found.";
    return false;
  }
  auto plugin_param = tl_param_.detector_param(0).plugin_param();
  init_options.root_dir = GetAbsolutePath(work_root, plugin_param.root_dir());
  init_options.conf_file = plugin_param.config_file();
  init_options.gpu_id = tl_param_.gpu_id();

  detector_.reset(BaseTrafficLightDetectorRegisterer::GetInstanceByName(
      plugin_param.name()));
  if (!detector_ || !detector_->Init(init_options)) {
    AERROR << "Traffic Light Detector init failed: " << plugin_param.name();
    return false;
  }

  // 2. Init Tracker (Semantic Reviser)
  if (!tl_param_.has_tracker_param()) {
    AERROR << "No tracker param found.";
    return false;
  }
  TrafficLightTrackerInitOptions tracker_init_options;
  auto tracker_plugin_param = tl_param_.tracker_param().plugin_param();
  tracker_init_options.root_dir =
      GetAbsolutePath(work_root, tracker_plugin_param.root_dir());
  tracker_init_options.conf_file = tracker_plugin_param.config_file();

  tracker_.reset(BaseTrafficLightTrackerRegisterer::GetInstanceByName(
      tracker_plugin_param.name()));
  if (!tracker_ || !tracker_->Init(tracker_init_options)) {
    AERROR << "Traffic Light Tracker init failed: "
           << tracker_plugin_param.name();
    return false;
  }

  AINFO << "TrafficLight Pipeline Init Success. Detector: "
        << plugin_param.name() << ", Tracker: " << tracker_plugin_param.name();
  return true;
}

bool TrafficLightCameraPerception::Process(DataFrame *data_frame) {
  if (data_frame == nullptr) return false;
  return InnerProcess(data_frame);
}

bool TrafficLightCameraPerception::Perception(
    const CameraPerceptionOptions &options, CameraFrame *frame) {
  PERF_FUNCTION();
  if (frame == nullptr) return false;

  // 1. Detection (Includes classification in One-Stage)
  TrafficLightDetectorOptions detector_options;
  {
    PERF_BLOCK("TL_Detection");
    if (!detector_->Detect(detector_options, frame)) {
      AERROR << "TL detection failed.";
      return false;
    }
  }

  // 2. Tracking (Semantic Revision / Bayesian Filter)
  TrafficLightTrackerOptions tracker_options;
  tracker_options.time_stamp = frame->timestamp;
  {
    PERF_BLOCK("TL_Tracking");
    if (!tracker_->Track(tracker_options, frame)) {
      AERROR << "TL tracking failed.";
      return false;
    }
  }

  return true;
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
