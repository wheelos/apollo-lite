/******************************************************************************
 * Copyright 2026 The WheelOS Team. All Rights Reserved.
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

#include <functional>
#include <memory>
#include <vector>

#include "modules/drivers/camera_gst/frame_source.h"
#include "modules/drivers/camera_gst/frame_stitcher.h"
#include "modules/drivers/camera_gst/proto/config.pb.h"
#include "modules/drivers/camera_gst/streamer.h"

namespace apollo {
namespace drivers {
namespace camera_gst {

class CameraGstDriver {
 public:
  using PublishCallback = CameraGstStreamer::PublishCallback;

  explicit CameraGstDriver(
      std::unique_ptr<CameraGstStreamer> streamer = nullptr);
  ~CameraGstDriver();

  bool Init(const config::Config& config, PublishCallback publish_callback);
  bool CaptureStitchedFrame(cv::Mat* stitched_bgr, double* measurement_time);
  bool SubmitFrame(const cv::Mat& stitched_bgr, double measurement_time);

  void StartStreaming();
  void StopStreaming();

  int output_width() const;
  int output_height() const;

 private:
  config::Config config_;
  std::vector<std::unique_ptr<FrameSource>> sources_;
  std::unique_ptr<GridFrameStitcher> stitcher_;
  std::unique_ptr<CameraGstStreamer> streamer_;
  bool stream_enabled_ = false;
  bool stream_started_ = false;
};

}  // namespace camera_gst
}  // namespace drivers
}  // namespace apollo
