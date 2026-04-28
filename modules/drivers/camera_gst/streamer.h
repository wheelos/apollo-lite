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

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

#include "gst/app/gstappsink.h"
#include "gst/app/gstappsrc.h"
#include "gst/gst.h"
#include "opencv2/core/mat.hpp"

#include "modules/drivers/camera_gst/proto/config.pb.h"

namespace apollo {
namespace drivers {
namespace camera_gst {

class CameraGstStreamer {
 public:
  struct PublishedFrame {
    cv::Mat image_rgb;
    double measurement_time = 0.0;
  };

  using PublishCallback = std::function<void(PublishedFrame&&)>;

  CameraGstStreamer() = default;
  ~CameraGstStreamer();

  bool Start(int width, int height, double fps,
             const config::StreamConfig& config,
             PublishCallback publish_callback);
  void Stop();
  bool Submit(const cv::Mat& frame_bgr, double measurement_time);
  bool StartStreaming();
  bool StopStreaming();
  bool streaming_active() const;

 private:
  struct PendingFrame {
    cv::Mat image_bgr;
    double measurement_time = 0.0;
  };

  static GstFlowReturn OnNewSample(GstAppSink* appsink, gpointer user_data);
  static GstPadProbeReturn OnStreamPadIdle(GstPad* pad, GstPadProbeInfo* info,
                                           gpointer user_data);

  bool BuildPipelineLocked();
  bool StartStreamBranchLocked();
  void StopStreamBranchDirectLocked();
  void ReleasePipelineLocked();
  bool PushFrameToPipeline(const PendingFrame& frame);
  bool ForceKeyFrameLocked();
  std::string StreamBranchDescription() const;
  void FeedLoop();
  void CompleteIdleDetach(GstPad* tee_pad);

  int width_ = 0;
  int height_ = 0;
  double fps_ = 0.0;
  config::StreamConfig config_;
  PublishCallback publish_callback_;
  size_t max_pending_frames_ = 3;

  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<PendingFrame> pending_frames_;
  std::thread worker_;
  bool running_ = false;
  bool stop_requested_ = false;
  bool stream_attached_ = false;
  bool stream_detach_requested_ = false;

  GstElement* pipeline_ = nullptr;
  GstElement* appsrc_ = nullptr;
  GstElement* tee_ = nullptr;
  GstElement* publish_queue_ = nullptr;
  GstElement* publish_convert_ = nullptr;
  GstElement* publish_capsfilter_ = nullptr;
  GstElement* appsink_ = nullptr;
  GstElement* stream_branch_bin_ = nullptr;
  GstPad* publish_tee_pad_ = nullptr;
  GstPad* stream_tee_pad_ = nullptr;
  GstPad* stream_sink_pad_ = nullptr;
};

}  // namespace camera_gst
}  // namespace drivers
}  // namespace apollo
