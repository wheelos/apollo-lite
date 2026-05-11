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

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "gst/app/gstappsink.h"
#include "gst/gst.h"

#include "modules/drivers/camera_gst/proto/config.pb.h"

namespace apollo {
namespace drivers {
namespace camera_gst {

class CameraGstStreamer {
 public:
  struct PublishedFrame {
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t step = 0;
    double measurement_time = 0.0;
    std::string encoding = "rgb8";
    std::string data;
  };

  using PublishCallback = std::function<void(PublishedFrame&&)>;
  using SourcePublishCallback =
      std::function<void(const std::string&, PublishedFrame&&)>;

  CameraGstStreamer() = default;
  ~CameraGstStreamer();

  bool Start(const config::Config& config,
             SourcePublishCallback source_publish_callback,
             PublishCallback stitched_publish_callback);
  void Stop();
  bool StartStreaming();
  bool StopStreaming();
  bool streaming_active() const;
  int output_width() const { return output_width_; }
  int output_height() const { return output_height_; }

 private:
  struct LayoutSlot {
    std::string source_name;
    size_t pad_index = 0;
    int row = 0;
    int col = 0;
  };

  struct SinkContext {
    CameraGstStreamer* owner = nullptr;
    std::string source_name;
    bool stitched = false;
  };

  static GstFlowReturn OnSourceSample(GstAppSink* appsink, gpointer user_data);
  static GstFlowReturn OnStitchedSample(GstAppSink* appsink,
                                        gpointer user_data);
  bool BuildPipelineLocked();
  bool ValidateConfigLocked();
  bool StartStreamBranchLocked();
  void StopStreamBranchDirectLocked();
  void ReleasePipelineLocked();
  bool ForceKeyFrameLocked();
  void BusLoop();
  PublishedFrame ExtractFrame(GstSample* sample) const;
  std::string StreamBranchDescriptionLocked() const;
  std::string BuildPipelineDescriptionLocked() const;
  std::string BuildCompositorDescriptionLocked() const;
  std::string BuildSourceDescriptionLocked(
      size_t source_index, const config::CameraSourceConfig& source_config,
      const LayoutSlot* layout_slot) const;
  const LayoutSlot* FindLayoutSlotLocked(const std::string& source_name) const;
  std::string SourceTeeName(size_t source_index) const;
  std::string SourcePublishSinkName(size_t source_index) const;

  mutable std::mutex mutex_;
  config::Config config_;
  std::vector<LayoutSlot> layout_slots_;
  SourcePublishCallback source_publish_callback_;
  PublishCallback stitched_publish_callback_;
  std::vector<std::unique_ptr<SinkContext>> source_sink_contexts_;
  std::unique_ptr<SinkContext> stitched_sink_context_;
  std::thread bus_thread_;
  bool running_ = false;
  bool stop_requested_ = false;
  bool stream_enabled_ = false;
  bool stream_attached_ = false;
  int output_width_ = 0;
  int output_height_ = 0;

  GstElement* pipeline_ = nullptr;
  GstElement* stitched_tee_ = nullptr;
  GstElement* stitched_publish_sink_ = nullptr;
  GstElement* stream_branch_bin_ = nullptr;
  GstBus* bus_ = nullptr;
  GstPad* stream_tee_pad_ = nullptr;
  GstPad* stream_sink_pad_ = nullptr;
};

}  // namespace camera_gst
}  // namespace drivers
}  // namespace apollo
