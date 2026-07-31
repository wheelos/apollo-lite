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

#include "modules/drivers/camera_gst/frame_types.h"
#include "modules/drivers/camera_gst/pipeline_builder.h"

namespace apollo {
namespace drivers {
namespace camera_gst {

class CameraGstStreamer {
 public:
  // CPU callback types are maintained for backward compatibility; GPU-first
  // runtime should prefer GpuFrameCallback.
  using PublishCallback = std::function<void(PublishedFrame&&)>;
  using SourcePublishCallback =
      std::function<void(const std::string&, PublishedFrame&&)>;
  using GpuFrameCallback = std::function<void(GpuFrame&&)>;

  CameraGstStreamer() = default;
  ~CameraGstStreamer();

  bool Start(const config::Config& config,
             SourcePublishCallback source_publish_callback,
             PublishCallback stitched_publish_callback,
             GpuFrameCallback gpu_frame_callback);
  void Stop();
  bool StartStreaming();
  bool StopStreaming();
  bool streaming_active() const;
  int output_width() const { return output_width_; }
  int output_height() const { return output_height_; }
  StreamStats stats() const;

 private:
  enum class RunState {
    kStopped,
    kStarting,
    kRunning,
    kRecovering,
    kFailed,
  };

  struct SourceRuntimeState {
    explicit SourceRuntimeState(std::string name)
        : source_name(std::move(name)) {}

    std::string source_name;
    std::atomic<uint64_t> cpu_frames{0};
    std::atomic<uint64_t> gpu_frames{0};
    std::atomic<uint64_t> cpu_rate_limited_frames{0};
    std::atomic<uint64_t> cpu_drop_frames{0};
    std::atomic<uint64_t> gpu_drop_frames{0};
    std::atomic<uint64_t> published_frames{0};
    std::atomic<uint64_t> queue_drop_frames{0};
    std::atomic<uint64_t> last_sequence{0};
    std::atomic<uint32_t> queue_depth{0};
    std::atomic<double> last_measurement_time{0.0};
  };

  struct SinkContext {
    CameraGstStreamer* owner = nullptr;
    std::string source_name;
    SourceRuntimeState* source_state = nullptr;
    double min_publish_interval_sec = 0.0;
    double last_measurement_time = 0.0;
    bool has_last_measurement_time = false;
    bool stitched = false;
    bool cpu_readback = false;
  };

  static GstFlowReturn OnSourceSample(GstAppSink* appsink, gpointer user_data);
  static GstFlowReturn OnStitchedSample(GstAppSink* appsink,
                                        gpointer user_data);
  static GstFlowReturn OnGpuSample(GstAppSink* appsink, gpointer user_data);
  bool BuildPipelineLocked();
  bool ValidateConfigLocked();
  bool StartStreamBranchLocked();
  bool RebuildPipelineLocked(const std::string& reason);
  void StopStreamBranchDirectLocked();
  void ReleasePipelineLocked();
  bool ForceKeyFrameLocked();
  void BusLoop();
  void HandleBusMessage(GstMessage* message);
  std::string StreamBranchDescriptionLocked() const;
  const PipelineLayoutSlot* FindLayoutSlotLocked(
      const std::string& source_name) const;
  CameraGstPipelineBuilder MakePipelineBuilderLocked() const;

  mutable std::mutex mutex_;
  config::Config config_;
  std::vector<PipelineLayoutSlot> layout_slots_;
  std::vector<std::unique_ptr<SourceRuntimeState>> source_states_;
  SourcePublishCallback source_publish_callback_;
  PublishCallback stitched_publish_callback_;
  GpuFrameCallback gpu_frame_callback_;
  std::vector<std::unique_ptr<SinkContext>> source_sink_contexts_;
  std::vector<std::unique_ptr<SinkContext>> gpu_sink_contexts_;
  std::unique_ptr<SinkContext> stitched_sink_context_;
  std::thread bus_thread_;
  RunState state_ = RunState::kStopped;
  bool running_ = false;
  bool stop_requested_ = false;
  bool stream_enabled_ = false;
  bool stream_attached_ = false;
  int output_width_ = 0;
  int output_height_ = 0;
  uint64_t pipeline_warning_count_ = 0;
  uint64_t pipeline_error_count_ = 0;
  uint64_t pipeline_restart_count_ = 0;

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
