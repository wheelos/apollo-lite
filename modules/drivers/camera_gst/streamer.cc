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

#include "modules/drivers/camera_gst/streamer.h"

#include <algorithm>
#include <chrono>
#include <set>
#include <thread>
#include <utility>

#include "cyber/cyber.h"
#include "modules/drivers/camera_gst/frame_extractor.h"

namespace apollo {
namespace drivers {
namespace camera_gst {

namespace {

constexpr const char* kStitchedTeeName = "stitched_tee";
constexpr const char* kStitchedPublishSinkName = "stitched_publish_sink";
constexpr GstClockTime kBusPollInterval = 100 * GST_MSECOND;
constexpr auto kRecoveryDelay = std::chrono::milliseconds(250);

GstPad* RequestPadCompat(GstElement* element, const char* name_template) {
#if GST_CHECK_VERSION(1, 20, 0)
  return gst_element_request_pad_simple(element, name_template);
#else
  return gst_element_get_request_pad(element, name_template);
#endif
}

void GstInitOnce() {
  static std::once_flag gst_init_once;
  std::call_once(gst_init_once, []() {
    int argc = 0;
    char** argv = nullptr;
    gst_init(&argc, &argv);
  });
}

double MeasurementTimeFromSample(GstSample* sample) {
  GstBuffer* buffer = gst_sample_get_buffer(sample);
  if (buffer != nullptr && GST_BUFFER_PTS_IS_VALID(buffer)) {
    return static_cast<double>(GST_BUFFER_PTS(buffer)) /
           static_cast<double>(GST_SECOND);
  }
  return apollo::cyber::Time::Now().ToSecond();
}

}  // namespace

CameraGstStreamer::~CameraGstStreamer() { Stop(); }

bool CameraGstStreamer::Start(const config::Config& config,
                              SourcePublishCallback source_publish_callback,
                              PublishCallback stitched_publish_callback,
                              GpuFrameCallback gpu_frame_callback) {
  GstInitOnce();
  std::lock_guard<std::mutex> lock(mutex_);
  if (running_) {
    return true;
  }

  config_ = config;
  source_publish_callback_ = std::move(source_publish_callback);
  stitched_publish_callback_ = std::move(stitched_publish_callback);
  gpu_frame_callback_ = std::move(gpu_frame_callback);
  stream_enabled_ = config_.stream().enable();
  stop_requested_ = false;
  stream_attached_ = false;
  output_width_ = 0;
  output_height_ = 0;
  pipeline_warning_count_ = 0;
  pipeline_error_count_ = 0;
  pipeline_restart_count_ = 0;
  state_ = RunState::kStarting;

  if (!ValidateConfigLocked()) {
    state_ = RunState::kFailed;
    return false;
  }
  if (!BuildPipelineLocked()) {
    state_ = RunState::kFailed;
    return false;
  }

  if (stream_enabled_ && config_.stream().auto_start() &&
      !StartStreamBranchLocked()) {
    AERROR << "camera_gst failed to auto-start stream branch.";
    ReleasePipelineLocked();
    state_ = RunState::kFailed;
    return false;
  }

  running_ = true;
  state_ = RunState::kRunning;
  bus_thread_ = std::thread(&CameraGstStreamer::BusLoop, this);
  return true;
}

void CameraGstStreamer::Stop() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_ && !bus_thread_.joinable()) {
      return;
    }
    stop_requested_ = true;
  }
  if (bus_thread_.joinable()) {
    bus_thread_.join();
  }
  std::lock_guard<std::mutex> lock(mutex_);
  ReleasePipelineLocked();
  running_ = false;
  state_ = RunState::kStopped;
}

bool CameraGstStreamer::StartStreaming() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!running_ || !stream_enabled_) {
    return false;
  }
  return StartStreamBranchLocked();
}

bool CameraGstStreamer::StopStreaming() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!stream_attached_) {
    return true;
  }
  StopStreamBranchDirectLocked();
  return !stream_attached_;
}

bool CameraGstStreamer::streaming_active() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return stream_attached_;
}

StreamStats CameraGstStreamer::stats() const {
  std::lock_guard<std::mutex> lock(mutex_);
  StreamStats snapshot;
  snapshot.pipeline_warning_count = pipeline_warning_count_;
  snapshot.pipeline_error_count = pipeline_error_count_;
  snapshot.pipeline_restart_count = pipeline_restart_count_;
  snapshot.stream_attached = stream_attached_;
  snapshot.source_stats.reserve(source_states_.size());
  for (const auto& source_state : source_states_) {
    SourceStats source;
    source.source_name = source_state->source_name;
    source.cpu_frames = source_state->cpu_frames.load();
    source.gpu_frames = source_state->gpu_frames.load();
    source.cpu_rate_limited_frames =
        source_state->cpu_rate_limited_frames.load();
    source.cpu_drop_frames = source_state->cpu_drop_frames.load();
    source.gpu_drop_frames = source_state->gpu_drop_frames.load();
    snapshot.source_stats.push_back(std::move(source));
  }
  return snapshot;
}

bool CameraGstStreamer::ValidateConfigLocked() {
  if (config_.sources_size() == 0) {
    AERROR << "camera_gst requires at least one source.";
    return false;
  }

  if (source_publish_callback_) {
    AERROR << "camera_gst source CPU publish callbacks are unsupported in "
           << "GPU-only mode.";
    return false;
  }
  if (stitched_publish_callback_) {
    AERROR << "camera_gst stitched CPU publish callback is unsupported in "
           << "GPU-only mode.";
    return false;
  }

  layout_slots_.clear();
  source_states_.clear();
  source_states_.reserve(static_cast<size_t>(config_.sources_size()));
  output_width_ = static_cast<int>(config_.cols() * config_.tile_width());
  output_height_ = static_cast<int>(config_.rows() * config_.tile_height());

  std::set<std::string> source_names;
  for (const auto& source_config : config_.sources()) {
    if (source_config.name().empty()) {
      AERROR << "camera_gst source name must not be empty.";
      return false;
    }
    if (!source_names.insert(source_config.name()).second) {
      AERROR << "Duplicate camera_gst source name: " << source_config.name();
      return false;
    }
    if (source_config.uri().empty() &&
        source_config.capture_pipeline().empty()) {
      AERROR << "camera_gst source " << source_config.name()
             << " requires a uri or capture_pipeline.";
      return false;
    }
    if (source_config.has_publish()) {
      AERROR << "camera_gst source " << source_config.name()
             << " declares CPU publish config, which is unsupported in "
             << "GPU-only mode.";
      return false;
    }
    source_states_.emplace_back(
        std::make_unique<SourceRuntimeState>(source_config.name()));
  }

  const bool stitched_consumer_enabled =
      static_cast<bool>(stitched_publish_callback_) || stream_enabled_;
  if (stitched_consumer_enabled) {
    if (config_.layout_slots_size() == 0) {
      AERROR << "camera_gst requires layout slots when stitched publish or "
             << "streaming is enabled.";
      return false;
    }
    if (config_.rows() == 0 || config_.cols() == 0 ||
        config_.tile_width() == 0 || config_.tile_height() == 0) {
      AERROR << "camera_gst rows/cols/tile sizes must be positive.";
      return false;
    }
  }

  std::set<std::string> slot_sources;
  std::set<std::pair<int, int>> occupied_cells;
  layout_slots_.reserve(static_cast<size_t>(config_.layout_slots_size()));
  for (int index = 0; index < config_.layout_slots_size(); ++index) {
    const auto& slot = config_.layout_slots(index);
    if (!source_names.count(slot.source_name())) {
      AERROR << "camera_gst layout references unknown source "
             << slot.source_name();
      return false;
    }
    if (!slot_sources.insert(slot.source_name()).second) {
      AERROR << "camera_gst layout contains a duplicate source "
             << slot.source_name();
      return false;
    }
    const int row = static_cast<int>(slot.row());
    const int col = static_cast<int>(slot.col());
    if (row < 0 || row >= static_cast<int>(config_.rows()) || col < 0 ||
        col >= static_cast<int>(config_.cols())) {
      AERROR << "camera_gst layout slot out of range for source "
             << slot.source_name();
      return false;
    }
    if (!occupied_cells.insert(std::make_pair(row, col)).second) {
      AERROR << "camera_gst layout reuses grid cell (" << row << ", " << col
             << ")";
      return false;
    }
    layout_slots_.push_back(PipelineLayoutSlot{
        slot.source_name(), static_cast<size_t>(index), row, col});
  }

  for (const auto& source_config : config_.sources()) {
    const bool publish_enabled = false;
    const bool stitch_selected =
        FindLayoutSlotLocked(source_config.name()) != nullptr;
    const bool gpu_publish_enabled =
        config_.publish_gpu_channel() && static_cast<bool>(gpu_frame_callback_);
    if (!publish_enabled && !stitch_selected && !gpu_publish_enabled) {
      AERROR << "camera_gst source " << source_config.name()
             << " is not connected to publish or stitch output.";
      return false;
    }
  }

  if (config_.has_platform()) {
    if (!config_.platform().target().empty() &&
        config_.platform().target() != "jetson-orin") {
      AWARN << "camera_gst platform target is " << config_.platform().target()
            << "; the GPU zero-copy reference path is tuned for jetson-orin.";
    }
    if (config_.platform().require_nvmm() && !config_.zero_copy_required()) {
      AWARN << "camera_gst platform.require_nvmm is set while "
            << "zero_copy_required is false; non-NVMM GPU samples will be "
            << "skipped instead of failing the graph.";
    }
  }

  if (stream_enabled_ && config_.stream().branch_pipeline().empty() &&
      config_.stream().pipeline_suffix().empty() &&
      config_.stream().host().empty()) {
    AERROR << "camera_gst stream.host must not be empty when using default "
           << "NVENC/RTP stream branch.";
    return false;
  }

  if (config_.stream().perception_transcode_before_publish() &&
      !config_.publish_gpu_channel()) {
    AERROR << "camera_gst perception_transcode_before_publish requires "
           << "publish_gpu_channel=true.";
    return false;
  }

  return true;
}

bool CameraGstStreamer::BuildPipelineLocked() {
  const auto builder = MakePipelineBuilderLocked();
  const bool validate_factories =
      config_.validate_nvidia_plugins() || config_.zero_copy_required() ||
      (config_.has_platform() && config_.platform().require_nvidia_plugins());
  if (validate_factories && !builder.ValidateRequiredFactories()) {
    AERROR << "camera_gst required NVIDIA/GStreamer plugins are unavailable.";
    return false;
  }

  const std::string description = builder.BuildPipelineDescription();
  if (description.empty()) {
    AERROR << "camera_gst built an empty pipeline description.";
    return false;
  }

  GError* error = nullptr;
  pipeline_ = gst_parse_launch(description.c_str(), &error);
  if (pipeline_ == nullptr) {
    const std::string message =
        error == nullptr ? "unknown parse error" : error->message;
    if (error != nullptr) {
      g_error_free(error);
    }
    AERROR << "camera_gst failed to parse capture graph: " << message;
    return false;
  }

  bus_ = gst_element_get_bus(pipeline_);
  stitched_tee_ = gst_bin_get_by_name(GST_BIN(pipeline_), kStitchedTeeName);
  if ((static_cast<bool>(stitched_publish_callback_) || stream_enabled_) &&
      stitched_tee_ == nullptr) {
    AERROR << "camera_gst failed to locate the stitched tee.";
    ReleasePipelineLocked();
    return false;
  }

  source_sink_contexts_.clear();
  source_sink_contexts_.reserve(static_cast<size_t>(config_.sources_size()));
  for (int index = 0; index < config_.sources_size(); ++index) {
    const auto& source_config = config_.sources(index);
    const bool publish_enabled =
        source_config.has_publish() &&
        !source_config.publish().channel_name().empty();
    if (!publish_enabled) {
      continue;
    }

    GstElement* appsink = gst_bin_get_by_name(
        GST_BIN(pipeline_),
        builder.SourcePublishSinkName(static_cast<size_t>(index)).c_str());
    if (appsink == nullptr) {
      AERROR << "camera_gst failed to locate appsink for source "
             << source_config.name();
      ReleasePipelineLocked();
      return false;
    }

    auto context = std::make_unique<SinkContext>();
    context->owner = this;
    context->source_name = source_config.name();
    context->source_state = source_states_[static_cast<size_t>(index)].get();
    if (source_config.publish().output_fps() > 0.0) {
      context->min_publish_interval_sec =
          1.0 / source_config.publish().output_fps();
    }
    GstAppSinkCallbacks callbacks = {};
    callbacks.new_sample = &CameraGstStreamer::OnSourceSample;
    g_object_set(appsink, "sync", FALSE, "max-buffers", 1u, "drop", TRUE,
                 nullptr);
    gst_app_sink_set_callbacks(GST_APP_SINK(appsink), &callbacks, context.get(),
                               nullptr);
    gst_object_unref(appsink);
    source_sink_contexts_.push_back(std::move(context));
  }

  gpu_sink_contexts_.clear();
  if (gpu_frame_callback_ && config_.publish_gpu_channel()) {
    gpu_sink_contexts_.reserve(static_cast<size_t>(config_.sources_size()));
    for (int index = 0; index < config_.sources_size(); ++index) {
      const auto& source_config = config_.sources(index);
      GstElement* appsink = gst_bin_get_by_name(
          GST_BIN(pipeline_),
          builder.SourceGpuSinkName(static_cast<size_t>(index)).c_str());
      if (appsink == nullptr) {
        AERROR << "camera_gst failed to locate GPU appsink for source "
               << source_config.name();
        ReleasePipelineLocked();
        return false;
      }

      auto context = std::make_unique<SinkContext>();
      context->owner = this;
      context->source_name = source_config.name();
      context->source_state = source_states_[static_cast<size_t>(index)].get();
      GstAppSinkCallbacks callbacks = {};
      callbacks.new_sample = &CameraGstStreamer::OnGpuSample;
      g_object_set(appsink, "sync", FALSE, "max-buffers", 1u, "drop", TRUE,
                   nullptr);
      gst_app_sink_set_callbacks(GST_APP_SINK(appsink), &callbacks,
                                 context.get(), nullptr);
      gst_object_unref(appsink);
      gpu_sink_contexts_.push_back(std::move(context));
    }
  }

  stitched_publish_sink_ = nullptr;
  if (stitched_publish_callback_) {
    stitched_publish_sink_ =
        gst_bin_get_by_name(GST_BIN(pipeline_), kStitchedPublishSinkName);
    if (stitched_publish_sink_ == nullptr) {
      AERROR << "camera_gst failed to locate the stitched publish appsink.";
      ReleasePipelineLocked();
      return false;
    }

    stitched_sink_context_ = std::make_unique<SinkContext>();
    stitched_sink_context_->owner = this;
    stitched_sink_context_->stitched = true;
    GstAppSinkCallbacks callbacks = {};
    callbacks.new_sample = &CameraGstStreamer::OnStitchedSample;
    g_object_set(stitched_publish_sink_, "sync", FALSE, "max-buffers", 1u,
                 "drop", TRUE, nullptr);
    gst_app_sink_set_callbacks(GST_APP_SINK(stitched_publish_sink_), &callbacks,
                               stitched_sink_context_.get(), nullptr);
  }

  if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) ==
      GST_STATE_CHANGE_FAILURE) {
    AERROR << "camera_gst failed to set the capture graph to PLAYING.";
    ReleasePipelineLocked();
    return false;
  }

  gst_element_get_state(pipeline_, nullptr, nullptr, GST_SECOND);
  return true;
}

bool CameraGstStreamer::RebuildPipelineLocked(const std::string& reason) {
  const bool restore_stream =
      stream_enabled_ && (stream_attached_ || config_.stream().auto_start());
  ++pipeline_restart_count_;
  AWARN << "camera_gst rebuilding capture graph after " << reason;

  ReleasePipelineLocked();
  if (!BuildPipelineLocked()) {
    state_ = RunState::kFailed;
    stop_requested_ = true;
    AERROR << "camera_gst failed to rebuild capture graph.";
    return false;
  }
  if (restore_stream && !StartStreamBranchLocked()) {
    ReleasePipelineLocked();
    state_ = RunState::kFailed;
    stop_requested_ = true;
    AERROR << "camera_gst failed to restore stream branch.";
    return false;
  }
  state_ = RunState::kRunning;
  return true;
}

bool CameraGstStreamer::StartStreamBranchLocked() {
  if (stream_attached_) {
    return true;
  }
  if (!stream_enabled_ || stitched_tee_ == nullptr) {
    return false;
  }

  const std::string description = StreamBranchDescriptionLocked();
  if (description.empty()) {
    AERROR << "camera_gst stream branch description is empty.";
    return false;
  }

  GError* error = nullptr;
  GstElement* branch_bin =
      gst_parse_bin_from_description(description.c_str(), TRUE, &error);
  if (branch_bin == nullptr) {
    const std::string message =
        error == nullptr ? "unknown error" : error->message;
    if (error != nullptr) {
      g_error_free(error);
    }
    AERROR << "camera_gst failed to parse stream branch: " << message;
    return false;
  }

  GstPad* branch_sink_pad = gst_element_get_static_pad(branch_bin, "sink");
  if (branch_sink_pad == nullptr) {
    AERROR << "camera_gst stream branch has no sink pad.";
    gst_object_unref(branch_bin);
    return false;
  }

  gst_bin_add(GST_BIN(pipeline_), branch_bin);
  if (!gst_element_sync_state_with_parent(branch_bin)) {
    AERROR << "camera_gst failed to sync stream branch with the graph.";
    gst_object_unref(branch_sink_pad);
    gst_bin_remove(GST_BIN(pipeline_), branch_bin);
    return false;
  }

  GstPad* tee_pad = RequestPadCompat(stitched_tee_, "src_%u");
  if (tee_pad == nullptr ||
      gst_pad_link(tee_pad, branch_sink_pad) != GST_PAD_LINK_OK) {
    if (tee_pad != nullptr) {
      gst_element_release_request_pad(stitched_tee_, tee_pad);
      gst_object_unref(tee_pad);
    }
    gst_object_unref(branch_sink_pad);
    gst_element_set_state(branch_bin, GST_STATE_NULL);
    gst_bin_remove(GST_BIN(pipeline_), branch_bin);
    AERROR << "camera_gst failed to connect the stitched tee to the stream "
           << "branch.";
    return false;
  }

  stream_branch_bin_ = branch_bin;
  stream_tee_pad_ = tee_pad;
  stream_sink_pad_ = branch_sink_pad;
  stream_attached_ = true;
  if (config_.stream().force_keyframe_on_attach()) {
    ForceKeyFrameLocked();
  }
  return true;
}

void CameraGstStreamer::StopStreamBranchDirectLocked() {
  if (!stream_attached_) {
    return;
  }

  GstElement* branch_bin = stream_branch_bin_;
  GstPad* tee_pad = stream_tee_pad_;
  GstPad* sink_pad = stream_sink_pad_;
  stream_branch_bin_ = nullptr;
  stream_tee_pad_ = nullptr;
  stream_sink_pad_ = nullptr;
  stream_attached_ = false;

  if (tee_pad != nullptr && sink_pad != nullptr) {
    gst_pad_unlink(tee_pad, sink_pad);
  }
  if (stitched_tee_ != nullptr && tee_pad != nullptr) {
    gst_element_release_request_pad(stitched_tee_, tee_pad);
  }
  if (tee_pad != nullptr) {
    gst_object_unref(tee_pad);
  }
  if (sink_pad != nullptr) {
    gst_object_unref(sink_pad);
  }
  if (branch_bin != nullptr) {
    if (config_.stream().emit_eos_on_detach()) {
      gst_element_send_event(branch_bin, gst_event_new_eos());
    }
    gst_element_set_state(branch_bin, GST_STATE_NULL);
    if (pipeline_ != nullptr) {
      gst_bin_remove(GST_BIN(pipeline_), branch_bin);
    }
  }
}

void CameraGstStreamer::ReleasePipelineLocked() {
  StopStreamBranchDirectLocked();
  if (stitched_publish_sink_ != nullptr) {
    GstAppSinkCallbacks callbacks = {};
    gst_app_sink_set_callbacks(GST_APP_SINK(stitched_publish_sink_), &callbacks,
                               nullptr, nullptr);
    gst_object_unref(stitched_publish_sink_);
    stitched_publish_sink_ = nullptr;
  }
  if (stitched_tee_ != nullptr) {
    gst_object_unref(stitched_tee_);
    stitched_tee_ = nullptr;
  }
  if (bus_ != nullptr) {
    gst_object_unref(bus_);
    bus_ = nullptr;
  }
  if (pipeline_ != nullptr) {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    gst_element_get_state(pipeline_, nullptr, nullptr, GST_CLOCK_TIME_NONE);
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
  }
  source_sink_contexts_.clear();
  gpu_sink_contexts_.clear();
  stitched_sink_context_.reset();
}

bool CameraGstStreamer::ForceKeyFrameLocked() {
  if (!stream_attached_ || stream_branch_bin_ == nullptr) {
    return false;
  }

  GstElement* target = stream_branch_bin_;
  if (!config_.stream().keyframe_element_name().empty()) {
    GstElement* named =
        gst_bin_get_by_name(GST_BIN(stream_branch_bin_),
                            config_.stream().keyframe_element_name().c_str());
    if (named != nullptr) {
      target = named;
    } else {
      AWARN << "camera_gst keyframe target not found: "
            << config_.stream().keyframe_element_name();
    }
  }

  GstStructure* structure = gst_structure_new("GstForceKeyUnit", "all-headers",
                                              G_TYPE_BOOLEAN, TRUE, nullptr);
  const bool sent = gst_element_send_event(
      target, gst_event_new_custom(GST_EVENT_CUSTOM_DOWNSTREAM, structure));
  if (target != stream_branch_bin_) {
    gst_object_unref(target);
  }
  return sent;
}

void CameraGstStreamer::BusLoop() {
  while (true) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stop_requested_) {
        break;
      }
      if (bus_ == nullptr) {
        return;
      }
      gst_object_ref(bus_);
    }

    GstMessage* message = gst_bus_timed_pop_filtered(
        bus_, kBusPollInterval,
        static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_WARNING |
                                    GST_MESSAGE_EOS));
    gst_object_unref(bus_);

    if (message == nullptr) {
      continue;
    }

    HandleBusMessage(message);
  }
}

void CameraGstStreamer::HandleBusMessage(GstMessage* message) {
  bool recover = false;
  std::string recovery_reason;

  switch (GST_MESSAGE_TYPE(message)) {
    case GST_MESSAGE_ERROR: {
      GError* error = nullptr;
      gchar* debug = nullptr;
      gst_message_parse_error(message, &error, &debug);
      {
        std::lock_guard<std::mutex> lock(mutex_);
        ++pipeline_error_count_;
      }
      recovery_reason =
          error == nullptr ? "unknown GStreamer error" : error->message;
      AERROR << "camera_gst GStreamer error: " << recovery_reason
             << (debug == nullptr ? "" : std::string(" debug: ") + debug);
      recover = true;
      if (error != nullptr) {
        g_error_free(error);
      }
      if (debug != nullptr) {
        g_free(debug);
      }
      break;
    }
    case GST_MESSAGE_WARNING: {
      GError* error = nullptr;
      gchar* debug = nullptr;
      gst_message_parse_warning(message, &error, &debug);
      {
        std::lock_guard<std::mutex> lock(mutex_);
        ++pipeline_warning_count_;
      }
      AWARN << "camera_gst GStreamer warning: "
            << (error == nullptr ? "unknown" : error->message)
            << (debug == nullptr ? "" : std::string(" debug: ") + debug);
      if (error != nullptr) {
        g_error_free(error);
      }
      if (debug != nullptr) {
        g_free(debug);
      }
      break;
    }
    case GST_MESSAGE_EOS:
      AWARN << "camera_gst capture graph reached EOS.";
      recovery_reason = "EOS";
      recover = true;
      break;
    default:
      break;
  }
  gst_message_unref(message);

  if (!recover) {
    return;
  }

  std::this_thread::sleep_for(kRecoveryDelay);
  std::lock_guard<std::mutex> lock(mutex_);
  if (stop_requested_ || !running_) {
    return;
  }
  state_ = RunState::kRecovering;
  RebuildPipelineLocked(recovery_reason);
}

std::string CameraGstStreamer::StreamBranchDescriptionLocked() const {
  if (!config_.stream().branch_pipeline().empty()) {
    return config_.stream().branch_pipeline();
  }
  if (!config_.stream().pipeline_suffix().empty()) {
    return config_.stream().pipeline_suffix();
  }
  return MakePipelineBuilderLocked().BuildDefaultStreamBranch();
}

const PipelineLayoutSlot* CameraGstStreamer::FindLayoutSlotLocked(
    const std::string& source_name) const {
  const auto iter =
      std::find_if(layout_slots_.begin(), layout_slots_.end(),
                   [&source_name](const PipelineLayoutSlot& slot) {
                     return slot.source_name == source_name;
                   });
  return iter == layout_slots_.end() ? nullptr : &(*iter);
}

CameraGstPipelineBuilder CameraGstStreamer::MakePipelineBuilderLocked() const {
  return CameraGstPipelineBuilder(
      config_, layout_slots_, false, false, stream_enabled_,
      config_.publish_gpu_channel() && static_cast<bool>(gpu_frame_callback_));
}

GstFlowReturn CameraGstStreamer::OnSourceSample(GstAppSink* appsink,
                                                gpointer user_data) {
  auto* context = static_cast<SinkContext*>(user_data);
  if (context == nullptr || context->owner == nullptr) {
    return GST_FLOW_ERROR;
  }

  GstSample* sample = gst_app_sink_pull_sample(appsink);
  if (sample == nullptr) {
    return GST_FLOW_EOS;
  }

  const double measurement_time = MeasurementTimeFromSample(sample);
  if (context->min_publish_interval_sec > 0.0 &&
      context->has_last_measurement_time &&
      measurement_time - context->last_measurement_time <
          context->min_publish_interval_sec) {
    if (context->source_state != nullptr) {
      ++context->source_state->cpu_rate_limited_frames;
    }
    gst_sample_unref(sample);
    return GST_FLOW_OK;
  }

  PublishedFrame frame = ExtractCpuFrame(sample);
  gst_sample_unref(sample);
  if (frame.data.empty()) {
    if (context->source_state != nullptr) {
      ++context->source_state->cpu_drop_frames;
    }
    return GST_FLOW_ERROR;
  }
  if (context->source_state != nullptr) {
    frame.sequence = ++context->source_state->cpu_frames;
  }
  context->last_measurement_time = measurement_time;
  context->has_last_measurement_time = true;
  if (context->owner->source_publish_callback_) {
    context->owner->source_publish_callback_(context->source_name,
                                             std::move(frame));
  }
  return GST_FLOW_OK;
}

GstFlowReturn CameraGstStreamer::OnStitchedSample(GstAppSink* appsink,
                                                  gpointer user_data) {
  auto* context = static_cast<SinkContext*>(user_data);
  if (context == nullptr || context->owner == nullptr) {
    return GST_FLOW_ERROR;
  }

  GstSample* sample = gst_app_sink_pull_sample(appsink);
  if (sample == nullptr) {
    return GST_FLOW_EOS;
  }

  PublishedFrame frame = ExtractCpuFrame(sample);
  gst_sample_unref(sample);
  if (frame.data.empty()) {
    return GST_FLOW_ERROR;
  }
  if (context->owner->stitched_publish_callback_) {
    context->owner->stitched_publish_callback_(std::move(frame));
  }
  return GST_FLOW_OK;
}

GstFlowReturn CameraGstStreamer::OnGpuSample(GstAppSink* appsink,
                                             gpointer user_data) {
  auto* context = static_cast<SinkContext*>(user_data);
  if (context == nullptr || context->owner == nullptr) {
    return GST_FLOW_ERROR;
  }

  GstSample* sample = gst_app_sink_pull_sample(appsink);
  if (sample == nullptr) {
    return GST_FLOW_EOS;
  }

  GpuFrame frame = ExtractNvmmFrame(sample, context->source_name);
  gst_sample_unref(sample);
  if (frame.empty()) {
    if (context->owner->config_.zero_copy_required()) {
      AERROR << "camera_gst zero-copy GPU frame extraction failed for source "
             << context->source_name;
      if (context->source_state != nullptr) {
        ++context->source_state->gpu_drop_frames;
      }
      return GST_FLOW_ERROR;
    }
    AWARN_EVERY(100) << "camera_gst skipped non-NVMM GPU sample for source "
                     << context->source_name;
    return GST_FLOW_OK;
  }
  if (context->source_state != nullptr) {
    ++context->source_state->gpu_frames;
  }
  if (context->owner->gpu_frame_callback_) {
    context->owner->gpu_frame_callback_(std::move(frame));
  }
  return GST_FLOW_OK;
}
}  // namespace camera_gst
}  // namespace drivers
}  // namespace apollo
