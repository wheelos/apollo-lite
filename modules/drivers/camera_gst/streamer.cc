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
#include <cmath>
#include <cstring>
#include <utility>

#include "cyber/cyber.h"

namespace apollo {
namespace drivers {
namespace camera_gst {

namespace {

constexpr int kQueueLeakyDownstream = 2;
constexpr const char* kInputFormat = "BGR";
constexpr const char* kPublishFormat = "RGB";

void GstInitOnce() {
  static std::once_flag gst_init_once;
  std::call_once(gst_init_once, []() {
    int argc = 0;
    char** argv = nullptr;
    gst_init(&argc, &argv);
  });
}

}  // namespace

CameraGstStreamer::~CameraGstStreamer() { Stop(); }

bool CameraGstStreamer::Start(int width, int height, double fps,
                              const config::StreamConfig& config,
                              PublishCallback publish_callback) {
  GstInitOnce();
  std::lock_guard<std::mutex> lock(mutex_);
  if (running_) {
    return true;
  }

  width_ = width;
  height_ = height;
  fps_ = fps;
  config_ = config;
  publish_callback_ = std::move(publish_callback);
  max_pending_frames_ = std::max<size_t>(1, config_.has_ingest_queue_size()
                                                ? config_.ingest_queue_size()
                                                : config_.queue_size());
  pending_frames_.clear();
  stop_requested_ = false;
  if (!BuildPipelineLocked()) {
    return false;
  }

  running_ = true;
  worker_ = std::thread(&CameraGstStreamer::FeedLoop, this);
  if (config_.enable() && config_.auto_start() && !StartStreamBranchLocked()) {
    AERROR << "camera_gst failed to auto-start stream branch.";
    ReleasePipelineLocked();
    stop_requested_ = true;
    running_ = false;
    worker_.join();
    return false;
  }
  return true;
}

void CameraGstStreamer::Stop() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_ && !worker_.joinable()) {
      return;
    }
    stop_requested_ = true;
  }
  condition_.notify_all();
  if (worker_.joinable()) {
    worker_.join();
  }
  std::lock_guard<std::mutex> lock(mutex_);
  pending_frames_.clear();
  ReleasePipelineLocked();
  running_ = false;
}

bool CameraGstStreamer::Submit(const cv::Mat& frame_bgr, double measurement_time) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!running_ || stop_requested_ || frame_bgr.empty()) {
    return false;
  }

  pending_frames_.push_back(PendingFrame{frame_bgr.clone(), measurement_time});
  while (pending_frames_.size() > max_pending_frames_) {
    pending_frames_.pop_front();
  }
  condition_.notify_one();
  return true;
}

bool CameraGstStreamer::StartStreaming() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!running_ || !config_.enable()) {
    return false;
  }
  return StartStreamBranchLocked();
}

bool CameraGstStreamer::StopStreaming() {
  GstPad* tee_pad = nullptr;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!stream_attached_) {
      return true;
    }
    if (stream_detach_requested_) {
      return false;
    }
    stream_detach_requested_ = true;
    tee_pad = stream_tee_pad_;
    gst_object_ref(tee_pad);
  }

  gst_pad_add_probe(tee_pad, GST_PAD_PROBE_TYPE_IDLE,
                    &CameraGstStreamer::OnStreamPadIdle, this, nullptr);
  gst_object_unref(tee_pad);

  std::unique_lock<std::mutex> lock(mutex_);
  condition_.wait_for(lock, std::chrono::seconds(2), [this]() {
    return !stream_attached_ && !stream_detach_requested_;
  });
  return !stream_attached_;
}

bool CameraGstStreamer::streaming_active() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return stream_attached_;
}

bool CameraGstStreamer::BuildPipelineLocked() {
  pipeline_ = gst_pipeline_new("camera_gst_pipeline");
  appsrc_ = gst_element_factory_make("appsrc", "stitched_src");
  tee_ = gst_element_factory_make("tee", "main_tee");
  publish_queue_ = gst_element_factory_make("queue", "publish_queue");
  publish_convert_ = gst_element_factory_make("videoconvert", "publish_convert");
  publish_capsfilter_ =
      gst_element_factory_make("capsfilter", "publish_capsfilter");
  appsink_ = gst_element_factory_make("appsink", "publish_sink");
  if (pipeline_ == nullptr || appsrc_ == nullptr || tee_ == nullptr ||
      publish_queue_ == nullptr || publish_convert_ == nullptr ||
      publish_capsfilter_ == nullptr ||
      appsink_ == nullptr) {
    AERROR << "camera_gst failed to create one or more GStreamer elements.";
    ReleasePipelineLocked();
    return false;
  }

  const int fps_num = std::max(1, static_cast<int>(std::lround(fps_)));
  GstCaps* src_caps =
      gst_caps_new_simple("video/x-raw", "format", G_TYPE_STRING, kInputFormat,
                          "width", G_TYPE_INT, width_, "height", G_TYPE_INT,
                          height_, "framerate", GST_TYPE_FRACTION, fps_num, 1,
                          nullptr);
  GstCaps* publish_caps = gst_caps_new_simple(
      "video/x-raw", "format", G_TYPE_STRING, kPublishFormat, nullptr);
  g_object_set(appsrc_, "caps", src_caps, "is-live", TRUE, "format",
               GST_FORMAT_TIME, "block", FALSE, nullptr);
  g_object_set(tee_, "allow-not-linked", TRUE, nullptr);
  g_object_set(publish_queue_, "max-size-buffers",
               std::max(1u, config_.publish_queue_size()), "leaky",
               kQueueLeakyDownstream, nullptr);
  g_object_set(publish_capsfilter_, "caps", publish_caps, nullptr);
  g_object_set(appsink_, "sync", FALSE, "max-buffers", 1u, "drop", TRUE,
               nullptr);
  GstAppSinkCallbacks callbacks = {};
  callbacks.new_sample = &CameraGstStreamer::OnNewSample;
  gst_app_sink_set_callbacks(GST_APP_SINK(appsink_), &callbacks, this, nullptr);
  gst_caps_unref(src_caps);
  gst_caps_unref(publish_caps);

  gst_bin_add_many(GST_BIN(pipeline_), appsrc_, tee_, publish_queue_,
                   publish_convert_, publish_capsfilter_, appsink_, nullptr);
  if (!gst_element_link(appsrc_, tee_) ||
      !gst_element_link_many(publish_queue_, publish_convert_,
                             publish_capsfilter_, appsink_, nullptr)) {
    AERROR << "camera_gst failed to link static GStreamer elements.";
    ReleasePipelineLocked();
    return false;
  }

  publish_tee_pad_ = gst_element_request_pad_simple(tee_, "src_%u");
  GstPad* publish_sink_pad = gst_element_get_static_pad(publish_queue_, "sink");
  if (publish_tee_pad_ == nullptr || publish_sink_pad == nullptr ||
      gst_pad_link(publish_tee_pad_, publish_sink_pad) != GST_PAD_LINK_OK) {
    if (publish_sink_pad != nullptr) {
      gst_object_unref(publish_sink_pad);
    }
    AERROR << "camera_gst failed to connect tee to publish branch.";
    ReleasePipelineLocked();
    return false;
  }
  gst_object_unref(publish_sink_pad);

  if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) ==
      GST_STATE_CHANGE_FAILURE) {
    AERROR << "camera_gst failed to set pipeline to PLAYING.";
    ReleasePipelineLocked();
    return false;
  }
  return true;
}

bool CameraGstStreamer::StartStreamBranchLocked() {
  if (stream_attached_) {
    return true;
  }

  const std::string description = StreamBranchDescription();
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
    AERROR << "camera_gst stream branch has no ghost sink pad.";
    gst_object_unref(branch_bin);
    return false;
  }

  gst_bin_add(GST_BIN(pipeline_), branch_bin);
  if (!gst_element_sync_state_with_parent(branch_bin)) {
    AERROR << "camera_gst failed to sync stream branch with pipeline.";
    gst_object_unref(branch_sink_pad);
    gst_bin_remove(GST_BIN(pipeline_), branch_bin);
    return false;
  }

  GstPad* tee_pad = gst_element_request_pad_simple(tee_, "src_%u");
  if (tee_pad == nullptr ||
      gst_pad_link(tee_pad, branch_sink_pad) != GST_PAD_LINK_OK) {
    if (tee_pad != nullptr) {
      gst_element_release_request_pad(tee_, tee_pad);
      gst_object_unref(tee_pad);
    }
    gst_object_unref(branch_sink_pad);
    gst_element_set_state(branch_bin, GST_STATE_NULL);
    gst_bin_remove(GST_BIN(pipeline_), branch_bin);
    AERROR << "camera_gst failed to link tee to stream branch.";
    return false;
  }

  stream_branch_bin_ = branch_bin;
  stream_tee_pad_ = tee_pad;
  stream_sink_pad_ = branch_sink_pad;
  stream_attached_ = true;
  stream_detach_requested_ = false;
  if (config_.force_keyframe_on_attach()) {
    ForceKeyFrameLocked();
  }
  return true;
}

void CameraGstStreamer::StopStreamBranchDirectLocked() {
  if (!stream_attached_) {
    stream_detach_requested_ = false;
    return;
  }

  GstElement* branch_bin = stream_branch_bin_;
  GstPad* tee_pad = stream_tee_pad_;
  GstPad* sink_pad = stream_sink_pad_;
  stream_branch_bin_ = nullptr;
  stream_tee_pad_ = nullptr;
  stream_sink_pad_ = nullptr;
  stream_attached_ = false;
  stream_detach_requested_ = false;

  if (tee_pad != nullptr && sink_pad != nullptr) {
    gst_pad_unlink(tee_pad, sink_pad);
  }
  if (tee_ != nullptr && tee_pad != nullptr) {
    gst_element_release_request_pad(tee_, tee_pad);
  }
  if (tee_pad != nullptr) {
    gst_object_unref(tee_pad);
  }
  if (sink_pad != nullptr) {
    gst_object_unref(sink_pad);
  }
  if (branch_bin != nullptr) {
    if (config_.emit_eos_on_detach()) {
      gst_element_send_event(branch_bin, gst_event_new_eos());
    }
    gst_element_set_state(branch_bin, GST_STATE_NULL);
    if (pipeline_ != nullptr) {
      gst_bin_remove(GST_BIN(pipeline_), branch_bin);
    }
  }
}

void CameraGstStreamer::ReleasePipelineLocked() {
  if (appsink_ != nullptr) {
    GstAppSinkCallbacks callbacks = {};
    gst_app_sink_set_callbacks(GST_APP_SINK(appsink_), &callbacks, nullptr,
                               nullptr);
  }
  StopStreamBranchDirectLocked();
  if (tee_ != nullptr && publish_tee_pad_ != nullptr) {
    gst_element_release_request_pad(tee_, publish_tee_pad_);
    gst_object_unref(publish_tee_pad_);
    publish_tee_pad_ = nullptr;
  }
  if (pipeline_ != nullptr) {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    gst_object_unref(pipeline_);
  }
  pipeline_ = nullptr;
  appsrc_ = nullptr;
  tee_ = nullptr;
  publish_queue_ = nullptr;
  publish_convert_ = nullptr;
  publish_capsfilter_ = nullptr;
  appsink_ = nullptr;
}

bool CameraGstStreamer::PushFrameToPipeline(const PendingFrame& frame) {
  GstElement* appsrc = nullptr;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_ || stop_requested_ || appsrc_ == nullptr) {
      return false;
    }
    appsrc = appsrc_;
    gst_object_ref(appsrc);
  }

  cv::Mat contiguous = frame.image_bgr.isContinuous() ? frame.image_bgr
                                                      : frame.image_bgr.clone();
  const size_t size = contiguous.total() * contiguous.elemSize();
  GstBuffer* buffer = gst_buffer_new_allocate(nullptr, size, nullptr);
  gst_buffer_fill(buffer, 0, contiguous.data, size);

  const GstClockTime pts = static_cast<GstClockTime>(
      std::max(0.0, frame.measurement_time) * GST_SECOND);
  GST_BUFFER_PTS(buffer) = pts;
  GST_BUFFER_DTS(buffer) = pts;
  GST_BUFFER_DURATION(buffer) =
      fps_ > 0.0 ? static_cast<GstClockTime>(GST_SECOND / fps_)
                 : GST_CLOCK_TIME_NONE;
  const GstFlowReturn flow =
      gst_app_src_push_buffer(GST_APP_SRC(appsrc), buffer);
  gst_object_unref(appsrc);
  return flow == GST_FLOW_OK;
}

bool CameraGstStreamer::ForceKeyFrameLocked() {
  if (!stream_attached_ || stream_branch_bin_ == nullptr) {
    return false;
  }

  GstElement* target = stream_branch_bin_;
  if (!config_.keyframe_element_name().empty()) {
    GstElement* named =
        gst_bin_get_by_name(GST_BIN(stream_branch_bin_),
                            config_.keyframe_element_name().c_str());
    if (named != nullptr) {
      target = named;
    } else {
      AWARN << "camera_gst keyframe target not found: "
            << config_.keyframe_element_name();
    }
  }

  GstStructure* structure =
      gst_structure_new("GstForceKeyUnit", "all-headers", G_TYPE_BOOLEAN, TRUE,
                        nullptr);
  const bool sent = gst_element_send_event(
      target, gst_event_new_custom(GST_EVENT_CUSTOM_DOWNSTREAM, structure));
  if (target != stream_branch_bin_) {
    gst_object_unref(target);
  }
  return sent;
}

std::string CameraGstStreamer::StreamBranchDescription() const {
  if (!config_.branch_pipeline().empty()) {
    return config_.branch_pipeline();
  }
  return config_.pipeline_suffix();
}

void CameraGstStreamer::FeedLoop() {
  while (true) {
    PendingFrame frame;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      condition_.wait(
          lock, [this]() { return stop_requested_ || !pending_frames_.empty(); });
      if (stop_requested_ && pending_frames_.empty()) {
        break;
      }
      frame = pending_frames_.front();
      pending_frames_.pop_front();
    }

    if (!PushFrameToPipeline(frame)) {
      AWARN_EVERY(100) << "camera_gst dropped a frame before appsrc push.";
    }
  }
}

GstFlowReturn CameraGstStreamer::OnNewSample(GstAppSink* appsink,
                                             gpointer user_data) {
  auto* self = static_cast<CameraGstStreamer*>(user_data);
  if (self == nullptr) {
    return GST_FLOW_ERROR;
  }

  GstSample* sample = gst_app_sink_pull_sample(appsink);
  if (sample == nullptr) {
    return GST_FLOW_EOS;
  }

  GstBuffer* buffer = gst_sample_get_buffer(sample);
  GstCaps* caps = gst_sample_get_caps(sample);
  if (buffer == nullptr || caps == nullptr) {
    gst_sample_unref(sample);
    return GST_FLOW_ERROR;
  }

  GstStructure* structure = gst_caps_get_structure(caps, 0);
  int width = 0;
  int height = 0;
  gst_structure_get_int(structure, "width", &width);
  gst_structure_get_int(structure, "height", &height);

  PublishedFrame frame;
  frame.image_rgb = cv::Mat(height, width, CV_8UC3);
  const size_t bytes = frame.image_rgb.total() * frame.image_rgb.elemSize();
  const gsize copied = gst_buffer_extract(buffer, 0, frame.image_rgb.data, bytes);
  if (copied != bytes) {
    frame.image_rgb.setTo(cv::Scalar::all(0));
  }
  frame.measurement_time =
      GST_BUFFER_PTS_IS_VALID(buffer)
          ? static_cast<double>(GST_BUFFER_PTS(buffer)) / GST_SECOND
          : apollo::cyber::Time::Now().ToSecond();
  gst_sample_unref(sample);

  if (self->publish_callback_) {
    self->publish_callback_(std::move(frame));
  }
  return GST_FLOW_OK;
}

GstPadProbeReturn CameraGstStreamer::OnStreamPadIdle(GstPad* pad,
                                                     GstPadProbeInfo* info,
                                                     gpointer user_data) {
  auto* self = static_cast<CameraGstStreamer*>(user_data);
  if (self != nullptr) {
    self->CompleteIdleDetach(pad);
  }
  return GST_PAD_PROBE_REMOVE;
}

void CameraGstStreamer::CompleteIdleDetach(GstPad* tee_pad) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (tee_pad != stream_tee_pad_) {
    stream_detach_requested_ = false;
    condition_.notify_all();
    return;
  }
  StopStreamBranchDirectLocked();
  condition_.notify_all();
}

}  // namespace camera_gst
}  // namespace drivers
}  // namespace apollo
