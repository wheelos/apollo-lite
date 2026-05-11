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
#include <cmath>
#include <cctype>
#include <set>
#include <sstream>
#include <utility>

#include "cyber/cyber.h"
#include "cyber/time/time.h"

namespace apollo {
namespace drivers {
namespace camera_gst {

namespace {

constexpr const char* kSourceSinkPrefix = "source_publish_sink_";
constexpr const char* kSourceTeePrefix = "source_tee_";
constexpr const char* kStitchedTeeName = "stitched_tee";
constexpr const char* kStitchedPublishSinkName = "stitched_publish_sink";
constexpr const char* kGpuPixelFormat = "NV12";
constexpr const char* kPublishPixelFormat = "RGB";
constexpr GstClockTime kBusPollInterval = 100 * GST_MSECOND;

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

bool IsDevicePath(const std::string& uri) {
  return uri.rfind("/dev/", 0) == 0;
}

bool IsNumericIndex(const std::string& uri) {
  return !uri.empty() &&
         std::all_of(uri.begin(), uri.end(),
                     [](unsigned char c) { return std::isdigit(c) != 0; });
}

bool IsArgusUri(const std::string& uri) {
  return uri.rfind("csi://", 0) == 0 || uri.rfind("argus://", 0) == 0;
}

bool ParseArgusSensorId(const std::string& uri, int* sensor_id) {
  if (sensor_id == nullptr || !IsArgusUri(uri)) {
    return false;
  }
  const size_t separator = uri.find("://");
  if (separator == std::string::npos || separator + 3 >= uri.size()) {
    return false;
  }
  try {
    *sensor_id = std::stoi(uri.substr(separator + 3));
  } catch (const std::exception&) {
    return false;
  }
  return true;
}

std::string ToUpperCopy(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) {
                   return static_cast<char>(std::toupper(c));
                 });
  return value;
}

std::string BuildFramerate(double fps) {
  const int fps_num = std::max(1, static_cast<int>(std::lround(fps)));
  return std::to_string(fps_num) + "/1";
}

std::string QuoteForGst(const std::string& value) {
  std::string quoted = "\"";
  quoted.reserve(value.size() + 2);
  for (char c : value) {
    if (c == '\\' || c == '\"') {
      quoted.push_back('\\');
    }
    quoted.push_back(c);
  }
  quoted.push_back('\"');
  return quoted;
}

std::string GstRawFormatForFourcc(const std::string& fourcc) {
  const std::string upper = ToUpperCopy(fourcc);
  if (upper == "YUYV" || upper == "YUY2") {
    return "YUY2";
  }
  if (upper == "UYVY") {
    return "UYVY";
  }
  if (upper == "YVYU") {
    return "YVYU";
  }
  if (upper == "NV12") {
    return "NV12";
  }
  return "";
}

}  // namespace

CameraGstStreamer::~CameraGstStreamer() { Stop(); }

bool CameraGstStreamer::Start(const config::Config& config,
                              SourcePublishCallback source_publish_callback,
                              PublishCallback stitched_publish_callback) {
  GstInitOnce();
  std::lock_guard<std::mutex> lock(mutex_);
  if (running_) {
    return true;
  }

  config_ = config;
  source_publish_callback_ = std::move(source_publish_callback);
  stitched_publish_callback_ = std::move(stitched_publish_callback);
  stream_enabled_ = config_.stream().enable();
  stop_requested_ = false;
  stream_attached_ = false;
  output_width_ = 0;
  output_height_ = 0;

  if (!ValidateConfigLocked()) {
    return false;
  }
  if (!BuildPipelineLocked()) {
    return false;
  }

  if (stream_enabled_ && config_.stream().auto_start() &&
      !StartStreamBranchLocked()) {
    AERROR << "camera_gst failed to auto-start stream branch.";
    ReleasePipelineLocked();
    return false;
  }

  running_ = true;
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

bool CameraGstStreamer::ValidateConfigLocked() {
  if (config_.sources_size() == 0) {
    AERROR << "camera_gst requires at least one source.";
    return false;
  }

  layout_slots_.clear();
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
    layout_slots_.push_back(LayoutSlot{slot.source_name(),
                                       static_cast<size_t>(index), row, col});
  }

  for (const auto& source_config : config_.sources()) {
    const bool publish_enabled = source_config.has_publish() &&
                                 !source_config.publish().channel_name().empty();
    const bool stitch_selected = FindLayoutSlotLocked(source_config.name()) != nullptr;
    if (!publish_enabled && !stitch_selected) {
      AERROR << "camera_gst source " << source_config.name()
             << " is not connected to publish or stitch output.";
      return false;
    }
  }

  return true;
}

bool CameraGstStreamer::BuildPipelineLocked() {
  const std::string description = BuildPipelineDescriptionLocked();
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
    const bool publish_enabled = source_config.has_publish() &&
                                 !source_config.publish().channel_name().empty();
    if (!publish_enabled) {
      continue;
    }

    GstElement* appsink = gst_bin_get_by_name(
        GST_BIN(pipeline_), SourcePublishSinkName(static_cast<size_t>(index)).c_str());
    if (appsink == nullptr) {
      AERROR << "camera_gst failed to locate appsink for source "
             << source_config.name();
      ReleasePipelineLocked();
      return false;
    }

    auto context = std::make_unique<SinkContext>();
    context->owner = this;
    context->source_name = source_config.name();
    GstAppSinkCallbacks callbacks = {};
    callbacks.new_sample = &CameraGstStreamer::OnSourceSample;
    g_object_set(appsink, "sync", FALSE, "max-buffers", 1u, "drop", TRUE,
                 nullptr);
    gst_app_sink_set_callbacks(GST_APP_SINK(appsink), &callbacks,
                               context.get(), nullptr);
    gst_object_unref(appsink);
    source_sink_contexts_.push_back(std::move(context));
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
  source_sink_contexts_.clear();
  stitched_sink_context_.reset();
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
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
  }
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

    switch (GST_MESSAGE_TYPE(message)) {
      case GST_MESSAGE_ERROR: {
        GError* error = nullptr;
        gchar* debug = nullptr;
        gst_message_parse_error(message, &error, &debug);
        AERROR << "camera_gst GStreamer error: "
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
      case GST_MESSAGE_WARNING: {
        GError* error = nullptr;
        gchar* debug = nullptr;
        gst_message_parse_warning(message, &error, &debug);
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
        break;
      default:
        break;
    }
    gst_message_unref(message);
  }
}

CameraGstStreamer::PublishedFrame CameraGstStreamer::ExtractFrame(
    GstSample* sample) const {
  PublishedFrame frame;
  if (sample == nullptr) {
    return frame;
  }

  GstBuffer* buffer = gst_sample_get_buffer(sample);
  GstCaps* caps = gst_sample_get_caps(sample);
  if (buffer == nullptr || caps == nullptr) {
    return frame;
  }

  GstStructure* structure = gst_caps_get_structure(caps, 0);
  int width = 0;
  int height = 0;
  if (!gst_structure_get_int(structure, "width", &width) ||
      !gst_structure_get_int(structure, "height", &height) || width <= 0 ||
      height <= 0) {
    return frame;
  }

  GstMapInfo map_info;
  if (!gst_buffer_map(buffer, &map_info, GST_MAP_READ)) {
    return frame;
  }

  if (map_info.size == 0 || map_info.size % static_cast<size_t>(height) != 0) {
    gst_buffer_unmap(buffer, &map_info);
    return frame;
  }

  frame.width = static_cast<uint32_t>(width);
  frame.height = static_cast<uint32_t>(height);
  frame.step = static_cast<uint32_t>(map_info.size / static_cast<size_t>(height));
  frame.measurement_time =
      GST_BUFFER_PTS_IS_VALID(buffer)
          ? static_cast<double>(GST_BUFFER_PTS(buffer)) / GST_SECOND
          : apollo::cyber::Time::Now().ToSecond();
  frame.data.assign(reinterpret_cast<const char*>(map_info.data), map_info.size);

  gst_buffer_unmap(buffer, &map_info);
  return frame;
}

std::string CameraGstStreamer::StreamBranchDescriptionLocked() const {
  if (!config_.stream().branch_pipeline().empty()) {
    return config_.stream().branch_pipeline();
  }
  return config_.stream().pipeline_suffix();
}

std::string CameraGstStreamer::BuildPipelineDescriptionLocked() const {
  std::ostringstream pipeline;
  const bool stitched_consumer_enabled =
      static_cast<bool>(stitched_publish_callback_) || stream_enabled_;
  if (stitched_consumer_enabled) {
    pipeline << BuildCompositorDescriptionLocked();
  }

  for (int index = 0; index < config_.sources_size(); ++index) {
    const auto& source_config = config_.sources(index);
    const std::string source_description =
        BuildSourceDescriptionLocked(static_cast<size_t>(index), source_config,
                                     FindLayoutSlotLocked(source_config.name()));
    if (source_description.empty()) {
      AERROR << "camera_gst could not build a GPU source branch for "
             << source_config.name() << " uri: " << source_config.uri();
      return "";
    }
    pipeline << source_description << ' ';
  }

  return pipeline.str();
}

std::string CameraGstStreamer::BuildCompositorDescriptionLocked() const {
  std::ostringstream compositor;
  compositor << "nvcompositor name=comp ";
  for (const auto& slot : layout_slots_) {
    compositor << "sink_" << slot.pad_index << "::xpos="
               << slot.col * static_cast<int>(config_.tile_width()) << ' '
               << "sink_" << slot.pad_index << "::ypos="
               << slot.row * static_cast<int>(config_.tile_height()) << ' '
               << "sink_" << slot.pad_index << "::width="
               << config_.tile_width() << ' '
               << "sink_" << slot.pad_index << "::height="
               << config_.tile_height() << ' ';
  }

  compositor << "! queue leaky=downstream max-size-buffers=2 ! nvvidconv ! "
             << "video/x-raw(memory:NVMM),format=(string)" << kGpuPixelFormat
             << ",width=(int)" << output_width_ << ",height=(int)"
             << output_height_ << ",framerate=(fraction)"
             << BuildFramerate(config_.fps()) << " ! tee name="
             << kStitchedTeeName << ' ';

  if (stitched_publish_callback_) {
    compositor << kStitchedTeeName
               << ". ! queue leaky=downstream max-size-buffers=1 ! nvvidconv ! "
               << "video/x-raw,format=(string)" << kPublishPixelFormat
               << " ! appsink name=" << kStitchedPublishSinkName
               << " sync=false max-buffers=1 drop=true ";
  }

  return compositor.str();
}

std::string CameraGstStreamer::BuildSourceDescriptionLocked(
    size_t source_index, const config::CameraSourceConfig& source_config,
    const LayoutSlot* layout_slot) const {
  const bool publish_enabled = source_config.has_publish() &&
                               !source_config.publish().channel_name().empty();
  std::ostringstream source;
  source << [&]() {
    if (!source_config.capture_pipeline().empty()) {
      return source_config.capture_pipeline();
    }

    const std::string framerate = BuildFramerate(source_config.fps());
    if (IsArgusUri(source_config.uri())) {
      int sensor_id = 0;
      if (!ParseArgusSensorId(source_config.uri(), &sensor_id)) {
        return std::string();
      }
      return "nvarguscamerasrc sensor-id=" + std::to_string(sensor_id) +
             " do-timestamp=true ! video/x-raw(memory:NVMM),width=(int)" +
             std::to_string(source_config.width()) + ",height=(int)" +
             std::to_string(source_config.height()) +
             ",format=(string)NV12,framerate=(fraction)" + framerate;
    }

    std::string device_path;
    if (IsNumericIndex(source_config.uri())) {
      device_path = "/dev/video" + source_config.uri();
    } else if (IsDevicePath(source_config.uri())) {
      device_path = source_config.uri();
    }
    if (device_path.empty()) {
      return std::string();
    }

    const std::string caps_common =
        "width=(int)" + std::to_string(source_config.width()) +
        ",height=(int)" + std::to_string(source_config.height()) +
        ",framerate=(fraction)" + framerate;
    const std::string upper_fourcc = ToUpperCopy(source_config.fourcc());
    if (upper_fourcc == "MJPG" || upper_fourcc == "JPEG") {
      return "v4l2src device=" + QuoteForGst(device_path) +
             " io-mode=4 do-timestamp=true ! image/jpeg," + caps_common +
             " ! jpegparse ! nvv4l2decoder mjpeg=1";
    }

    const std::string raw_format = GstRawFormatForFourcc(source_config.fourcc());
    if (raw_format.empty()) {
      return std::string();
    }
    return "v4l2src device=" + QuoteForGst(device_path) +
           " io-mode=4 do-timestamp=true ! video/x-raw,format=(string)" +
           raw_format + "," + caps_common;
  }();

  const std::string source_head = source.str();
  if (source_head.empty()) {
    return std::string();
  }

  const std::string tee_name = SourceTeeName(source_index);
  std::ostringstream branch;
  branch << source_head << " ! queue leaky=downstream max-size-buffers=2 ! "
         << "nvvidconv ! video/x-raw(memory:NVMM),format=(string)"
         << kGpuPixelFormat << ",width=(int)" << source_config.width()
         << ",height=(int)" << source_config.height()
         << ",framerate=(fraction)" << BuildFramerate(source_config.fps())
         << " ! tee name=" << tee_name << ' ';

  if (publish_enabled) {
    branch << tee_name
           << ". ! queue leaky=downstream max-size-buffers=1 ! nvvidconv ! "
           << "video/x-raw,format=(string)" << kPublishPixelFormat
           << " ! appsink name=" << SourcePublishSinkName(source_index)
           << " sync=false max-buffers=1 drop=true ";
  }
  if (layout_slot != nullptr) {
    branch << tee_name << ". ! queue leaky=downstream max-size-buffers=2 ! "
           << "comp.sink_" << layout_slot->pad_index << ' ';
  }

  return branch.str();
}

const CameraGstStreamer::LayoutSlot* CameraGstStreamer::FindLayoutSlotLocked(
    const std::string& source_name) const {
  const auto iter =
      std::find_if(layout_slots_.begin(), layout_slots_.end(),
                   [&source_name](const LayoutSlot& slot) {
                     return slot.source_name == source_name;
                   });
  return iter == layout_slots_.end() ? nullptr : &(*iter);
}

std::string CameraGstStreamer::SourceTeeName(size_t source_index) const {
  return std::string(kSourceTeePrefix) + std::to_string(source_index);
}

std::string CameraGstStreamer::SourcePublishSinkName(size_t source_index) const {
  return std::string(kSourceSinkPrefix) + std::to_string(source_index);
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

  PublishedFrame frame = context->owner->ExtractFrame(sample);
  gst_sample_unref(sample);
  if (frame.data.empty()) {
    return GST_FLOW_ERROR;
  }
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

  PublishedFrame frame = context->owner->ExtractFrame(sample);
  gst_sample_unref(sample);
  if (frame.data.empty()) {
    return GST_FLOW_ERROR;
  }
  if (context->owner->stitched_publish_callback_) {
    context->owner->stitched_publish_callback_(std::move(frame));
  }
  return GST_FLOW_OK;
}
}  // namespace camera_gst
}  // namespace drivers
}  // namespace apollo
