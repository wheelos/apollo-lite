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

#include "modules/drivers/camera_gst/pipeline_builder.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <exception>
#include <set>
#include <sstream>

#include "gst/gst.h"

#include "cyber/common/log.h"

namespace apollo {
namespace drivers {
namespace camera_gst {

namespace {

constexpr const char* kSourceSinkPrefix = "source_publish_sink_";
constexpr const char* kSourceGpuSinkPrefix = "source_gpu_sink_";
constexpr const char* kSourceTeePrefix = "source_tee_";
constexpr const char* kStitchedTeeName = "stitched_tee";
constexpr const char* kStitchedPublishSinkName = "stitched_publish_sink";
constexpr const char* kGpuPixelFormat = "NV12";
constexpr const char* kPublishPixelFormat = "RGB";

bool IsDevicePath(const std::string& uri) { return uri.rfind("/dev/", 0) == 0; }

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
  std::transform(
      value.begin(), value.end(), value.begin(),
      [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
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

std::string ShellQuoteForGstProperty(const std::string& value) {
  return QuoteForGst(value);
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

bool FactoryExists(const std::string& factory_name) {
  if (!gst_is_initialized()) {
    int argc = 0;
    char** argv = nullptr;
    gst_init(&argc, &argv);
  }
  GstElementFactory* factory = gst_element_factory_find(factory_name.c_str());
  if (factory == nullptr) {
    return false;
  }
  gst_object_unref(factory);
  return true;
}

}  // namespace

CameraGstPipelineBuilder::CameraGstPipelineBuilder(
    const config::Config& config,
    const std::vector<PipelineLayoutSlot>& layout_slots,
    bool source_publish_enabled, bool stitched_publish_enabled,
    bool stream_enabled, bool gpu_frame_enabled)
    : config_(config),
      layout_slots_(layout_slots),
      source_publish_enabled_(source_publish_enabled),
      stitched_publish_enabled_(stitched_publish_enabled),
      stream_enabled_(stream_enabled),
      gpu_frame_enabled_(gpu_frame_enabled),
      output_width_(static_cast<int>(config_.cols() * config_.tile_width())),
      output_height_(static_cast<int>(config_.rows() * config_.tile_height())) {
}

std::string CameraGstPipelineBuilder::BuildPipelineDescription() const {
  std::ostringstream pipeline;
  if (stitched_publish_enabled_ || stream_enabled_) {
    pipeline << BuildCompositorDescription();
  }

  for (int index = 0; index < config_.sources_size(); ++index) {
    const auto& source_config = config_.sources(index);
    const std::string source_description =
        BuildSourceDescription(static_cast<size_t>(index), source_config,
                               FindLayoutSlot(source_config.name()));
    if (source_description.empty()) {
      AERROR << "camera_gst could not build a GPU source branch for "
             << source_config.name() << " uri: " << source_config.uri();
      return "";
    }
    pipeline << source_description << ' ';
  }
  return pipeline.str();
}

std::string CameraGstPipelineBuilder::BuildDefaultStreamBranch() const {
  std::ostringstream branch;
  branch << "queue name=stream_queue leaky=downstream max-size-buffers="
         << std::max<uint32_t>(1, config_.stream().queue_capacity())
         << " ! nvv4l2h264enc name=stream_encoder "
         << "bitrate=" << config_.stream().bitrate() << ' '
         << "iframeinterval=" << config_.stream().iframe_interval() << ' '
         << "idrinterval=" << config_.stream().idr_interval() << ' '
         << "control-rate=1 preset-level=1 bufapi-version=1 maxperf-enable=1 ";
  if (config_.stream().insert_sps_pps()) {
    branch << "insert-sps-pps=true ";
  }
  branch << "! h264parse ! rtph264pay config-interval=1 pt="
         << config_.stream().rtp_payload_type() << " ! udpsink host="
         << ShellQuoteForGstProperty(config_.stream().host())
         << " port=" << config_.stream().port() << " sync=false async=false";
  return branch.str();
}

std::vector<std::string> CameraGstPipelineBuilder::RequiredFactories() const {
  std::set<std::string> factories = {"queue", "tee", VideoConvertElement()};
  if (stitched_publish_enabled_ || stream_enabled_) {
    factories.insert("nvcompositor");
  }
  if (stitched_publish_enabled_ || source_publish_enabled_ ||
      gpu_frame_enabled_) {
    factories.insert("appsink");
  }
  for (const auto& source_config : config_.sources()) {
    if (source_config.has_publish() &&
        source_config.publish().output_fps() > 0.0) {
      factories.insert("videorate");
      break;
    }
  }

  for (const auto& source_config : config_.sources()) {
    if (!source_config.capture_pipeline().empty()) {
      continue;
    }
    if (IsArgusUri(source_config.uri())) {
      factories.insert("nvarguscamerasrc");
      continue;
    }
    const std::string upper_fourcc = ToUpperCopy(source_config.fourcc());
    if (upper_fourcc == "MJPG" || upper_fourcc == "JPEG") {
      factories.insert("v4l2src");
      factories.insert("jpegparse");
      factories.insert("nvv4l2decoder");
    } else {
      factories.insert("v4l2src");
    }
  }
  if (stream_enabled_) {
    factories.insert("nvv4l2h264enc");
    factories.insert("h264parse");
    factories.insert("rtph264pay");
    factories.insert("udpsink");
  }
  return std::vector<std::string>(factories.begin(), factories.end());
}

bool CameraGstPipelineBuilder::ValidateRequiredFactories() const {
  bool valid = true;
  for (const auto& factory_name : RequiredFactories()) {
    if (!FactoryExists(factory_name)) {
      AERROR << "camera_gst required GStreamer factory is missing: "
             << factory_name;
      valid = false;
    }
  }
  return valid;
}

std::string CameraGstPipelineBuilder::BuildCompositorDescription() const {
  std::ostringstream compositor;
  compositor << "nvcompositor name=comp ";
  for (const auto& slot : layout_slots_) {
    compositor << "sink_" << slot.pad_index
               << "::xpos=" << slot.col * static_cast<int>(config_.tile_width())
               << ' ' << "sink_" << slot.pad_index << "::ypos="
               << slot.row * static_cast<int>(config_.tile_height()) << ' '
               << "sink_" << slot.pad_index
               << "::width=" << config_.tile_width() << ' ' << "sink_"
               << slot.pad_index << "::height=" << config_.tile_height() << ' ';
  }

  compositor << "! queue leaky=downstream max-size-buffers=2 ! "
             << VideoConvertElement() << " ! "
             << "video/x-raw(memory:NVMM),format=(string)" << kGpuPixelFormat
             << ",width=(int)" << output_width_ << ",height=(int)"
             << output_height_ << ",framerate=(fraction)"
             << BuildFramerate(config_.fps())
             << " ! tee name=" << kStitchedTeeName << ' ';

  if (stitched_publish_enabled_) {
    compositor << kStitchedTeeName
               << ". ! queue leaky=downstream max-size-buffers=1 ! "
               << VideoConvertElement() << " ! "
               << "video/x-raw,format=(string)" << kPublishPixelFormat
               << " ! appsink name=" << kStitchedPublishSinkName
               << " sync=false max-buffers=1 drop=true ";
  }
  return compositor.str();
}

std::string CameraGstPipelineBuilder::BuildSourceDescription(
    size_t source_index, const config::CameraSourceConfig& source_config,
    const PipelineLayoutSlot* layout_slot) const {
  const bool publish_enabled = source_config.has_publish() &&
                               !source_config.publish().channel_name().empty();
  const std::string source_head = BuildSourceHead(source_config);
  if (source_head.empty()) {
    return "";
  }

  const std::string tee_name = SourceTeeName(source_index);
  std::ostringstream branch;
  branch << source_head << " ! queue leaky=downstream max-size-buffers=2 ! "
         << VideoConvertElement()
         << " ! video/x-raw(memory:NVMM),format=(string)" << kGpuPixelFormat
         << ",width=(int)" << source_config.width() << ",height=(int)"
         << source_config.height() << ",framerate=(fraction)"
         << BuildFramerate(source_config.fps()) << " ! tee name=" << tee_name
         << ' ';

  if (publish_enabled) {
    const uint32_t publish_width = source_config.publish().output_width() == 0
                                       ? source_config.width()
                                       : source_config.publish().output_width();
    const uint32_t publish_height =
        source_config.publish().output_height() == 0
            ? source_config.height()
            : source_config.publish().output_height();
    branch << tee_name << ". ! queue leaky=downstream max-size-buffers=1 ! "
           << VideoConvertElement() << " ! "
           << "video/x-raw,width=(int)" << publish_width << ",height=(int)"
           << publish_height << ",format=(string)" << kPublishPixelFormat;
    if (source_config.publish().output_fps() > 0.0) {
      branch << " ! videorate ! video/x-raw,format=(string)"
             << kPublishPixelFormat << ",framerate=(fraction)"
             << BuildFramerate(source_config.publish().output_fps());
    }
    branch << " ! appsink name=" << SourcePublishSinkName(source_index)
           << " sync=false max-buffers=1 drop=true ";
  }
  if (gpu_frame_enabled_) {
    branch << tee_name << ". ! queue leaky=downstream max-size-buffers=1 ! "
           << "appsink name=" << SourceGpuSinkName(source_index)
           << " sync=false max-buffers=1 drop=true ";
  }
  if (layout_slot != nullptr) {
    branch << tee_name << ". ! queue leaky=downstream max-size-buffers=2 ! "
           << "comp.sink_" << layout_slot->pad_index << ' ';
  }
  return branch.str();
}

std::string CameraGstPipelineBuilder::BuildSourceHead(
    const config::CameraSourceConfig& source_config) const {
  if (!source_config.capture_pipeline().empty()) {
    return source_config.capture_pipeline();
  }

  const std::string framerate = BuildFramerate(source_config.fps());
  if (IsArgusUri(source_config.uri())) {
    int sensor_id = 0;
    if (!ParseArgusSensorId(source_config.uri(), &sensor_id)) {
      return "";
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
    return "";
  }

  const std::string caps_common =
      "width=(int)" + std::to_string(source_config.width()) + ",height=(int)" +
      std::to_string(source_config.height()) + ",framerate=(fraction)" +
      framerate;
  const std::string upper_fourcc = ToUpperCopy(source_config.fourcc());
  if (upper_fourcc == "MJPG" || upper_fourcc == "JPEG") {
    return "v4l2src device=" + QuoteForGst(device_path) +
           " io-mode=4 do-timestamp=true ! image/jpeg," + caps_common +
           " ! jpegparse ! nvv4l2decoder mjpeg=1";
  }

  const std::string raw_format = GstRawFormatForFourcc(source_config.fourcc());
  if (raw_format.empty()) {
    return "";
  }
  return "v4l2src device=" + QuoteForGst(device_path) +
         " io-mode=4 do-timestamp=true ! video/x-raw,format=(string)" +
         raw_format + "," + caps_common;
}

const PipelineLayoutSlot* CameraGstPipelineBuilder::FindLayoutSlot(
    const std::string& source_name) const {
  const auto iter =
      std::find_if(layout_slots_.begin(), layout_slots_.end(),
                   [&source_name](const PipelineLayoutSlot& slot) {
                     return slot.source_name == source_name;
                   });
  return iter == layout_slots_.end() ? nullptr : &(*iter);
}

std::string CameraGstPipelineBuilder::SourceTeeName(size_t source_index) const {
  return std::string(kSourceTeePrefix) + std::to_string(source_index);
}

std::string CameraGstPipelineBuilder::SourcePublishSinkName(
    size_t source_index) const {
  return std::string(kSourceSinkPrefix) + std::to_string(source_index);
}

std::string CameraGstPipelineBuilder::SourceGpuSinkName(
    size_t source_index) const {
  return std::string(kSourceGpuSinkPrefix) + std::to_string(source_index);
}

std::string CameraGstPipelineBuilder::VideoConvertElement() const {
  if (FactoryExists("nvvidconv")) {
    return "nvvidconv";
  }
  if (FactoryExists("nvvideoconvert")) {
    return "nvvideoconvert";
  }
  return "nvvidconv";
}

}  // namespace camera_gst
}  // namespace drivers
}  // namespace apollo
