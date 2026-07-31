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
constexpr const char* kPublishRgbPixelFormat = "RGB";

bool IsDevicePath(const std::string& uri) { return uri.rfind("/dev/", 0) == 0; }

bool IsNumericIndex(const std::string& uri) {
  return !uri.empty() &&
         std::all_of(uri.begin(), uri.end(),
                     [](unsigned char c) { return std::isdigit(c) != 0; });
}

bool IsArgusUri(const std::string& uri) {
  return uri.rfind("csi://", 0) == 0 || uri.rfind("argus://", 0) == 0;
}

bool IsV4l2DeviceUri(const std::string& uri) {
  return IsDevicePath(uri) || IsNumericIndex(uri);
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

CameraGstPipelineBuilder::CaptureBackend ParseCaptureBackend(
    const std::string& token) {
  const std::string upper = ToUpperCopy(token);
  if (upper.empty() || upper == "AUTO") {
    return CameraGstPipelineBuilder::CaptureBackend::kAuto;
  }
  if (upper == "ARGUS" || upper == "NVARGUS") {
    return CameraGstPipelineBuilder::CaptureBackend::kArgus;
  }
  if (upper == "NVV4L2_DMABUF" || upper == "NVV4L2" ||
      upper == "NVV4L2CAMERASRC") {
    return CameraGstPipelineBuilder::CaptureBackend::kNvV4l2Dmabuf;
  }
  if (upper == "V4L2_DMABUF" || upper == "V4L2") {
    return CameraGstPipelineBuilder::CaptureBackend::kV4l2Dmabuf;
  }
  if (upper == "CUSTOM") {
    return CameraGstPipelineBuilder::CaptureBackend::kCustom;
  }
  return CameraGstPipelineBuilder::CaptureBackend::kAuto;
}

std::string BuildFramerate(double fps) {
  const int fps_num = std::max(1, static_cast<int>(std::lround(fps)));
  return std::to_string(fps_num) + "/1";
}

void AppendNvencOptions(std::ostringstream* branch, uint32_t bitrate,
                        uint32_t iframe_interval, uint32_t idr_interval,
                        bool insert_sps_pps) {
  if (branch == nullptr) {
    return;
  }
  (*branch) << "bitrate=" << bitrate << ' '
            << "iframeinterval=" << iframe_interval << ' '
            << "idrinterval=" << idr_interval << ' '
            << "control-rate=1 preset-level=1 output-io-mode=5 "
            << "maxperf-enable=1 ";
  if (insert_sps_pps) {
    (*branch) << "insert-sps-pps=true ";
  }
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

std::string NvV4l2CapsFormatForFourcc(const std::string& fourcc) {
  const std::string upper = ToUpperCopy(fourcc);
  // nvv4l2camerasrc on Jetson exposes UYVY for common 8-bit YUV422 sensors.
  // Requesting YUY2 here causes caps negotiation failures on this platform.
  if (upper == "YUYV" || upper == "YUY2" || upper == "UYVY") {
    return "UYVY";
  }
  if (upper == "NV12") {
    return "NV12";
  }
  return "";
}

std::string DevicePathForSource(
    const config::CameraSourceConfig& source_config) {
  if (!source_config.device().empty()) {
    return source_config.device();
  }
  if (IsNumericIndex(source_config.uri())) {
    return "/dev/video" + source_config.uri();
  }
  if (IsDevicePath(source_config.uri())) {
    return source_config.uri();
  }
  return "";
}

bool SupportsNvV4l2Dmabuf(const config::CameraSourceConfig& source_config) {
  return !NvV4l2CapsFormatForFourcc(source_config.fourcc()).empty();
}

std::string PublishFormatToken(const config::PublishConfig& publish_config) {
  if (!publish_config.has_output_format()) {
    return kPublishRgbPixelFormat;
  }
  const std::string upper = ToUpperCopy(publish_config.output_format());
  if (upper == "YUYV" || upper == "YUY2") {
    return "YUY2";
  }
  return kPublishRgbPixelFormat;
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
  // Single-source stream-only mode bypasses compositor for stability.
  const bool direct_single_source_stream = IsDirectSingleSourceStream();
  if (stitched_publish_enabled_ ||
      (stream_enabled_ && !direct_single_source_stream)) {
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
         << " ! nvv4l2h264enc name=stream_encoder ";
  AppendNvencOptions(
      &branch, config_.stream().bitrate(), config_.stream().iframe_interval(),
      config_.stream().idr_interval(), config_.stream().insert_sps_pps());
  branch << "! h264parse ! rtph264pay config-interval=1 pt="
         << config_.stream().rtp_payload_type() << " ! udpsink host="
         << ShellQuoteForGstProperty(config_.stream().host())
         << " port=" << config_.stream().port() << " sync=false async=false";
  return branch.str();
}

std::vector<std::string> CameraGstPipelineBuilder::RequiredFactories() const {
  std::set<std::string> factories = {"queue", "tee", VideoConvertElement()};
  const bool direct_single_source_stream = IsDirectSingleSourceStream();
  if (stitched_publish_enabled_ ||
      (stream_enabled_ && !direct_single_source_stream)) {
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
    switch (ResolveCaptureBackend(source_config)) {
      case CaptureBackend::kArgus:
        factories.insert("nvarguscamerasrc");
        break;
      case CaptureBackend::kNvV4l2Dmabuf:
        factories.insert("nvv4l2camerasrc");
        break;
      case CaptureBackend::kV4l2Dmabuf: {
        const std::string upper_fourcc = ToUpperCopy(source_config.fourcc());
        factories.insert("v4l2src");
        if (upper_fourcc == "MJPG" || upper_fourcc == "JPEG") {
          factories.insert("jpegparse");
          factories.insert("nvv4l2decoder");
        }
        break;
      }
      case CaptureBackend::kCustom:
      case CaptureBackend::kAuto:
        break;
    }
  }
  if (stream_enabled_ || GpuPublishViaCodecEnabled()) {
    factories.insert("nvv4l2h264enc");
    factories.insert("h264parse");
  }
  if (stream_enabled_) {
    factories.insert("rtph264pay");
    factories.insert("udpsink");
  }
  if (GpuPublishViaCodecEnabled()) {
    factories.insert("nvv4l2decoder");
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

  compositor << "! queue leaky=downstream max-size-buffers=1 ! "
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
               << "video/x-raw,format=(string)" << kPublishRgbPixelFormat
               << " ! appsink name=" << kStitchedPublishSinkName
               << " sync=false max-buffers=1 drop=true ";
  }
  return compositor.str();
}

std::string CameraGstPipelineBuilder::BuildSourceDescription(
    size_t source_index, const config::CameraSourceConfig& source_config,
    const PipelineLayoutSlot* layout_slot) const {
  const bool direct_single_source_stream = IsDirectSingleSourceStream();
  const bool publish_enabled = source_config.has_publish() &&
                               !source_config.publish().channel_name().empty();
  const std::string source_head = BuildSourceHead(source_config);
  if (source_head.empty()) {
    return "";
  }

  const std::string tee_name = direct_single_source_stream
                                   ? std::string(kStitchedTeeName)
                                   : SourceTeeName(source_index);
  const uint32_t source_queue_capacity =
      publish_enabled
          ? std::max<uint32_t>(1, source_config.publish().queue_capacity())
          : 1;
  std::ostringstream branch;
  branch << source_head << " ! queue leaky=downstream max-size-buffers=1 ! "
         << VideoConvertElement()
         << " ! video/x-raw(memory:NVMM),format=(string)" << kGpuPixelFormat
         << ",width=(int)" << source_config.width() << ",height=(int)"
         << source_config.height() << ",framerate=(fraction)"
         << BuildFramerate(source_config.fps()) << " ! tee name=" << tee_name
         << (direct_single_source_stream ? " allow-not-linked=true" : "") << ' ';

  if (publish_enabled) {
    const uint32_t publish_width = source_config.publish().output_width() == 0
                                       ? source_config.width()
                                       : source_config.publish().output_width();
    const uint32_t publish_height =
        source_config.publish().output_height() == 0
            ? source_config.height()
            : source_config.publish().output_height();
    const std::string publish_format =
        PublishFormatToken(source_config.publish());
    branch << tee_name << ". ! queue leaky=downstream max-size-buffers="
           << source_queue_capacity << " ! " << VideoConvertElement() << " ! "
           << "video/x-raw,width=(int)" << publish_width << ",height=(int)"
           << publish_height;
    if (publish_format == "YUY2") {
      branch << ",format=(string)YUY2";
    } else {
      branch
          << ",format=(string)BGRx ! videoconvert ! video/x-raw,format=(string)"
          << kPublishRgbPixelFormat;
    }
    if (source_config.publish().output_fps() > 0.0) {
      branch << " ! videorate ! video/x-raw,format=(string)" << publish_format
             << ",framerate=(fraction)"
             << BuildFramerate(source_config.publish().output_fps());
    }
    branch << " ! appsink name=" << SourcePublishSinkName(source_index)
           << " sync=false max-buffers=1 drop=true ";
  }
  if (gpu_frame_enabled_) {
    branch << BuildGpuPublishBranch(source_index, tee_name) << ' ';
  }
  if (layout_slot != nullptr && !direct_single_source_stream) {
    branch << tee_name << ". ! queue leaky=downstream max-size-buffers=1 ! "
           << "comp.sink_" << layout_slot->pad_index << ' ';
  }
  return branch.str();
}

std::string CameraGstPipelineBuilder::BuildSourceHead(
    const config::CameraSourceConfig& source_config) const {
  if (!source_config.capture_pipeline().empty()) {
    return source_config.capture_pipeline();
  }

  const CaptureBackend backend = ResolveCaptureBackend(source_config);
  const std::string framerate = BuildFramerate(source_config.fps());
  std::string device_path = DevicePathForSource(source_config);
  if (device_path.empty()) {
    if (backend != CaptureBackend::kArgus) {
      return "";
    }
  }

  const std::string caps_common =
      "width=(int)" + std::to_string(source_config.width()) + ",height=(int)" +
      std::to_string(source_config.height()) + ",framerate=(fraction)" +
      framerate;
  const std::string upper_fourcc = ToUpperCopy(source_config.fourcc());

  if (backend == CaptureBackend::kArgus) {
    int sensor_id = static_cast<int>(source_config.sensor_id());
    if (!source_config.has_sensor_id() &&
        !ParseArgusSensorId(source_config.uri(), &sensor_id)) {
      sensor_id = 0;
    }
    std::ostringstream source;
    source << "nvarguscamerasrc sensor-id=" << sensor_id;
    if (source_config.sensor_mode() > 0) {
      source << " sensor-mode=" << source_config.sensor_mode();
    }
    source << " do-timestamp=true ! video/x-raw(memory:NVMM),width=(int)"
           << source_config.width() << ",height=(int)" << source_config.height()
           << ",format=(string)NV12,framerate=(fraction)" << framerate;
    return source.str();
  }

  if (upper_fourcc == "MJPG" || upper_fourcc == "JPEG") {
    return "v4l2src device=" + QuoteForGst(device_path) +
           " io-mode=4 do-timestamp=true ! image/jpeg," + caps_common +
           " ! jpegparse ! nvv4l2decoder mjpeg=1";
  }

  const std::string raw_format = GstRawFormatForFourcc(source_config.fourcc());
  if (raw_format.empty()) {
    return "";
  }
  if (backend == CaptureBackend::kNvV4l2Dmabuf) {
    const std::string nv_format =
        NvV4l2CapsFormatForFourcc(source_config.fourcc());
    if (nv_format.empty()) {
      return "";
    }
    if (nv_format != raw_format) {
      AWARN << "camera_gst mapped fourcc " << source_config.fourcc()
            << " to nvv4l2 caps format " << nv_format
            << " for stable DMABUF capture.";
    }
    return "nvv4l2camerasrc device=" + QuoteForGst(device_path) +
           " do-timestamp=true ! video/x-raw(memory:NVMM),format=(string)" +
           nv_format + "," + caps_common;
  }
  return "v4l2src device=" + QuoteForGst(device_path) +
         " io-mode=4 do-timestamp=true ! video/x-raw,format=(string)" +
         raw_format + "," + caps_common;
}

CameraGstPipelineBuilder::CaptureBackend
CameraGstPipelineBuilder::ResolveCaptureBackend(
    const config::CameraSourceConfig& source_config) const {
  if (!source_config.capture_pipeline().empty()) {
    return CaptureBackend::kCustom;
  }

  const CaptureBackend requested =
      ParseCaptureBackend(source_config.capture_backend());
  if (requested == CaptureBackend::kArgus) {
    return CaptureBackend::kArgus;
  }
  if (requested == CaptureBackend::kNvV4l2Dmabuf) {
    return SupportsNvV4l2Dmabuf(source_config) ? CaptureBackend::kNvV4l2Dmabuf
                                               : CaptureBackend::kV4l2Dmabuf;
  }
  if (requested == CaptureBackend::kV4l2Dmabuf) {
    return CaptureBackend::kV4l2Dmabuf;
  }

  if (IsArgusUri(source_config.uri())) {
    return CaptureBackend::kArgus;
  }
  if (IsV4l2DeviceUri(source_config.uri()) &&
      SupportsNvV4l2Dmabuf(source_config)) {
    return CaptureBackend::kNvV4l2Dmabuf;
  }
  return CaptureBackend::kV4l2Dmabuf;
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

bool CameraGstPipelineBuilder::IsDirectSingleSourceStream() const {
  return stream_enabled_ && !stitched_publish_enabled_ &&
         config_.sources_size() == 1;
}

bool CameraGstPipelineBuilder::GpuPublishViaCodecEnabled() const {
  return gpu_frame_enabled_ &&
         config_.stream().perception_transcode_before_publish();
}

std::string CameraGstPipelineBuilder::BuildGpuPublishBranch(
    size_t source_index, const std::string& tee_name) const {
  std::ostringstream branch;
  if (!GpuPublishViaCodecEnabled()) {
    // Perception default: source tee directly publishes NVMM frames.
    branch << tee_name << ". ! queue leaky=downstream max-size-buffers=1 ! "
           << "appsink name=" << SourceGpuSinkName(source_index)
           << " sync=false max-buffers=1 drop=true";
    return branch.str();
  }

  // Optional perception stress path: encode+decode stays on GPU, no RTP.
  branch << tee_name << ". ! queue name=perception_codec_queue_" << source_index
         << " leaky=downstream max-size-buffers="
         << std::max<uint32_t>(1, config_.stream().perception_queue_capacity())
         << " ! nvv4l2h264enc name=perception_codec_encoder_" << source_index
         << ' ';
  AppendNvencOptions(&branch, config_.stream().perception_bitrate(),
                     config_.stream().perception_iframe_interval(),
                     config_.stream().perception_idr_interval(),
                     config_.stream().perception_insert_sps_pps());
  branch << "! h264parse ! nvv4l2decoder ! queue leaky=downstream "
         << "max-size-buffers=1 ! " << VideoConvertElement()
         << " ! video/x-raw(memory:NVMM),format=(string)" << kGpuPixelFormat
         << " ! appsink name=" << SourceGpuSinkName(source_index)
         << " sync=false max-buffers=1 drop=true";
  return branch.str();
}

}  // namespace camera_gst
}  // namespace drivers
}  // namespace apollo
