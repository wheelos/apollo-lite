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

#include "modules/drivers/camera_gst/frame_source.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <mutex>
#include <sstream>

#include "cyber/cyber.h"
#include "cyber/time/time.h"
#include "gst/app/gstappsink.h"
#include "opencv2/imgproc.hpp"

namespace apollo {
namespace drivers {
namespace camera_gst {

namespace {

constexpr const char* kSourceAppSinkName = "source_sink";

bool IsDevicePathImpl(const std::string& uri) {
  return uri.rfind("/dev/", 0) == 0;
}

bool IsNumericIndexImpl(const std::string& uri) {
  return !uri.empty() &&
         std::all_of(uri.begin(), uri.end(),
                     [](unsigned char c) { return std::isdigit(c) != 0; });
}

bool IsArgusUriImpl(const std::string& uri) {
  return uri.rfind("csi://", 0) == 0 || uri.rfind("argus://", 0) == 0;
}

bool ParseArgusSensorId(const std::string& uri, int* sensor_id) {
  if (sensor_id == nullptr || !IsArgusUriImpl(uri)) {
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

int ResolveApiPreference(const std::string& uri) {
  return IsDevicePathImpl(uri) || IsNumericIndexImpl(uri) ? cv::CAP_V4L2
                                                          : cv::CAP_ANY;
}

std::string ToUpperCopy(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) {
                   return static_cast<char>(std::toupper(c));
                 });
  return value;
}

std::string QuoteForGst(const std::string& value) {
  std::string quoted = "\"";
  quoted.reserve(value.size() + 2);
  for (char c : value) {
    if (c == '\\' || c == '"') {
      quoted.push_back('\\');
    }
    quoted.push_back(c);
  }
  quoted.push_back('"');
  return quoted;
}

std::string BuildFramerate(double fps) {
  const int fps_num = std::max(1, static_cast<int>(std::lround(fps)));
  return std::to_string(fps_num) + "/1";
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
  if (upper == "RGB") {
    return "RGB";
  }
  return "";
}

void GstInitOnce() {
  static std::once_flag gst_init_once;
  std::call_once(gst_init_once, []() {
    int argc = 0;
    char** argv = nullptr;
    gst_init(&argc, &argv);
  });
}

}  // namespace

GstFrameSource::GstFrameSource(const config::CameraSourceConfig& config)
    : config_(config) {}

GstFrameSource::~GstFrameSource() { Close(); }

const std::vector<std::string>& GstFrameSource::PipelineDescriptions() {
  if (pipeline_descriptions_.empty()) {
    pipeline_descriptions_ = BuildPipelineDescriptions();
  }
  return pipeline_descriptions_;
}

void GstFrameSource::AdvancePipelineDescription() {
  const auto& descriptions = PipelineDescriptions();
  if (descriptions.empty()) {
    return;
  }

  if (has_opened_pipeline_description_index_) {
    next_pipeline_description_index_ =
        (opened_pipeline_description_index_ + 1) % descriptions.size();
    has_opened_pipeline_description_index_ = false;
    return;
  }

  next_pipeline_description_index_ =
      (next_pipeline_description_index_ + 1) % descriptions.size();
}

bool GstFrameSource::Open() {
  if (opened_) {
    return true;
  }

  GstInitOnce();
  const auto& descriptions = PipelineDescriptions();
  if (descriptions.empty()) {
    AERROR << "camera_gst source " << config_.name()
           << " has no supported GStreamer capture pipeline.";
    return false;
  }

  const size_t start_index =
      next_pipeline_description_index_ % descriptions.size();
  std::string last_error = "no candidate pipeline succeeded";
  for (size_t attempt = 0; attempt < descriptions.size(); ++attempt) {
    const size_t index = (start_index + attempt) % descriptions.size();
    const auto& description = descriptions[index];
    GError* error = nullptr;
    GstElement* candidate_pipeline = gst_parse_launch(description.c_str(), &error);
    if (candidate_pipeline == nullptr) {
      last_error = error == nullptr ? "unknown parse error" : error->message;
      if (error != nullptr) {
        g_error_free(error);
      }
      AWARN << "camera_gst source " << config_.name()
            << " rejected pipeline candidate: " << last_error;
      continue;
    }

    GstElement* candidate_sink =
        gst_bin_get_by_name(GST_BIN(candidate_pipeline), kSourceAppSinkName);
    if (candidate_sink == nullptr) {
      last_error = "appsink named source_sink not found";
      gst_object_unref(candidate_pipeline);
      AWARN << "camera_gst source " << config_.name()
            << " rejected pipeline candidate: " << last_error;
      continue;
    }

    g_object_set(candidate_sink, "emit-signals", FALSE, "max-buffers", 1u,
                 "drop", TRUE, "sync", FALSE, nullptr);
    if (gst_element_set_state(candidate_pipeline, GST_STATE_PLAYING) ==
        GST_STATE_CHANGE_FAILURE) {
      last_error = "failed to set pipeline to PLAYING";
      gst_object_unref(candidate_sink);
      gst_element_set_state(candidate_pipeline, GST_STATE_NULL);
      gst_object_unref(candidate_pipeline);
      AWARN << "camera_gst source " << config_.name()
            << " rejected pipeline candidate: " << last_error;
      continue;
    }

    gst_element_get_state(candidate_pipeline, nullptr, nullptr, GST_SECOND);
    pipeline_ = candidate_pipeline;
    appsink_ = candidate_sink;
    opened_ = true;
    opened_pipeline_description_index_ = index;
    next_pipeline_description_index_ = index;
    has_opened_pipeline_description_index_ = true;
    return true;
  }

  next_pipeline_description_index_ = (start_index + 1) % descriptions.size();

  AERROR << "camera_gst failed to open source pipeline for " << config_.name()
         << ": " << last_error;
  return false;
}

void GstFrameSource::Close() {
  opened_ = false;
  if (appsink_ != nullptr) {
    gst_object_unref(appsink_);
    appsink_ = nullptr;
  }
  if (pipeline_ != nullptr) {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
  }
}

std::string GstFrameSource::BuildDeviceGpuPipelineDescription(
    const std::string& device_path) const {
  const std::string framerate = BuildFramerate(config_.fps());
  const std::string caps_common = "width=(int)" + std::to_string(config_.width()) +
                                  ",height=(int)" +
                                  std::to_string(config_.height()) +
                                  ",framerate=(fraction)" + framerate;
  const std::string upper_fourcc = ToUpperCopy(config_.fourcc());
  const std::string app_sink =
      std::string("appsink name=") + kSourceAppSinkName +
      " sync=false max-buffers=1 drop=true";

  if (upper_fourcc == "MJPG" || upper_fourcc == "JPEG") {
    return "v4l2src device=" + QuoteForGst(device_path) +
           " io-mode=2 do-timestamp=true ! image/jpeg," + caps_common +
           " ! jpegparse ! nvv4l2decoder mjpeg=1 ! nvvidconv ! "
           "video/x-raw,format=(string)BGRx ! videoconvert ! "
           "video/x-raw,format=(string)RGB ! " + app_sink;
  }

  const std::string raw_format = GstRawFormatForFourcc(config_.fourcc());
  if (raw_format.empty()) {
    return "";
  }
  if (raw_format == "RGB") {
    return "v4l2src device=" + QuoteForGst(device_path) +
           " io-mode=2 do-timestamp=true ! video/x-raw,format=(string)RGB," +
           caps_common + " ! " + app_sink;
  }

  return "v4l2src device=" + QuoteForGst(device_path) +
         " io-mode=2 do-timestamp=true ! video/x-raw,format=(string)" +
         raw_format + "," + caps_common +
         " ! nvvidconv ! video/x-raw,format=(string)BGRx ! videoconvert ! "
         "video/x-raw,format=(string)RGB ! " + app_sink;
}

std::string GstFrameSource::BuildDeviceCpuPipelineDescription(
    const std::string& device_path) const {
  const std::string framerate = BuildFramerate(config_.fps());
  const std::string caps_common = "width=(int)" + std::to_string(config_.width()) +
                                  ",height=(int)" +
                                  std::to_string(config_.height()) +
                                  ",framerate=(fraction)" + framerate;
  const std::string upper_fourcc = ToUpperCopy(config_.fourcc());
  const std::string raw_format = GstRawFormatForFourcc(config_.fourcc());
  const std::string app_sink =
      std::string("appsink name=") + kSourceAppSinkName +
      " sync=false max-buffers=1 drop=true";

  if (upper_fourcc == "MJPG" || upper_fourcc == "JPEG") {
    return "v4l2src device=" + QuoteForGst(device_path) +
           " io-mode=2 do-timestamp=true ! image/jpeg," + caps_common +
           " ! jpegparse ! jpegdec ! videoconvert ! "
           "video/x-raw,format=(string)RGB ! " + app_sink;
  }

  if (raw_format.empty()) {
    return "";
  }
  if (raw_format == "RGB") {
    return "v4l2src device=" + QuoteForGst(device_path) +
           " io-mode=2 do-timestamp=true ! video/x-raw,format=(string)RGB," +
           caps_common + " ! " + app_sink;
  }

  return "v4l2src device=" + QuoteForGst(device_path) +
         " io-mode=2 do-timestamp=true ! video/x-raw,format=(string)" +
         raw_format + "," + caps_common +
         " ! videoconvert ! video/x-raw,format=(string)RGB ! " + app_sink;
}

std::string GstFrameSource::BuildArgusPipelineDescription(int sensor_id) const {
  const std::string framerate = BuildFramerate(config_.fps());
  return "nvarguscamerasrc sensor-id=" + std::to_string(sensor_id) +
         " do-timestamp=true ! video/x-raw(memory:NVMM),width=(int)" +
         std::to_string(config_.width()) + ",height=(int)" +
         std::to_string(config_.height()) +
         ",format=(string)NV12,framerate=(fraction)" + framerate +
         " ! nvvidconv ! video/x-raw,format=(string)BGRx ! videoconvert ! "
         "video/x-raw,format=(string)RGB ! appsink name=" +
         std::string(kSourceAppSinkName) + " sync=false max-buffers=1 drop=true";
}

std::vector<std::string> GstFrameSource::BuildPipelineDescriptions() const {
  std::vector<std::string> descriptions;
  if (!config_.capture_pipeline().empty()) {
    descriptions.push_back(config_.capture_pipeline());
    return descriptions;
  }
  if (IsArgusUriImpl(config_.uri())) {
    int sensor_id = 0;
    if (!ParseArgusSensorId(config_.uri(), &sensor_id)) {
      return descriptions;
    }
    descriptions.push_back(BuildArgusPipelineDescription(sensor_id));
    return descriptions;
  }
  std::string device_path;
  if (IsNumericIndexImpl(config_.uri())) {
    device_path = "/dev/video" + config_.uri();
  } else if (IsDevicePathImpl(config_.uri())) {
    device_path = config_.uri();
  }
  if (!device_path.empty()) {
    const std::string gpu_pipeline =
        BuildDeviceGpuPipelineDescription(device_path);
    if (!gpu_pipeline.empty()) {
      descriptions.push_back(gpu_pipeline);
    }
    const std::string cpu_pipeline =
        BuildDeviceCpuPipelineDescription(device_path);
    if (!cpu_pipeline.empty() && cpu_pipeline != gpu_pipeline) {
      descriptions.push_back(cpu_pipeline);
    }
  }
  return descriptions;
}

bool GstFrameSource::Read(CapturedFrame* frame) {
  if (frame == nullptr) {
    return false;
  }
  if (!opened_ && !Open()) {
    return false;
  }

  GstSample* sample = gst_app_sink_try_pull_sample(
      GST_APP_SINK(appsink_),
      static_cast<GstClockTime>(config_.read_timeout_ms()) * GST_MSECOND);
  if (sample == nullptr) {
    AWARN_EVERY(100) << "camera_gst source " << config_.name()
                     << " timed out waiting for a sample; trying the next "
                     << "capture pipeline candidate.";
    AdvancePipelineDescription();
    Close();
    return false;
  }

  GstBuffer* buffer = gst_sample_get_buffer(sample);
  GstCaps* caps = gst_sample_get_caps(sample);
  if (buffer == nullptr || caps == nullptr) {
    AWARN_EVERY(100) << "camera_gst source " << config_.name()
                     << " received invalid sample metadata.";
    gst_sample_unref(sample);
    AdvancePipelineDescription();
    Close();
    return false;
  }

  GstStructure* structure = gst_caps_get_structure(caps, 0);
  int width = 0;
  int height = 0;
  gst_structure_get_int(structure, "width", &width);
  gst_structure_get_int(structure, "height", &height);
  if (width <= 0 || height <= 0) {
      AWARN_EVERY(100) << "camera_gst source " << config_.name()
           << " reported invalid frame size.";
    gst_sample_unref(sample);
      AdvancePipelineDescription();
      Close();
    return false;
  }

  frame->source_name = config_.name();
  frame->image_rgb = cv::Mat(height, width, CV_8UC3);
  const size_t bytes = frame->image_rgb.total() * frame->image_rgb.elemSize();
  const gsize copied =
      gst_buffer_extract(buffer, 0, frame->image_rgb.data, bytes);
  if (copied != bytes) {
      AWARN_EVERY(100) << "camera_gst source " << config_.name()
           << " copied " << copied << " bytes, expected "
           << bytes << ".";
    gst_sample_unref(sample);
      AdvancePipelineDescription();
      Close();
    return false;
  }
  frame->measurement_time =
      GST_BUFFER_PTS_IS_VALID(buffer)
          ? static_cast<double>(GST_BUFFER_PTS(buffer)) / GST_SECOND
          : apollo::cyber::Time::Now().ToSecond();
  gst_sample_unref(sample);
  return true;
}

FailoverFrameSource::FailoverFrameSource(std::unique_ptr<FrameSource> primary,
                                         std::unique_ptr<FrameSource> secondary,
                                         size_t primary_failure_threshold)
    : primary_(std::move(primary)),
      secondary_(std::move(secondary)),
      primary_failure_threshold_(std::max<size_t>(1, primary_failure_threshold)) {
  if (primary_ != nullptr) {
    name_ = primary_->name();
  } else if (secondary_ != nullptr) {
    name_ = secondary_->name();
  }
}

bool FailoverFrameSource::Read(CapturedFrame* frame) {
  if (!using_secondary_ && primary_ != nullptr) {
    if (primary_->Read(frame)) {
      primary_failure_count_ = 0;
      return true;
    }

    ++primary_failure_count_;
    if (secondary_ == nullptr ||
        primary_failure_count_ < primary_failure_threshold_) {
      return false;
    }

    using_secondary_ = true;
    AWARN << "camera_gst source " << name_
          << " is falling back to OpenCV capture after repeated GStreamer "
          << "startup failures.";
  }

  return secondary_ != nullptr && secondary_->Read(frame);
}

OpenCvFrameSource::OpenCvFrameSource(const config::CameraSourceConfig& config)
    : config_(config) {}

OpenCvFrameSource::~OpenCvFrameSource() { capture_.release(); }

bool OpenCvFrameSource::Open() {
  if (config_.uri().empty()) {
    AERROR << "camera_gst source " << config_.name() << " has empty uri.";
    return false;
  }

  const int api_preference = ResolveApiPreference(config_.uri());
  if (IsNumericIndexImpl(config_.uri())) {
    const int camera_index = std::stoi(config_.uri());
    opened_ = capture_.open(camera_index, api_preference);
  } else {
    opened_ = capture_.open(config_.uri(), api_preference);
  }
  if (!opened_) {
    AERROR << "Failed to open camera_gst source " << config_.name()
           << " uri: " << config_.uri();
    return false;
  }

  if (config_.width() > 0) {
    capture_.set(cv::CAP_PROP_FRAME_WIDTH, config_.width());
  }
  if (config_.height() > 0) {
    capture_.set(cv::CAP_PROP_FRAME_HEIGHT, config_.height());
  }
  if (config_.fps() > 0.0) {
    capture_.set(cv::CAP_PROP_FPS, config_.fps());
  }
  if (config_.fourcc().size() == 4) {
    capture_.set(cv::CAP_PROP_FOURCC,
                 cv::VideoWriter::fourcc(config_.fourcc()[0],
                                         config_.fourcc()[1],
                                         config_.fourcc()[2],
                                         config_.fourcc()[3]));
  }
  return true;
}

bool OpenCvFrameSource::Read(CapturedFrame* frame) {
  if (frame == nullptr) {
    return false;
  }
  if (!opened_ && !Open()) {
    return false;
  }

  cv::Mat raw_frame;
  if (!capture_.read(raw_frame) || raw_frame.empty()) {
    capture_.release();
    opened_ = false;
    return false;
  }

  frame->source_name = config_.name();
  frame->measurement_time = apollo::cyber::Time::Now().ToSecond();
  if (raw_frame.channels() == 3) {
    cv::cvtColor(raw_frame, frame->image_rgb, cv::COLOR_BGR2RGB);
  } else if (raw_frame.channels() == 4) {
    cv::cvtColor(raw_frame, frame->image_rgb, cv::COLOR_BGRA2RGB);
  } else if (raw_frame.channels() == 1) {
    cv::cvtColor(raw_frame, frame->image_rgb, cv::COLOR_GRAY2RGB);
  } else {
    AERROR << "Unsupported channel count from source " << config_.name()
           << ": " << raw_frame.channels();
    return false;
  }
  return true;
}

bool OpenCvFrameSource::IsDevicePath(const std::string& uri) {
  return IsDevicePathImpl(uri);
}

bool OpenCvFrameSource::IsNumericIndex(const std::string& uri) {
  return IsNumericIndexImpl(uri);
}

std::unique_ptr<FrameSource> CreateFrameSource(
    const config::CameraSourceConfig& config) {
  if (!config.capture_pipeline().empty() || IsArgusUriImpl(config.uri())) {
    return std::unique_ptr<FrameSource>(new GstFrameSource(config));
  }
  if (IsDevicePathImpl(config.uri()) || IsNumericIndexImpl(config.uri())) {
    return std::unique_ptr<FrameSource>(new FailoverFrameSource(
        std::unique_ptr<FrameSource>(new GstFrameSource(config)),
        std::unique_ptr<FrameSource>(new OpenCvFrameSource(config)), 2));
  }
  return std::unique_ptr<FrameSource>(new OpenCvFrameSource(config));
}

}  // namespace camera_gst
}  // namespace drivers
}  // namespace apollo
