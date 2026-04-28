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

#include "cyber/cyber.h"
#include "cyber/time/time.h"
#include "opencv2/imgproc.hpp"

namespace apollo {
namespace drivers {
namespace camera_gst {

namespace {

bool IsDevicePathImpl(const std::string& uri) {
  return uri.rfind("/dev/video", 0) == 0;
}

bool IsNumericIndexImpl(const std::string& uri) {
  return !uri.empty() &&
         std::all_of(uri.begin(), uri.end(),
                     [](unsigned char c) { return std::isdigit(c) != 0; });
}

int ResolveApiPreference(const std::string& uri) {
  return IsDevicePathImpl(uri) || IsNumericIndexImpl(uri) ? cv::CAP_V4L2
                                                          : cv::CAP_ANY;
}

}  // namespace

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
    frame->image_bgr = raw_frame.clone();
  } else if (raw_frame.channels() == 4) {
    cv::cvtColor(raw_frame, frame->image_bgr, cv::COLOR_BGRA2BGR);
  } else if (raw_frame.channels() == 1) {
    cv::cvtColor(raw_frame, frame->image_bgr, cv::COLOR_GRAY2BGR);
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
  return std::unique_ptr<FrameSource>(new OpenCvFrameSource(config));
}

}  // namespace camera_gst
}  // namespace drivers
}  // namespace apollo
