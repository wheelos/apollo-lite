// Copyright 2026 The WheelOS Team. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "modules/drivers/camera/backend/image_processor.h"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>

extern "C" {
#include "libavcodec/avcodec.h"
#include "libavformat/avformat.h"
#include "libavutil/error.h"
#include "libavutil/imgutils.h"
#include "libswscale/swscale.h"
}

#include "libavcodec/version.h"

#include "cyber/common/log.h"
#if LIBAVCODEC_VERSION_MAJOR < 55
#define AV_CODEC_ID_MJPEG CODEC_ID_MJPEG
#endif

namespace apollo {
namespace drivers {
namespace camera {

// --- YuvProcessor Helper Functions (now private to the class, declared static)
namespace {

// Convert UYVY (U Y V Y) to YUYV (Y U Y V)
inline void ConvertUYVYToYUYV(const uint8_t* src, size_t len,
                              std::vector<uint8_t>& out) {
  out.resize(len);
  for (size_t i = 0; i + 3 < len; i += 4) {
    out[i + 0] = src[i + 1];  // Y0
    out[i + 1] = src[i + 0];  // U0
    out[i + 2] = src[i + 3];  // Y1
    out[i + 3] = src[i + 2];  // V0
  }
}

// Convert VYUY (V Y U Y) to YUYV (Y U Y V)
inline void ConvertVYUYToYUYV(const uint8_t* src, size_t len,
                              std::vector<uint8_t>& out) {
  out.resize(len);
  for (size_t i = 0; i + 3 < len; i += 4) {
    out[i + 0] = src[i + 1];  // Y0
    out[i + 1] = src[i + 2];  // U0
    out[i + 2] = src[i + 3];  // Y1
    out[i + 3] = src[i + 0];  // V0
  }
}

// Convert YVYU (Y V Y U) to YUYV (Y U Y V)
inline void ConvertYVYUToYUYV(const uint8_t* src, size_t len,
                              std::vector<uint8_t>& out) {
  out.resize(len);
  for (size_t i = 0; i + 3 < len; i += 4) {
    out[i + 0] = src[i + 0];  // Y0
    out[i + 1] = src[i + 3];  // U0
    out[i + 2] = src[i + 2];  // Y1
    out[i + 3] = src[i + 1];  // V0
  }
}

}  // namespace

// --- YuvProcessor Implementation ---

YuvProcessor::YuvProcessor(OutputFormat format, YuvFormat yuv_format,
                           bool swap_uv)
    : output_format_(format), yuv_format_(yuv_format), swap_uv_(swap_uv) {}

void YuvProcessor::ConvertYUYVToRGB(const uint8_t* src, int width, int height,
                                    uint8_t* dst_rgb) {
  // YUV 4:2:2 formats use 2 bytes per pixel (4 bytes for 2 pixels)
  size_t yuyv_len = static_cast<size_t>(width) * height * 2;

  // Determine the effective format for conversion (swap UV if requested)
  YuvFormat effective_format = yuv_format_;
  if (swap_uv_) {
    // Swap U and V by toggling between formats
    switch (yuv_format_) {
      case YuvFormat::YUYV:
        effective_format = YuvFormat::YVYU;
        break;
      case YuvFormat::YVYU:
        effective_format = YuvFormat::YUYV;
        break;
      case YuvFormat::UYVY:
        effective_format = YuvFormat::VYUY;
        break;
      case YuvFormat::VYUY:
        effective_format = YuvFormat::UYVY;
        break;
    }
  }

  // For formats that OpenCV doesn't directly support, convert to YUYV first
  const uint8_t* convert_src = src;

  if (effective_format == YuvFormat::VYUY) {
    // VYUY is not directly supported by OpenCV, convert to YUYV first
    if (yuyv_buffer_.size() < yuyv_len) {
      yuyv_buffer_.resize(yuyv_len);
    }
    ConvertVYUYToYUYV(src, yuyv_len, yuyv_buffer_);
    convert_src = yuyv_buffer_.data();
    effective_format = YuvFormat::YUYV;
  }

  cv::Mat yuyv_mat(height, width, CV_8UC2, const_cast<uint8_t*>(convert_src));
  cv::Mat rgb_mat(height, width, CV_8UC3, dst_rgb);

  // Select the correct OpenCV conversion based on YUV format
  int cv_code;
  switch (effective_format) {
    case YuvFormat::YUYV:
      cv_code = cv::COLOR_YUV2RGB_YUYV;
      break;
    case YuvFormat::YVYU:
      cv_code = cv::COLOR_YUV2RGB_YVYU;
      break;
    case YuvFormat::UYVY:
      cv_code = cv::COLOR_YUV2RGB_UYVY;
      break;
    case YuvFormat::VYUY:
      // Already converted to YUYV above
      cv_code = cv::COLOR_YUV2RGB_YUYV;
      break;
    default:
      cv_code = cv::COLOR_YUV2RGB_YUYV;
      break;
  }
  cv::cvtColor(yuyv_mat, rgb_mat, cv_code);
}

bool YuvProcessor::Process(const void* src, size_t len,
                           std::shared_ptr<Image> dest_pb) {
  if (dest_pb == nullptr) {
    AERROR << "YuvProcessor: destination image is null";
    return false;
  }

  if (src == nullptr || len == 0) {
    AERROR << "YuvProcessor: source buffer is invalid";
    return false;
  }

  const int width = dest_pb->width();
  const int height = dest_pb->height();
  if (width <= 0 || height <= 0 || width % 2 != 0) {
    AERROR << "YuvProcessor: invalid YUV422 image size " << width << "x"
           << height;
    return false;
  }
  const size_t required_len =
      static_cast<size_t>(width) * static_cast<size_t>(height) * 2;
  if (len < required_len) {
    AERROR << "YuvProcessor: source buffer too small: " << len << " < "
           << required_len;
    return false;
  }

  const uint8_t* yuv_data_ptr = static_cast<const uint8_t*>(src);

  // Determine the effective format (account for UV swapping)
  YuvFormat effective_format = yuv_format_;
  if (swap_uv_) {
    switch (yuv_format_) {
      case YuvFormat::YUYV:
        effective_format = YuvFormat::YVYU;
        break;
      case YuvFormat::YVYU:
        effective_format = YuvFormat::YUYV;
        break;
      case YuvFormat::UYVY:
        effective_format = YuvFormat::VYUY;
        break;
      case YuvFormat::VYUY:
        effective_format = YuvFormat::UYVY;
        break;
    }
  }

  // Convert non-YUYV formats to YUYV first for passthrough mode
  if (output_format_ == OutputFormat::YUYV &&
      effective_format != YuvFormat::YUYV) {
    if (yuyv_buffer_.size() != len) {
      yuyv_buffer_.resize(len);
    }
    switch (effective_format) {
      case YuvFormat::UYVY:
        ConvertUYVYToYUYV(yuv_data_ptr, len, yuyv_buffer_);
        break;
      case YuvFormat::VYUY:
        ConvertVYUYToYUYV(yuv_data_ptr, len, yuyv_buffer_);
        break;
      case YuvFormat::YVYU:
        ConvertYVYUToYUYV(yuv_data_ptr, len, yuyv_buffer_);
        break;
      default:
        break;
    }
    yuv_data_ptr = yuyv_buffer_.data();
  }

  if (output_format_ == OutputFormat::YUYV) {
    // Direct copy to protobuf buffer for YUYV output
    dest_pb->set_data(yuv_data_ptr, required_len);
  } else if (output_format_ == OutputFormat::RGB) {
    const size_t needed = static_cast<size_t>(width) * height * 3;
    auto* out = dest_pb->mutable_data();
    if (out->size() < needed) {
      AERROR << "YuvProcessor: mutable_data buffer too small: " << out->size()
             << " < " << needed;
      out->resize(needed);
    }
    uint8_t* dest_rgb_ptr = reinterpret_cast<uint8_t*>(out->data());

    // Convert YUV to RGB using the correct format
    ConvertYUYVToRGB(yuv_data_ptr, width, height, dest_rgb_ptr);
  } else {
    AERROR << "YuvProcessor: unsupported output format "
           << static_cast<int>(output_format_);
    return false;
  }

  return true;
}

// --- MjpegProcessor Implementation ---

// Helper functions for FFmpeg error handling (replacing the macro)
namespace MjpegUtil {
inline std::string GetFFmpegErrorString(int ret_code) {
  char err_buf[AV_ERROR_MAX_STRING_SIZE];
  av_strerror(ret_code, err_buf, sizeof(err_buf));
  return err_buf;
}

inline void ThrowFFmpegError(int ret_code, const std::string& msg) {
  throw std::runtime_error(msg + ": " + GetFFmpegErrorString(ret_code));
}
}  // namespace MjpegUtil

MjpegProcessor::MjpegProcessor(int width, int height,
                               YuvProcessor::OutputFormat output_format)
    : width_(width), height_(height), output_format_(output_format) {
  // Basic validation for dimensions
  if (width_ <= 0 || height_ <= 0 ||
      (output_format_ == YuvProcessor::OutputFormat::YUYV && width_ % 2 != 0)) {
    throw std::runtime_error(
        "MjpegProcessor: Invalid width or height provided.");
  }

  try {
    InitDecoder();
    frame_camera_ = av_frame_alloc();
    av_packet_ = av_packet_alloc();
    frame_output_ = av_frame_alloc();
    if (!frame_camera_ || !av_packet_ || !frame_output_) {
      throw std::runtime_error(
          "MjpegProcessor: frame/packet allocation failed");
    }
  } catch (...) {
    Cleanup();
    throw;
  }

  AINFO << "MjpegProcessor initialized for " << width_ << "x" << height_;
}

void MjpegProcessor::InitDecoder() {
  const AVCodec* codec = avcodec_find_decoder(AV_CODEC_ID_MJPEG);
  if (!codec) {
    throw std::runtime_error("MjpegProcessor: MJPEG codec not found");
  }

  codec_ctx_ = avcodec_alloc_context3(codec);
  if (!codec_ctx_)
    throw std::runtime_error(
        "MjpegProcessor: Failed to allocate codec context");

  codec_ctx_->width = width_;
  codec_ctx_->height = height_;

  int ret = avcodec_open2(codec_ctx_, codec, nullptr);
  if (ret < 0) {
    MjpegUtil::ThrowFFmpegError(ret, "Could not open MJPEG decoder");
  }

  AINFO << "MJPEG decoder opened.";
}

MjpegProcessor::~MjpegProcessor() { Cleanup(); }

void MjpegProcessor::Cleanup() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (sws_ctx_) {
    sws_freeContext(sws_ctx_);
    sws_ctx_ = nullptr;
  }
  if (codec_ctx_) {
    avcodec_free_context(&codec_ctx_);
    codec_ctx_ = nullptr;  // Set to nullptr after freeing
  }
  if (frame_camera_) {
    av_frame_free(&frame_camera_);
    frame_camera_ = nullptr;  // Set to nullptr after freeing
  }
  if (frame_output_) {
    av_frame_free(&frame_output_);
  }
  if (av_packet_) {
    av_packet_free(&av_packet_);
    av_packet_ = nullptr;  // Set to nullptr after freeing
  }
}

bool MjpegProcessor::DecodePacket(const uint8_t* data, int size) {
  av_packet_unref(av_packet_);
  int ret = av_new_packet(av_packet_, size);
  if (ret < 0) {
    AERROR << "MjpegProcessor: packet allocation failed: "
           << MjpegUtil::GetFFmpegErrorString(ret);
    return false;
  }
  // FFmpeg requires zero padding after compressed input.
  std::memcpy(av_packet_->data, data, size);
  ret = avcodec_send_packet(codec_ctx_, av_packet_);
  av_packet_unref(av_packet_);

  if (ret < 0) {
    AERROR << "MjpegProcessor: Failed to send packet to decoder: "
           << MjpegUtil::GetFFmpegErrorString(ret);
    return false;
  }

  ret = avcodec_receive_frame(codec_ctx_, frame_camera_);
  if (ret < 0) {
    AERROR << "MjpegProcessor: Failed to receive frame from decoder: "
           << MjpegUtil::GetFFmpegErrorString(ret);
    return false;
  }
  return true;
}

bool MjpegProcessor::ConvertFrame(ImagePtr dest_pb) {
  if (frame_camera_->width != width_ || frame_camera_->height != height_) {
    AERROR << "MJPEG frame dimensions differ from the negotiated camera size.";
    return false;
  }
  const bool rgb_output = output_format_ == YuvProcessor::OutputFormat::RGB;
  const auto pixel_format = rgb_output ? AV_PIX_FMT_RGB24 : AV_PIX_FMT_YUYV422;
  sws_ctx_ = sws_getCachedContext(
      sws_ctx_, width_, height_,
      static_cast<AVPixelFormat>(frame_camera_->format), width_, height_,
      pixel_format, SWS_BILINEAR, nullptr, nullptr, nullptr);
  if (!sws_ctx_) {
    AERROR << "Failed to create MJPEG conversion context.";
    return false;
  }
  const size_t needed_size = size_t(width_) * height_ * (rgb_output ? 3 : 2);
  if (dest_pb->data().size() < needed_size) {
    AERROR << "MjpegProcessor: Destination buffer too small for output. "
              "Needed: "
           << needed_size << ", Actual: " << dest_pb->data().size();
    return false;
  }

  if (!frame_output_->data[0]) {
    frame_output_->format = pixel_format;
    frame_output_->width = width_;
    frame_output_->height = height_;
    const int ret = av_frame_get_buffer(frame_output_, 32);
    if (ret < 0) {
      AERROR << "MjpegProcessor: output frame allocation failed: "
             << MjpegUtil::GetFFmpegErrorString(ret);
      return false;
    }
  }

  // Perform the scaling/color space conversion
  const int rows = sws_scale(sws_ctx_, frame_camera_->data,
                             frame_camera_->linesize, 0, frame_camera_->height,
                             frame_output_->data, frame_output_->linesize);
  if (rows != height_) {
    AERROR << "Incomplete MJPEG conversion: " << rows << " rows.";
    return false;
  }
  // libswscale may use aligned SIMD stores; do not write into packed protobuf
  // storage directly, especially for small or unaligned image widths.
  auto* output_buffer = dest_pb->mutable_data();
  const int copied = av_image_copy_to_buffer(
      reinterpret_cast<uint8_t*>(output_buffer->data()), needed_size,
      frame_output_->data, frame_output_->linesize, pixel_format, width_,
      height_, 1);
  if (copied < 0) {
    AERROR << "MjpegProcessor: output copy failed: "
           << MjpegUtil::GetFFmpegErrorString(copied);
    return false;
  }
  return true;
}

bool MjpegProcessor::Process(const void* src, size_t len,
                             std::shared_ptr<Image> dest_pb) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!dest_pb) {
    AERROR << "MjpegProcessor: Destination Image pointer is null.";
    return false;
  }

  if (src == nullptr || len == 0 ||
      len > static_cast<size_t>(std::numeric_limits<int>::max())) {
    AERROR << "MjpegProcessor: Source MJPEG buffer is invalid.";
    return false;
  }

  // Check if the underlying data buffer in dest_pb is empty.
  // This check is important if Image's internal buffer might not be allocated.
  if (dest_pb->data().empty()) {
    AERROR
        << "MjpegProcessor: Destination Image's internal data buffer is empty.";
    return false;
  }

  if (!DecodePacket(static_cast<const uint8_t*>(src), static_cast<int>(len))) {
    AERROR << "MjpegProcessor: Failed to decode packet.";
    return false;
  }

  if (!ConvertFrame(dest_pb)) {
    AERROR << "MjpegProcessor: Failed to convert decoded frame.";
    return false;
  }

  return true;
}

}  // namespace camera
}  // namespace drivers
}  // namespace apollo
