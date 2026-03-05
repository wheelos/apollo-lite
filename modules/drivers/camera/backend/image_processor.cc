#include "modules/drivers/camera/backend/image_processor.h"

#include <algorithm>
#include <cstdint>
#include <iomanip>
#include <mutex>
#include <stdexcept>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "cyber/common/log.h"

// FFmpeg headers must always be wrapped in extern "C" in C++
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/error.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

#include "libavcodec/version.h"
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

  if (opencv_yuyv_temp_buffer_.size() < yuyv_len) {
    opencv_yuyv_temp_buffer_.resize(yuyv_len);
  }
  std::memcpy(opencv_yuyv_temp_buffer_.data(), convert_src, yuyv_len);

  cv::Mat yuyv_mat(height, width, CV_8UC2, opencv_yuyv_temp_buffer_.data());
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

void YuvProcessor::Process(const void* src, size_t len,
                           std::shared_ptr<Image> dest_pb) {
  if (src == nullptr || len == 0) {
    AERROR << "YuvProcessor: received empty or null image data.";
    return;
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

  int width = dest_pb->width();
  int height = dest_pb->height();
  if (width <= 0 || height <= 0) {
    AERROR << "YuvProcessor: invalid image size " << width << "x" << height;
    throw std::invalid_argument("Invalid width/height");
  }

  if (output_format_ == OutputFormat::YUYV) {
    // Direct copy to protobuf buffer for YUYV output
    dest_pb->set_data(yuv_data_ptr, len);
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
    throw std::runtime_error("Unsupported output format");
  }
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

MjpegProcessor::MjpegProcessor(int width, int height)
    : width_(width), height_(height) {
  // Basic validation for dimensions
  if (width_ <= 0 || height_ <= 0) {
    throw std::runtime_error(
        "MjpegProcessor: Invalid width or height provided.");
  }

  InitDecoder();

  frame_camera_ = av_frame_alloc();
  av_packet_ = av_packet_alloc();
  if (!frame_camera_ || !av_packet_) {
    throw std::runtime_error("MjpegProcessor: AVFrame/AVPacket alloc failed");
  }

  // Must be called after InitDecoder to get actual pix_fmt
  InitConverter();

  frame_rgb_ = av_frame_alloc();
  if (!frame_rgb_) {
    throw std::runtime_error("MjpegProcessor: allocate RGB wrapper failed");
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

  AINFO << "Decoder opened, src pix_fmt="
        << av_get_pix_fmt_name(codec_ctx_->pix_fmt);
}

void MjpegProcessor::InitConverter() {
  sws_ctx_ =
      sws_getContext(width_, height_, codec_ctx_->pix_fmt, width_, height_,
                     AV_PIX_FMT_RGB24, SWS_BILINEAR, nullptr, nullptr, nullptr);
  if (!sws_ctx_) {
    throw std::runtime_error("Failed to create SwsContext");
  }
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
  if (frame_rgb_) {
    av_frame_free(&frame_rgb_);
    frame_rgb_ = nullptr;  // Set to nullptr after freeing
  }
  if (av_packet_) {
    av_packet_free(&av_packet_);
    av_packet_ = nullptr;  // Set to nullptr after freeing
  }
}

bool MjpegProcessor::DecodePacket(const uint8_t* data, int size) {
  av_packet_unref(av_packet_);
  av_packet_->data = const_cast<uint8_t*>(data);
  av_packet_->size = size;

  int ret = avcodec_send_packet(codec_ctx_, av_packet_);
  // 4. IMPORTANT: Immediately unref the packet AFTER sending it.
  //    This is crucial for zero-copy. It tells FFmpeg to release any internal
  //    references it might have had to the *external* data buffer.
  //    Since MJPEG decoding is usually frame-by-frame and immediate,
  //    the external buffer is no longer needed by FFmpeg after this point.
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

bool MjpegProcessor::ConvertToRGB(ImagePtr dest_pb) {
  // Make sure dest_pb's buffer is correctly sized before filling.
  // Image::mutable_data() should return a buffer that can be resized.
  const size_t needed_size = size_t(width_) * height_ * 3;
  if (dest_pb->data().size() < needed_size) {
    // Optionally resize the buffer if it's too small. This makes it more
    // robust.
    AERROR << "MjpegProcessor: Destination buffer too small for RGB output. "
              "Needed: "
           << needed_size << ", Actual: " << dest_pb->data().size();
    return false;
  }

  // Prepare frame_rgb_ to point to the destination protobuf buffer
  // av_image_fill_arrays is a utility to set data pointers and linesizes for an
  // AVFrame. We are essentially making frame_rgb_ a "wrapper" around dest_pb's
  // data.
  int ret = av_image_fill_arrays(
      frame_rgb_->data, frame_rgb_->linesize,
      reinterpret_cast<uint8_t*>(
          dest_pb->mutable_data()),           // Get raw pointer to vector data
      AV_PIX_FMT_RGB24, width_, height_, 1);  // Align by 1

  if (ret < 0) {
    AERROR << "MjpegProcessor: Failed to fill RGB frame arrays: "
           << MjpegUtil::GetFFmpegErrorString(ret);
    return false;
  }

  // Perform the scaling/color space conversion
  sws_scale(sws_ctx_, (const uint8_t* const*)frame_camera_->data,
            frame_camera_->linesize, 0, frame_camera_->height, frame_rgb_->data,
            frame_rgb_->linesize);
  return true;
}

void MjpegProcessor::Process(const void* src, size_t len,
                             std::shared_ptr<Image> dest_pb) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!dest_pb) {
    AERROR << "MjpegProcessor: Destination Image pointer is null.";
    return;  // Early exit
  }

  // Check if the underlying data buffer in dest_pb is empty.
  // This check is important if Image's internal buffer might not be allocated.
  if (dest_pb->data().empty()) {
    AERROR
        << "MjpegProcessor: Destination Image's internal data buffer is empty.";
    return;  // Early exit
  }

  // Decode the MJPEG packet from the external (Protobuf) memory
  if (!DecodePacket(static_cast<const uint8_t*>(src), static_cast<int>(len))) {
    AERROR << "MjpegProcessor: Failed to decode packet.";
    return;
  }

  // Convert the decoded frame (frame_camera_) to RGB and store in dest_pb
  if (!ConvertToRGB(dest_pb)) {
    AERROR << "MjpegProcessor: Failed to convert to RGB.";
  }
}

}  // namespace camera
}  // namespace drivers
}  // namespace apollo
