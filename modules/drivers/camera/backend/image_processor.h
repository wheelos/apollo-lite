#pragma once

#include <memory>
#include <mutex>
#include <vector>

#include "modules/common_msgs/sensor_msgs/sensor_image.pb.h"

// Forward declarations for FFmpeg structs to avoid pulling in FFmpeg headers
// into the header file unnecessarily.
struct SwsContext;
struct AVCodecContext;
struct AVFrame;
struct AVPacket;

namespace apollo {
namespace drivers {
namespace camera {

/**
 * @brief Abstract base class for image processing.
 * Defines the common interface for different image processors.
 */
class ImageProcessor {
 public:
  using ImagePtr = std::shared_ptr<Image>;

  /**
   * @brief Default constructor for ImageProcessor.
   * Derived classes should implement the process method.
   */
  virtual ~ImageProcessor() = default;

  /**
   * @brief Processes raw image data and writes the result directly into a
   * protobuf Image message. This method aims for zero-copy by writing directly
   * to dest_pb's internal buffer.
   * @param src Pointer to the source raw image data.
   * @param len Length of the source raw image data.
   * @param dest_pb Shared pointer to the destination protobuf Image message.
   *                The message's data buffer must be pre-allocated to the
   * correct size.
   */
  virtual bool Process(const void* src, size_t len, ImagePtr dest_pb) = 0;
};

/**
 * @brief Processes YUYV (or UYVY) image data.
 * Supports converting to RGB or outputting as YUYV. Includes SIMD
 * optimizations.
 */
class YuvProcessor : public ImageProcessor {
 public:
  enum class OutputFormat { YUYV, RGB };
  enum class YuvFormat { YUYV, YVYU, UYVY, VYUY };

  /**
   * @brief Constructs a YuvProcessor.
   * @param format The desired output format (YUYV or RGB).
   * @param yuv_format The source YUV format (YUYV, YVYU, UYVY, or VYUY).
   * @param swap_uv If true, swap U and V channels (e.g., treat YUYV as YVYU).
   */
  YuvProcessor(OutputFormat format, YuvFormat yuv_format = YuvFormat::YUYV,
               bool swap_uv = false);

  /**
   * @brief Processes YUYV/UYVY data.
   * If output_format is YUYV, it's a direct copy. If RGB, performs color
   * conversion.
   */
  bool Process(const void* src, size_t len, ImagePtr dest_pb) override;

 private:
  /**
   * @brief Converts YUYV (YUV 4:2:2) formatted data to a RGB image.
   *
   * @param src       Pointer to input YUYV data, with length width * height * 2
   * bytes.
   * @param width     Image width in pixels.
   * @param height    Image height in pixels.
   * @param dst_rgb   Pointer to output buffer for RGB data, size should be
   * width * height * 3 bytes.
   */
  void ConvertYUYVToRGB(const uint8_t* src, int width, int height,
                        uint8_t* dst_rgb);

 private:
  OutputFormat output_format_;
  YuvFormat yuv_format_;  ///< The source YUV format variant
  bool swap_uv_;          ///< If true, swap U and V channels
  std::vector<uint8_t>
      yuyv_buffer_;  ///< Buffer for UYVY/VYUY to YUYV conversion
};

/**
 * @brief Processes MJPEG image data using FFmpeg.
 * Decodes MJPEG frames and converts them to RGB.
 */
class MjpegProcessor : public ImageProcessor {
 public:
  /**
   * @brief Constructs an MjpegProcessor.
   * @param width Expected width of the MJPEG frames.
   * @param height Expected height of the MJPEG frames.
   */
  MjpegProcessor(int width, int height);

  // Destructor for FFmpeg resource cleanup
  ~MjpegProcessor();

  /**
   * @brief Decodes an MJPEG frame and writes the RGB data to the protobuf Image
   * message.
   */
  bool Process(const void* src, size_t len, ImagePtr dest_pb) override;

 private:
  void InitDecoder();
  void InitConverter();
  void Cleanup();

  // Private helper for decoding. Uses zero-copy with external memory.
  bool DecodePacket(const uint8_t* data, int size);
  // Private helper for conversion
  bool ConvertToRGB(ImagePtr dest_pb);

  // FFmpeg related members
  SwsContext* sws_ctx_ = nullptr;
  AVCodecContext* codec_ctx_ = nullptr;
  AVFrame* frame_camera_ = nullptr;  // Raw frame from decoder (typically YUV)
  AVFrame* frame_rgb_ = nullptr;     // Frame for RGB output
  AVPacket* av_packet_ = nullptr;    // For holding input MJPEG data

  int width_;
  int height_;

  std::mutex mutex_;  // Protects FFmpeg context during process calls
};

}  // namespace camera
}  // namespace drivers
}  // namespace apollo
