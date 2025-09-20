#pragma once

#include <memory>
#include <string>
#include <vector>

#include <linux/videodev2.h>

#include "modules/drivers/camera/proto/config.pb.h"

namespace apollo {
namespace drivers {
namespace camera {

// Define a public Buffer struct for returning data to upper layers
struct V4L2Buffer {
  void* start = nullptr;
  size_t length = 0;
  uint32_t index = 0;
  uint32_t flags = 0;
  struct timeval timestamp {};
};

class V4L2Device {
 public:
  using ConfPtr = std::shared_ptr<config::Config>;

  explicit V4L2Device(ConfPtr config);
  ~V4L2Device() noexcept;

  // Disable copy and assignment
  V4L2Device(const V4L2Device&) = delete;
  V4L2Device& operator=(const V4L2Device&) = delete;

  // Configure the device, parameters are const input
  void Configure(uint32_t width, uint32_t height, uint32_t pixel_format,
                 int frame_rate);

  // Set standard camera parameters
  bool SetParameter(uint32_t id, int32_t value);

  // Stream control
  void StartStreaming();
  void StopStreaming() noexcept;

  // Data capture
  int WaitForData(long sec, long usec);
  V4L2Buffer DequeueBuffer();
  void QueueBuffer(uint32_t index);

  // Provide methods to query the final device state
  uint32_t GetWidth() const { return width_; }
  uint32_t GetHeight() const { return height_; }
  uint32_t GetPixelFormat() const { return pixel_format_; }
  int GetFd() const { return fd_; }

 private:
  void OpenDevice();
  void SetFrameRate(int frame_rate);
  void InitBuffers();
  void UninitBuffers() noexcept;
  int xioctl(int request, void* arg);

 private:
  struct V4L2BufferInternal {
    void* start = nullptr;
    size_t length = 0;
  };
  std::vector<V4L2BufferInternal> buffers_;

  ConfPtr config_;
  std::string device_path_;
  config::IOMethod io_method_;

  int fd_;
  bool is_streaming_;

  // Store actual device parameters determined by the driver
  uint32_t width_ = 0;
  uint32_t height_ = 0;
  uint32_t pixel_format_ = 0;
  uint32_t buffer_size_ = 0;
};

}  // namespace camera
}  // namespace drivers
}  // namespace apollo
