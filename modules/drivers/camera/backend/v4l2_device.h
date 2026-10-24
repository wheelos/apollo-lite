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

class CameraCaptureDevice {
 public:
  using ConfPtr = std::shared_ptr<config::Config>;

  virtual ~CameraCaptureDevice() = default;
  virtual void Configure(uint32_t width, uint32_t height, uint32_t pixel_format,
                         int frame_rate) = 0;
  virtual bool SetParameter(uint32_t id, int32_t value) = 0;
  virtual void StartStreaming() = 0;
  virtual void StopStreaming() noexcept = 0;
  virtual int WaitForData(long sec, long usec) = 0;
  virtual V4L2Buffer DequeueBuffer() = 0;
  virtual void QueueBuffer(uint32_t index) = 0;
  virtual uint32_t GetWidth() const = 0;
  virtual uint32_t GetHeight() const = 0;
  virtual uint32_t GetPixelFormat() const = 0;
};

class V4L2Device : public CameraCaptureDevice {
 public:
  using ConfPtr = std::shared_ptr<config::Config>;

  explicit V4L2Device(ConfPtr config);
  ~V4L2Device() noexcept;

  // Disable copy and assignment
  V4L2Device(const V4L2Device&) = delete;
  V4L2Device& operator=(const V4L2Device&) = delete;

  // Configure the device, parameters are const input
  void Configure(uint32_t width, uint32_t height, uint32_t pixel_format,
                 int frame_rate) override;

  // Set standard camera parameters
  bool SetParameter(uint32_t id, int32_t value) override;

  // Stream control
  void StartStreaming() override;
  void StopStreaming() noexcept override;

  // Data capture
  // Return value semantics:
  //   > 0: data ready
  //   = 0: timeout
  //   = -1: error
  //   = -2: interrupted by signal (EINTR)
  int WaitForData(long sec, long usec) override;
  V4L2Buffer DequeueBuffer() override;
  void QueueBuffer(uint32_t index) override;

  // Provide methods to query the final device state
  uint32_t GetWidth() const override { return width_; }
  uint32_t GetHeight() const override { return height_; }
  uint32_t GetPixelFormat() const override { return pixel_format_; }
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
