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

#pragma once

#include <sys/time.h>

#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "modules/drivers/camera/proto/config.pb.h"
#include "wheelos_msgs/sensor_msgs/sensor_image.pb.h"

#include "modules/drivers/camera/backend/image_processor.h"
#include "modules/drivers/camera/backend/v4l2_device.h"

namespace apollo {
namespace drivers {
namespace camera {

/**
 * @brief Manages a V4L2 camera device, including configuration, streaming,
 * image processing, and error handling (reconnection).
 */
class CameraDevice {
 public:
  using ConfPtr = std::shared_ptr<apollo::drivers::camera::config::Config>;
  using DeviceFactory =
      std::function<std::unique_ptr<CameraCaptureDevice>(ConfPtr)>;
  /**
   * @brief Enumeration for camera device states.
   */
  enum class State {
    UNINITIALIZED,  ///< Device is not initialized or failed initialization
    INITIALIZED,    ///< Device is initialized and ready for streaming
    RECONNECTING    ///< Device is attempting to reconnect after an error
  };

  /**
   * @brief Constructs a CameraDevice instance.
   * Attempts to initialize the camera upon construction.
   * @param config Shared pointer to the camera configuration Protobuf message.
   */
  explicit CameraDevice(const ConfPtr& config,
                        DeviceFactory device_factory = DeviceFactory());

  /**
   * @brief Destroys the CameraDevice instance.
   * Ensures proper cleanup of V4L2 device and image processor resources.
   */
  ~CameraDevice();

  /**
   * @brief Polls the camera for a new image frame.
   * This method will wait for data, dequeue a buffer, process it,
   * and enqueue the buffer back. It also handles timestamps and frame drops.
   * @param pb_image Shared pointer to the protobuf Image message to fill.
   *                 The caller is responsible for ensuring pb_image points to a
   * valid Image object with pre-allocated data buffer of sufficient size.
   * @return True if a new image was successfully processed and filled into
   * pb_image, false otherwise.
   */
  bool Poll(std::shared_ptr<Image> pb_image);

  /**
   * @brief Checks if the camera is currently in an initialized state and
   * capturing.
   * @return True if the camera is initialized, false otherwise.
   */
  bool IsCapturing() const;

  // Interrupts reconnect backoff; an in-flight capture wait remains bounded
  // by CameraCaptureDevice::WaitForData's timeout.
  void Stop();

 private:
  // Disallow copy constructor and assignment operator to prevent accidental
  // copying and ensure proper resource management (unique_ptr, file
  // descriptors).
  CameraDevice(const CameraDevice&) = delete;
  CameraDevice& operator=(const CameraDevice&) = delete;

  /**
   * @brief Attempts to initialize the camera device and its components.
   * @return True if initialization is successful, false otherwise.
   */
  bool Init();

  /**
   * @brief Attempts to reconnect the camera device after a failure.
   * Implements an exponential backoff strategy for retries.
   */
  void Reconnect();

  /**
   * @brief Sets various V4L2 camera parameters based on the configuration.
   */
  void SetCameraParameters();

  /**
   * @brief Retrieves a monotonic timestamp for the given V4L2 buffer.
   * Prefers hardware-provided monotonic timestamp if available, falls back to
   * system monotonic time.
   * @param v4l2_buf The V4L2 buffer containing timestamp information.
   * @return Monotonic timestamp in nanoseconds.
   */
  uint64_t GetMonotonicTimestamp(const V4L2Buffer& v4l2_buf);

 private:
  ConfPtr config_;  // Camera configuration
  DeviceFactory device_factory_;
  std::unique_ptr<CameraCaptureDevice> device_;
  // V4L2 device handler
  std::unique_ptr<ImageProcessor> processor_;  // Image data processor

  State state_ = State::UNINITIALIZED;
  uint64_t last_timestamp_ns_ = 0;
  uint64_t frame_drop_interval_ns_ = 0;
  uint64_t frame_warning_interval_ns_ = 0;

  uint32_t reconnect_attempts_ = 0;  // Counter for reconnection attempts
  uint32_t select_timeout_count_ =
      0;  // Counter for consecutive select() timeouts
  std::atomic<bool> stopping_{false};
  std::mutex stop_mutex_;
  std::condition_variable stop_condition_;
};

}  // namespace camera
}  // namespace drivers
}  // namespace apollo
