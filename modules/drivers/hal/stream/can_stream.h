// Copyright 2025 WheelOS. All Rights Reserved.
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

//  Created Date: 2025-03-21
//  Author: daohu527

#pragma once

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <stdexcept>
#include <string>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "modules/drivers/hal/stream/stream.h"

namespace apollo {
namespace drivers {
namespace hal {

// Implements a Stream interface for CAN bus communication using Linux
// SocketCAN. Reads and writes operate on raw struct can_frame or struct
// canfd_frame structures depending on the enable_can_fd flag passed in the
// constructor. max_length/length must be a multiple of the chosen frame size.
class CanStream : public Stream {
 public:
  // Constructor takes CAN interface name (address), port (unused for basic
  // SocketCAN), timeout, and a flag to enable CAN FD support. Throws
  // std::invalid_argument if interface name is empty or too long.
  CanStream(const std::string &address, uint32_t timeout_usec,
            bool enable_can_fd = false);

  // Destructor closes the CAN socket.
  ~CanStream() override;

  // Connect: Creates, binds, and configures the CAN socket.
  // Returns true if socket is successfully opened/already open.
  // Returns false on failure.
  bool Connect() override;

  // Disconnect: Closes the CAN socket.
  // Returns true if socket is successfully closed/already closed.
  bool Disconnect() override;

  // Reads CAN frames into the buffer.
  // buffer must be large enough to hold at least one frame (sizeof(struct
  // can_frame) or sizeof(struct canfd_frame)). max_length should be a multiple
  // of the chosen frame size. Returns number of bytes read (multiple of frame
  // size, >= 0). Returns 0 on timeout or non-blocking with no data
  // (EAGAIN/EWOULDBLOCK). Throws std::runtime_error on fatal read errors or
  // connection loss.
  size_t read(uint8_t *buffer, size_t max_length) override;

  // Writes CAN frames from the data buffer.
  // data buffer must contain one or more frame structures.
  // length must be a multiple of the chosen frame size.
  // Returns number of bytes written (multiple of frame size, >= 0).
  // Returns 0 if not connected or on non-blocking EAGAIN/timeout.
  // Throws std::runtime_error on fatal write errors.
  size_t write(const uint8_t *data, size_t length) override;

 private:
  // Helper function to create, bind, and configure the CAN socket.
  // Throws std::runtime_error on failure.
  void open();

  // Helper function to close the CAN socket. Safe to call multiple times.
  void close();

  // Determines the size of the frame structure based on enable_can_fd_.
  size_t get_frame_size() const {
    return enable_can_fd_ ? sizeof(struct canfd_frame)
                          : sizeof(struct can_frame);
  }

 private:
  // CAN interface name (e.g., "can0")
  std::string interface_name_;

  // Read/Write timeout in microseconds. 0 indicates non-blocking.
  uint32_t timeout_usec_ = 0;

  // File descriptor for the CAN socket. -1 indicates not open.
  int fd_ = -1;

  // Flag indicating if CAN FD support is enabled for this instance.
  bool enable_can_fd_ = false;
};

}  // namespace hal
}  // namespace drivers
}  // namespace apollo
