/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#pragma once

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/time.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>

#include "modules/drivers/hal/stream/stream.h"

namespace apollo {
namespace drivers {
namespace hal {

// Enum for common serial port configurations
enum class SerialByteSize { B5, B6, B7, B8 };
enum class SerialParity { None, Even, Odd };
enum class SerialStopBits { One, Two };
enum class SerialFlowControl { None, XONXOFF, RTSCTS };

class SerialStream : public Stream {
  using baud_rate_t = speed_t;

 public:
  // Constructor takes device name, baud rate constant, timeout, and other port
  // settings. timeout_usec = 0 for non-blocking reads/writes (will return
  // immediately if no data/buffer). timeout_usec > 0 implements a timeout for
  // read/write waiting using pselect.
  SerialStream(const std::string& device_name, baud_rate_t baud_rate,
               SerialByteSize bytesize, SerialParity parity,
               SerialStopBits stopbits, SerialFlowControl flowcontrol,
               uint32_t timeout_usec);

  // Destructor closes the serial port.
  ~SerialStream() override;

  // Connect: Opens and configures the serial port.
  // Returns true if port is successfully opened/already open.
  // Returns false on opening/configuration failure.
  bool Connect() override;

  // Disconnect: Closes the serial port.
  // Returns true if port is successfully closed/already closed.
  bool Disconnect() override;

  // Read data from the serial port.
  // If timeout_usec > 0, waits up to timeout_usec for data.
  // If timeout_usec == 0, returns immediately (non-blocking).
  // Returns number of bytes read on success (> 0).
  // Returns 0 on timeout or non-blocking with no data (EAGAIN/EWOULDBLOCK).
  // Returns 0 if device not open and auto_reconnect fails or is disabled.
  // Throws std::runtime_error for fatal read errors (other than
  // EAGAIN/EWOULDBLOCK/EINTR) or if remote closed/device removed detected.
  size_t read(uint8_t* buffer, size_t max_length) override;

  // Write data to the serial port.
  // If timeout_usec > 0, waits up to timeout_usec for write buffer space if
  // needed. If timeout_usec == 0, returns immediately (non-blocking) if cannot
  // write. Returns number of bytes written on success (<= length). Returns 0 if
  // device not open and auto_reconnect fails or is disabled. Throws
  // std::runtime_error for fatal write errors (other than
  // EAGAIN/EWOULDBLOCK/EINTR).
  size_t write(const uint8_t* data, size_t length) override;

 private:
  // Default constructor deleted to prevent instantiation without parameters.
  SerialStream() = delete;

  // Helper function to open the serial port file descriptor.
  // Throws std::runtime_error on failure.
  void open();

  // Helper function to close the serial port. Safe to call multiple times.
  void close();

  // Helper function to configure the serial port using termios.
  // Assumes fd_ is valid and the port is open.
  // Throws std::runtime_error on failure.
  void configure_port();

  // Waits for the file descriptor to become readable.
  // Uses pselect with the given timeout.
  // Returns true if readable, false on timeout or error.
  // Sets last_errno_ on select error or timeout.
  bool wait_readable(uint32_t timeout_us);

  // Waits for the file descriptor to become writable.
  // Uses pselect with the given timeout.
  // Returns true if writable, false on timeout or error.
  // Sets last_errno_ on select error or timeout.
  bool wait_writable(uint32_t timeout_us);

  // Heuristic check to detect if the device has been removed.
  // Attempts a zero-byte write. If certain errors occur (EBADF, EIO),
  // logs, disconnects, and attempts to reconnect.
  // Returns true if device likely removed and disconnected.
  // Returns false otherwise.
  bool check_remove();

 private:
  // Stored configuration parameters
  std::string device_name_;
  baud_rate_t baud_rate_;
  SerialByteSize bytesize_;
  SerialParity parity_;
  SerialStopBits stopbits_;
  SerialFlowControl flowcontrol_;
  uint32_t timeout_usec_;

  // Calculated time to transmit one byte (used potentially for write timeouts)
  // Correctly calculated based on actual baud rate and settings.
  uint32_t byte_time_us_ = 0;

  // File descriptor for the serial port. -1 indicates not open.
  int fd_ = -1;

  // Indicates if the port is currently open. Kept for clarity alongside fd_.
  bool is_open_ = false;
};

}  // namespace hal
}  // namespace drivers
}  // namespace apollo
