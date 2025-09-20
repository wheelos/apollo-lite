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

#include <unistd.h>

#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "modules/common/util/string_util.h"
#include "modules/common/util/util.h"
#include "modules/drivers/hal/stream/stream.h"
#include "modules/drivers/hal/stream/tcp_stream.h"

namespace apollo {
namespace drivers {
namespace hal {

// Implements an NTRIP client stream built on top of a TcpStream.
// Handles NTRIP handshake (GET request, response parsing), data activity
// timeout, and auto-reconnection.
class NtripStream : public Stream {
 public:
  // Constructor takes NTRIP caster address, port, mountpoint, credentials, and
  // a timeout. Timeout_s specifies the data activity timeout and handshake
  // timeout. The underlying TcpStream will be configured with non-blocking
  // reads/writes (timeout_usec = 0) unless a different behavior is desired for
  // NTRIP data flow. Let's pass timeout_s converted to us for TcpStream? No,
  // the original passed 0 to TcpStream. Let's keep TcpStream non-blocking and
  // manage timeout in NtripStream.
  NtripStream(const std::string& address, uint16_t port,
              const std::string& mountpoint, const std::string& user,
              const std::string& passwd, uint32_t timeout_s);

  // Destructor disconnects the stream and cleans up the TcpStream.
  ~NtripStream() override;

  // Read data from the NTRIP stream (delegates to TcpStream).
  // Handles data activity timeout and triggers reconnection.
  // Thread-safe.
  // Returns bytes read (> 0).
  // Returns 0 on timeout or non-blocking with no data.
  // Throws std::runtime_error on fatal errors or if device removed/connection
  // lost.
  size_t read(uint8_t* buffer, size_t max_length) override;

  // Write data to the NTRIP stream (delegates to TcpStream after handshake).
  // Note: Standard NTRIP clients send NMEA/GGA commands after handshake, NOT
  // GET headers. This method is intended for sending client commands like NMEA.
  // Thread-safe.
  // Returns bytes written (<= length).
  // Returns 0 if not connected or on non-blocking EAGAIN/timeout.
  // Throws std::runtime_error on fatal write errors.
  size_t write(const uint8_t* data, size_t length) override;

  // Connect: Establishes the TCP connection and performs the NTRIP handshake.
  // Returns true on success. Returns false on failure (including handshake
  // failure).
  bool Connect() override;

  // Disconnect: Disconnects the underlying TCP stream.
  // Returns true on success.
  bool Disconnect() override;

  int get_last_error_code() {
    std::lock_guard<std::mutex> lock(internal_mutex_);
    return tcp_stream_ ? tcp_stream_->get_last_error_code() : 0;
  }

 private:
  // Attempts to disconnect and reconnect the NTRIP stream.
  // Called internally on timeout or read/write errors.
  // Assumes the caller holds the internal_mutex_ lock.
  void Reconnect();

  // Reads the HTTP response header from the TCP stream until "\r\n\r\n" or
  // timeout. Throws std::runtime_error on read error or timeout during
  // handshake. Returns the received header string.
  std::string read_http_header_locked(uint32_t timeout_s);

 private:
  bool is_login_ = false;  // Indicates if the NTRIP handshake was successful
  const std::string mountpoint_;
  const std::string address_;
  const uint16_t port_;
  const std::string write_data_prefix_;
  const std::string login_data_;

  uint32_t timeout_s_ = 60;     // Data activity timeout in seconds
  double data_active_s_ = 0.0;  // Timestamp (seconds) of last received data

  std::unique_ptr<TcpStream> tcp_stream_;  // Underlying TCP stream
  std::mutex internal_mutex_;
};

}  // namespace hal
}  // namespace drivers
}  // namespace apollo
