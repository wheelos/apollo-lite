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

#include "modules/drivers/hal/stream/stream.h"

namespace apollo {
namespace drivers {
namespace hal {

class TcpStream : public Stream {
  using ipv4_addr_t = uint32_t;
  using port_t = uint16_t;

 public:
  TcpStream(const std::string& address, uint16_t port, uint32_t timeout_usec,
            bool auto_reconnect = true);
  ~TcpStream() override;

  virtual bool Connect() override;
  virtual bool Disconnect() override;
  virtual size_t read(uint8_t* buffer, size_t max_length) override;
  virtual size_t write(const uint8_t* data, size_t length) override;

  bool Readable(uint32_t timeout_us);

  // Attempts to disconnect and reconnect if auto_reconnect is enabled.
  // Returns true on successful reconnection.
  bool Reconnect();

 private:
  TcpStream() = delete;
  // Helper function to create the socket file descriptor.
  // Throws std::runtime_error on failure.
  void open();

  // Helper function to close the socket. Safe to call multiple times.
  void close();

  // Helper function to configure socket options (blocking/non-blocking,
  // timeouts, TCP_NODELAY).
  bool InitSocket();

 private:
  ipv4_addr_t peer_addr_ = 0;
  port_t peer_port_ = 0;

  // File descriptor for the socket. -1 indicates not open.
  int sockfd_ = -1;

  // Configured timeout in microseconds. 0 indicates non-blocking.
  uint32_t timeout_usec_ = 0;

  // Flag to enable automatic reconnection on read/write failures.
  bool auto_reconnect_ = false;
};

}  // namespace hal
}  // namespace drivers
}  // namespace apollo
