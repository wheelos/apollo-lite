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

class UdpStream : public Stream {
  using ipv4_addr_t = uint32_t;
  using port_t = uint16_t;

 public:
  UdpStream(const std::string& address, uint16_t port, uint32_t timeout_usec);
  ~UdpStream() override;

  // Connect: For UDP, this conceptually means "open and configure the socket".
  // It does NOT establish a connection like TCP.
  // Returns true if socket is successfully opened/already open.
  virtual bool Connect() override;
  virtual bool Disconnect() override;
  virtual size_t read(uint8_t* buffer, size_t max_length) override;
  virtual size_t write(const uint8_t* data, size_t length) override;

  int get_last_error_code() const { return last_errno_; }
 private:
  // Default constructor deleted to prevent instantiation without parameters.
  UdpStream() = delete;

  // Helper function to create and configure the socket.
  // Throws std::runtime_error on failure.
  void open();

  // Helper function to close the socket. Safe to call multiple times.
  void close();

 private:
  ipv4_addr_t peer_addr_ = 0;
  port_t peer_port_ = 0;

  int sockfd_ = -1;
  uint32_t timeout_usec_ = 0;
};

}  // namespace hal
}  // namespace drivers
}  // namespace apollo
