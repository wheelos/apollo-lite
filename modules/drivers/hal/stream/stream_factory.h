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

#include "modules/drivers/hal/stream/can_stream.h"
#include "modules/drivers/hal/stream/ntrip_stream.h"
#include "modules/drivers/hal/stream/serial_stream.h"
#include "modules/drivers/hal/stream/stream.h"
#include "modules/drivers/hal/stream/tcp_stream.h"
#include "modules/drivers/hal/stream/udp_stream.h"

namespace apollo {
namespace drivers {
namespace hal {

class StreamFactory {
 public:
  using StreamPtr = std::shared_ptr<Stream>;

  static StreamPtr CreateTcp(const std::string &address, uint16_t port,
                             uint32_t timeout_usec = 1000000);

  static StreamPtr CreateUdp(const std::string &address, uint16_t port,
                             uint32_t timeout_usec = 1000000);

  // Currently the following baud rates are supported:
  //  9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600.
  static StreamPtr CreateSerial(
      const std::string &device_name, uint32_t baud_rate,
      SerialByteSize bytesize = SerialByteSize::B8,
      SerialParity parity = SerialParity::None,
      SerialStopBits stopbits = SerialStopBits::One,
      SerialFlowControl flowcontrol = SerialFlowControl::None,
      uint32_t timeout_usec = 0);

  static StreamPtr CreateNtrip(const std::string &address, uint16_t port,
                               const std::string &mountpoint,
                               const std::string &user,
                               const std::string &passwd,
                               uint32_t timeout_s = 30);

  static StreamPtr CreateCan(const std::string &address,
                             uint32_t timeout_usec = 30 * 1000000u,
                             bool enable_can_fd = false);
};

}  // namespace hal
}  // namespace drivers
}  // namespace apollo
