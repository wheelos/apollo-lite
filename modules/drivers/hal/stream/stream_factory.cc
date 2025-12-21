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

#include "modules/drivers/hal/stream/stream_factory.h"

#include <termios.h>

namespace apollo {
namespace drivers {
namespace hal {

speed_t get_serial_baudrate(uint32_t rate) {
  switch (rate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 921600:
      return B921600;
    default:
      return 0;
  }
}

StreamFactory::StreamPtr StreamFactory::CreateCan(const std::string& address,
                                                  uint32_t timeout_usec,
                                                  bool enable_can_fd) {
  return std::make_shared<CanStream>(address, timeout_usec, enable_can_fd);
}

StreamFactory::StreamPtr StreamFactory::CreateNtrip(
    const std::string& address, uint16_t port, const std::string& mountpoint,
    const std::string& user, const std::string& passwd, uint32_t timeout_s) {
  return std::make_shared<NtripStream>(address, port, mountpoint, user, passwd,
                                       timeout_s);
}

StreamFactory::StreamPtr StreamFactory::CreateSerial(
    const std::string& device_name, uint32_t baud_rate, SerialByteSize bytesize,
    SerialParity parity, SerialStopBits stopbits, SerialFlowControl flowcontrol,
    uint32_t timeout_usec) {
  speed_t baud = get_serial_baudrate(baud_rate);
  return baud == 0 ? nullptr
                   : std::make_shared<SerialStream>(device_name, baud, bytesize,
                                                    parity, stopbits,
                                                    flowcontrol, timeout_usec);
}

StreamFactory::StreamPtr StreamFactory::CreateTcp(const std::string& address,
                                                  uint16_t port,
                                                  uint32_t timeout_usec) {
  return std::make_shared<TcpStream>(address, port, timeout_usec);
}

StreamFactory::StreamPtr StreamFactory::CreateUdp(const std::string& address,
                                                  uint16_t port,
                                                  uint32_t timeout_usec) {
  return std::make_shared<UdpStream>(address, port, timeout_usec);
}

}  // namespace hal
}  // namespace drivers
}  // namespace apollo
