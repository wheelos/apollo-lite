/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include <atomic>
#include <cstdio>
#include <map>
#include <memory>
#include <string>

#include "wheelos_msgs/sensor_msgs/sensor_image.pb.h"

#include "cyber/cyber.h"
#include "modules/drivers/video/frame_assembler.h"

namespace apollo {
namespace drivers {
namespace video {

static const int SOCKET_TIMEOUT = -2;
static const int RECEIVE_FAIL = -3;
static const int POLL_TIMEOUT = 1000;  // one second (in msec)

/** @brief Live Velodyne input from socket. */
class SocketInput {
 public:
  SocketInput();
  virtual ~SocketInput();
  bool Init(uint32_t port);
  void Stop() { running_.store(false); }
  int GetFramePacket(std::shared_ptr<CompressedImage> h265);

 private:
  int sockfd_;
  int port_;
  std::atomic<bool> running_{false};
  FrameAssembler assembler_;
  bool InputAvailable(int timeout);
};

}  // namespace video
}  // namespace drivers
}  // namespace apollo
