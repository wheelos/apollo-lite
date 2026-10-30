/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.

 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at

 * http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/drivers/video/socket_input.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <memory>

#include "cyber/cyber.h"

namespace apollo {
namespace drivers {
namespace video {

////////////////////////////////////////////////////////////////////////
// InputSocket class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
 *
 *  @param private_nh private node handle for driver
 *  @param udpport UDP port number to connect
 */
SocketInput::SocketInput() : sockfd_(-1), port_(0) {}

/** @brief destructor */
SocketInput::~SocketInput() { (void)close(sockfd_); }

bool SocketInput::Init(uint32_t port) {
  running_.store(false);
  if (sockfd_ != -1) {
    (void)close(sockfd_);
    sockfd_ = -1;
  }
  if (port > 65535) {
    AERROR << "Invalid camera UDP port: " << port;
    return false;
  }

  sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd_ < 0) {
    AERROR << "Failed to create socket fd.";
    return false;
  }

  // Connect to camera UDP port
  AINFO << "Opening UDP socket on port: " << port;
  port_ = port;
  sockaddr_in my_addr;
  memset(&my_addr, '\0', sizeof(my_addr));
  my_addr.sin_family = AF_INET;
  my_addr.sin_port = htons(static_cast<uint16_t>(port));
  my_addr.sin_addr.s_addr = INADDR_ANY;
  if (bind(sockfd_, reinterpret_cast<sockaddr *>(&my_addr),
           sizeof(sockaddr_in)) < 0) {
    AERROR << "Failed to bind socket on local address.";
    (void)close(sockfd_);
    sockfd_ = -1;
    return false;
  }

  if (fcntl(sockfd_, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
    AERROR << "Failed to enable non-blocking I/O.";
    (void)close(sockfd_);
    sockfd_ = -1;
    return false;
  }

  const int rbuf = 4 * 1024 * 1024;
  if (setsockopt(sockfd_, SOL_SOCKET, SO_RCVBUF, &rbuf, sizeof(int)) < 0) {
    AERROR << "Failed to enable socket receive buffer.";
  }

  int enable = 1;
  if (setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0) {
    AERROR << "Failed to enable socket reuseable address.";
  }

  AINFO << "Camera socket fd: " << sockfd_ << ", port: " << port_;
  running_.store(true);
  return true;
}

/** @brief Get one camera packet. */
int SocketInput::GetFramePacket(std::shared_ptr<CompressedImage> h265Pb) {
  std::array<uint8_t, H265_PDU_SIZE> packet;
  assembler_.Reset();

  while (running_.load()) {
    if (!InputAvailable(POLL_TIMEOUT)) {
      return SOCKET_TIMEOUT;
    }
    // Receive packets that should now be available from the socket using
    // a blocking read.
    const ssize_t pdu_len = recvfrom(sockfd_, packet.data(), packet.size(),
                                     MSG_TRUNC, nullptr, nullptr);
    if (pdu_len < 0) {
      if (errno == EWOULDBLOCK || errno == EINTR) {
        continue;
      }
      AERROR << "Failed to receive package from port: " << port_;
      return RECEIVE_FAIL;
    }

    if (assembler_.AddPacket(packet.data(), static_cast<size_t>(pdu_len),
                             h265Pb.get())) {
      return 0;
    }
  }
  return RECEIVE_FAIL;
}

bool SocketInput::InputAvailable(int timeout) {
  struct pollfd fds[1];
  fds[0].fd = sockfd_;
  fds[0].events = POLLIN;
  // Unfortunately, the Linux kernel recvfrom() implementation
  // uses a non-interruptible sleep() when waiting for data,
  // which would cause this method to hang if the device is not
  // providing data.  We poll() the device first to make sure
  // the recvfrom() will not block.
  // Note that, however, there is a known Linux kernel bug:
  // Under Linux, select() may report a socket file descriptor
  // as "ready for reading", while nevertheless a subsequent
  // read blocks.  This could for example happen when data has
  // arrived but upon examination has wrong checksum and is
  // discarded.  There may be other circumstances in which a
  // file descriptor is spuriously reported as ready.  Thus it
  // may be safer to use O_NONBLOCK on sockets that should not
  // block.

  // poll() until input available
  do {
    int ret = poll(fds, 1, timeout);
    if (ret < 0) {
      if (errno != EINTR) {
        AERROR << "H265 camera port: " << port_
               << "poll() error: " << strerror(errno);
      }
      return false;
    }

    // Timeout
    if (ret == 0) {
      return false;
    }

    if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) ||
        (fds[0].revents & POLLNVAL)) {
      AERROR << "Error! poll failed on H265 camera port: " << port_;
      return false;
    }
  } while (!(fds[0].revents & POLLIN));

  return true;
}

}  // namespace video
}  // namespace drivers
}  // namespace apollo
