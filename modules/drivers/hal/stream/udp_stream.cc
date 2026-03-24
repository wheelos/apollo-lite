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

#include "modules/drivers/hal/stream/udp_stream.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>

#include "cyber/cyber.h"

namespace apollo {
namespace drivers {
namespace hal {

UdpStream::UdpStream(const std::string& address, uint16_t port,
                     uint32_t timeout_usec)
    : sockfd_(-1), timeout_usec_(timeout_usec), is_receiver_(false) {
  // Determine the role based on the address. "0.0.0.0" means receiver.
  if (address == "0.0.0.0") {
    is_receiver_ = true;
  }

  struct in_addr addr_struct;
  int rc = inet_pton(AF_INET, address.c_str(), &addr_struct);
  if (rc <= 0) {
    throw std::invalid_argument("Invalid IPv4 address format: " + address);
  }
  peer_addr_ = addr_struct.s_addr;
  peer_port_ = htons(port);
}

UdpStream::~UdpStream() { this->close(); }

void UdpStream::close() {
  if (sockfd_ >= 0) {
    ::close(sockfd_);
    sockfd_ = -1;
  }
}

bool UdpStream::Connect() {
  if (sockfd_ >= 0) {
    return true;
  }

  try {
    // 1. Create socket
    int fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (fd < 0) {
      last_errno_ = errno;
      throw std::runtime_error("Failed to create UDP socket: " +
                               std::string(strerror(last_errno_)));
    }
    sockfd_ = fd;

    // 2. Set to non-blocking mode to work with pselect
    int flags = fcntl(sockfd_, F_GETFL, 0);
    if (flags == -1 || fcntl(sockfd_, F_SETFL, flags | O_NONBLOCK) == -1) {
      last_errno_ = errno;
      throw std::runtime_error("fcntl for non-blocking failed: " +
                               std::string(strerror(last_errno_)));
    }

    // 3. Perform bind() for receiver or connect() for sender
    if (is_receiver_) {
      // Role: Receiver. We must BIND to the specified port.
      struct sockaddr_in local_addr = {};
      local_addr.sin_family = AF_INET;
      local_addr.sin_port = peer_port_;
      local_addr.sin_addr.s_addr = INADDR_ANY;  // Bind to all interfaces

      if (::bind(sockfd_, (struct sockaddr*)&local_addr, sizeof(local_addr)) <
          0) {
        last_errno_ = errno;
        throw std::runtime_error("Failed to bind UDP socket to port " +
                                 std::to_string(ntohs(peer_port_)) + ": " +
                                 std::string(strerror(last_errno_)));
      }
      AINFO << "UdpStream bound to port " << ntohs(peer_port_)
            << " for receiving.";

    } else {
      // Role: Sender. We CONNECT to the remote peer.
      struct sockaddr_in remote_addr = {};
      remote_addr.sin_family = AF_INET;
      remote_addr.sin_port = peer_port_;
      remote_addr.sin_addr.s_addr = peer_addr_;

      if (::connect(sockfd_, (struct sockaddr*)&remote_addr,
                    sizeof(remote_addr)) < 0) {
        last_errno_ = errno;
        throw std::runtime_error(
            std::string("Failed to connect UDP socket to ") +
            inet_ntoa(remote_addr.sin_addr) + ":" +
            std::to_string(ntohs(peer_port_)) + " - " + strerror(last_errno_));
      }
      AINFO << "UdpStream connected to remote "
            << inet_ntoa(remote_addr.sin_addr) << ":" << ntohs(peer_port_);
    }

    status_ = Stream::Status::CONNECTED;
    return true;

  } catch (const std::exception& e) {
    AERROR << "Failed to Connect (UDP): " << e.what();
    if (sockfd_ >= 0) {
      this->close();
    }
    status_ = Stream::Status::ERROR;
    return false;
  }
}

bool UdpStream::Disconnect() {
  if (sockfd_ < 0) {
    return true;
  }
  this->close();
  status_ = Stream::Status::DISCONNECTED;
  return true;
}

size_t UdpStream::read(uint8_t* buffer, size_t max_length) {
  if (sockfd_ < 0) {
    last_errno_ = EBADF;
    return 0;
  }
  if (buffer == nullptr || max_length == 0) {
    return 0;
  }

  fd_set readfds;
  FD_ZERO(&readfds);
  FD_SET(sockfd_, &readfds);

  timespec timeout_ts = {0, 0};
  timespec* timeout_ptr = nullptr;
  if (timeout_usec_ > 0) {
    timeout_ts.tv_sec = timeout_usec_ / 1000000;
    timeout_ts.tv_nsec = (timeout_usec_ % 1000000) * 1000;
    timeout_ptr = &timeout_ts;
  }

  int r;
  do {
    r = pselect(sockfd_ + 1, &readfds, NULL, NULL, timeout_ptr, NULL);
  } while (r < 0 && errno == EINTR);

  if (r < 0) {
    last_errno_ = errno;
    throw std::runtime_error("UDP pselect error during read: " +
                             std::string(strerror(last_errno_)));
  } else if (r == 0) {
    return 0;  // Timeout
  }

  ssize_t ret = ::recvfrom(sockfd_, buffer, max_length, 0, NULL, NULL);

  if (ret < 0) {
    // Should not happen if pselect succeeded, but handle defensively.
    last_errno_ = errno;
    throw std::runtime_error("UDP recvfrom error after pselect: " +
                             std::string(strerror(last_errno_)));
  }

  return static_cast<size_t>(ret);
}

size_t UdpStream::write(const uint8_t* data, size_t length) {
  if (sockfd_ < 0) {
    last_errno_ = EBADF;
    return 0;
  }
  if (data == nullptr || length == 0) {
    return 0;
  }

  // NOTE: For a 'connect'ed UDP socket, send() is more efficient.
  // But sendto() works for both connected and unconnected sockets, making the
  // code simpler and more universal. The kernel optimizes for the connected
  // case.
  struct sockaddr_in peer_sockaddr = {};
  peer_sockaddr.sin_family = AF_INET;
  peer_sockaddr.sin_port = peer_port_;
  peer_sockaddr.sin_addr.s_addr = peer_addr_;

  ssize_t nsent;
  do {
    nsent = ::sendto(sockfd_, data, length, 0, (struct sockaddr*)&peer_sockaddr,
                     sizeof(peer_sockaddr));
  } while (nsent < 0 && errno == EINTR);

  if (nsent < 0) {
    // Unlike TCP, UDP writes are atomic. A full blocking-on-write is less
    // common. EAGAIN means the send buffer is full. We treat it as a
    // temporary failure and return 0.
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return 0;  // Output buffer full, caller can retry.
    }
    last_errno_ = errno;
    throw std::runtime_error("UDP sendto fatal error: " +
                             std::string(strerror(last_errno_)));
  }

  if (static_cast<size_t>(nsent) != length) {
    // This is rare for UDP but possible if length > buffer limits.
    AWARN << "UDP write was truncated: sent " << nsent << " of " << length
          << " bytes.";
  }

  return static_cast<size_t>(nsent);
}

}  // namespace hal
}  // namespace drivers
}  // namespace apollo
