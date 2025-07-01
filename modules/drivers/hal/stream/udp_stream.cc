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
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

#include "cyber/cyber.h"

namespace apollo {
namespace drivers {
namespace hal {

UdpStream::UdpStream(const std::string& address, uint16_t port,
                     uint32_t timeout_usec)
    : sockfd_(-1), timeout_usec_(timeout_usec) {
  // Convert address string to network byte order binary form (IPv4)
  struct in_addr addr_struct;
  int rc = inet_pton(AF_INET, address.c_str(), &addr_struct);

  if (rc <= 0) {
    // inet_pton returns 0 for invalid format, -1 for error and sets errno.
    if (rc == 0) {
      throw std::invalid_argument("Invalid IPv4 address format: " + address);
    } else {
      // rc == -1, check errno
      throw std::runtime_error("inet_pton failed for address " + address +
                               ": " + strerror(errno));
    }
  }
  // Store the converted address (which is in network byte order)
  peer_addr_ = addr_struct.s_addr;

  // Convert port to network byte order
  peer_port_ = htons(port);
}

UdpStream::~UdpStream() { this->close(); }

void UdpStream::open() {
  if (sockfd_ >= 0) {
    // Already open, nothing to do
    return;
  }

  int fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (fd < 0) {
    last_errno_ = errno;
    throw std::runtime_error("Failed to create UDP socket: " +
                             std::string(strerror(last_errno_)));
  }

  // Configure blocking/non-blocking and timeouts based on timeout_usec_
  if (timeout_usec_ > 0) {  // Blocking with timeout
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags == -1) {
      last_errno_ = errno;
      ::close(fd);
      throw std::runtime_error(
          "fcntl(F_GETFL) failed for socket configuration: " +
          std::string(strerror(last_errno_)));
    }

    // Ensure blocking mode (remove O_NONBLOCK if present)
    if (fcntl(fd, F_SETFL, flags & ~O_NONBLOCK) == -1) {
      last_errno_ = errno;
      ::close(fd);
      throw std::runtime_error(
          "fcntl(F_SETFL, blocking) failed for socket configuration: " +
          std::string(strerror(last_errno_)));
    }

    // Set receive and send timeouts
    struct timeval block_to = {
        static_cast<time_t>(timeout_usec_ / 1000000),      // seconds
        static_cast<suseconds_t>(timeout_usec_ % 1000000)  // microseconds
    };

    if (setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &block_to, sizeof(block_to)) <
        0) {
      last_errno_ = errno;
      ::close(fd);
      throw std::runtime_error("setsockopt(SO_RCVTIMEO) failed: " +
                               std::string(strerror(last_errno_)));
    }

    if (setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &block_to, sizeof(block_to)) <
        0) {
      last_errno_ = errno;
      ::close(fd);
      throw std::runtime_error("setsockopt(SO_SNDTIMEO) failed: " +
                               std::string(strerror(last_errno_)));
    }

  } else {  // Non-blocking (timeout_usec_ == 0)
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags == -1) {
      last_errno_ = errno;
      ::close(fd);
      throw std::runtime_error(
          "fcntl(F_GETFL) failed for socket configuration: " +
          std::string(strerror(last_errno_)));
    }

    // Set non-blocking mode
    if (fcntl(fd, F_SETFL, flags | O_NONBLOCK) == -1) {
      last_errno_ = errno;
      ::close(fd);
      throw std::runtime_error(
          "fcntl(F_SETFL, non-blocking) failed for socket configuration: " +
          std::string(strerror(last_errno_)));
    }
  }

  // Socket is successfully created and configured
  sockfd_ = fd;
}

void UdpStream::close() {
  if (sockfd_ >= 0) {  // Check if socket is open
    if (::close(sockfd_) < 0) {
      // Log error but proceed, as sockfd_ must be invalidated anyway.
      last_errno_ = errno;
      AERROR << "Failed to close UDP socket fd " << sockfd_ << ": "
             << strerror(last_errno_);
    }
    sockfd_ = -1;
  }
}

bool UdpStream::Connect() {
  if (sockfd_ >= 0) {
    return true;
  }

  try {
    // Attempt to open the socket. open() throws on failure.
    this->open();
    status_ = Stream::Status::CONNECTED;
    return true;
  } catch (const std::exception& e) {
    // Catch exceptions from open() and log the error.
    AERROR << "Failed to Connect (open socket): " << e.what();
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
    return 0;
  }

  if (buffer == nullptr || max_length == 0) {
    return 0;  // Nothing to read
  }

  ssize_t ret = 0;
  struct sockaddr_in peer_sockaddr;
  socklen_t peer_len = sizeof(peer_sockaddr);
  memset(&peer_sockaddr, 0, sizeof(peer_sockaddr));

  // Use a loop to handle EINTR (interrupted system calls)
  do {
    ret = ::recvfrom(sockfd_, buffer, max_length, 0,
                     (struct sockaddr*)&peer_sockaddr, &peer_len);
  } while (ret < 0 && errno == EINTR);

  if (ret < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      // Timeout or non-blocking socket has no data available
      // Return 0 bytes read, which is standard for timeout/would block
      return 0;
    } else {
      // Other serious errors (e.g., socket closed, network error)
      last_errno_ = errno;
      AERROR << "UDP read error: " << strerror(last_errno_)
             << " (errno: " << last_errno_ << "), fd: " << sockfd_;
      return 0;
    }
  }

  return ret;
}

size_t UdpStream::write(const uint8_t* data, size_t length) {
  if (sockfd_ < 0) {
    return 0;  // Socket not open
  }

  if (data == nullptr || length == 0) {
    return 0;  // Nothing to write
  }

  struct sockaddr_in peer_sockaddr;
  // Use memset for robustness, though individual assignments are also fine.
  memset(&peer_sockaddr, 0, sizeof(peer_sockaddr));
  peer_sockaddr.sin_family = AF_INET;
  peer_sockaddr.sin_port = peer_port_;
  peer_sockaddr.sin_addr.s_addr = peer_addr_;

  socklen_t peer_len = sizeof(peer_sockaddr);

  // For UDP, sendto is usually atomic for datagrams within limits.
  // A loop like in TCP write is generally not needed and might be misleading.
  // Handle EINTR with a do-while loop.
  ssize_t nsent;
  do {
    nsent = ::sendto(sockfd_, data, length, 0, (struct sockaddr*)&peer_sockaddr,
                     peer_len);
  } while (nsent < 0 && errno == EINTR);

  if (nsent < 0) {
    // Handle errors other than EINTR
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      // Timeout or non-blocking socket cannot send immediately
      // Return 0 bytes sent
      return 0;
    } else {
      // Other serious errors (e.g., network error, socket closed)
      last_errno_ = errno;
      AERROR << "UDP write error: " << strerror(last_errno_)
             << " (errno: " << last_errno_ << "), fd: " << sockfd_;
      return 0;
    }
  }

  return nsent;
}

}  // namespace hal
}  // namespace drivers
}  // namespace apollo
