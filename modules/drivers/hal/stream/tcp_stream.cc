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

#include "modules/drivers/hal/stream/tcp_stream.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <cinttypes>
#include <cstring>
#include <iostream>

#include "cyber/cyber.h"

namespace apollo {
namespace drivers {
namespace hal {

TcpStream::TcpStream(const std::string& address, uint16_t port,
                     uint32_t timeout_usec, bool auto_reconnect)
    : sockfd_(-1),
      timeout_usec_(timeout_usec),
      auto_reconnect_(auto_reconnect) {
  struct in_addr addr_struct;
  int rc = inet_pton(AF_INET, address.c_str(), &addr_struct);
  if (rc <= 0) {
    if (rc == 0) {
      throw std::invalid_argument("Invalid IPv4 address format: " + address);
    } else {
      throw std::runtime_error("inet_pton failed for address " + address +
                               ": " + std::string(strerror(errno)));
    }
  }
  peer_addr_ = addr_struct.s_addr;
  peer_port_ = htons(port);
}

TcpStream::~TcpStream() { this->close(); }

void TcpStream::open() {
  if (sockfd_ >= 0) {
    return;
  }
  int fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (fd < 0) {
    last_errno_ = errno;
    throw std::runtime_error("Failed to create TCP socket: " +
                             std::string(strerror(last_errno_)));
  }
  sockfd_ = fd;
}

bool TcpStream::InitSocket() {
  if (sockfd_ < 0) {
    throw std::runtime_error("InitSocket called with invalid sockfd_");
  }

  // Ensure the socket is in non-blocking mode. This is foundational for using
  // select/pselect for timeout control.
  int flags = fcntl(sockfd_, F_GETFL, 0);
  if (flags == -1) {
    last_errno_ = errno;
    ::close(sockfd_);
    sockfd_ = -1;
    throw std::runtime_error("fcntl(F_GETFL) failed: " +
                             std::string(strerror(last_errno_)));
  }

  if ((flags & O_NONBLOCK) == 0) {
    if (fcntl(sockfd_, F_SETFL, flags | O_NONBLOCK) == -1) {
      last_errno_ = errno;
      ::close(sockfd_);
      sockfd_ = -1;
      throw std::runtime_error("fcntl(F_SETFL, O_NONBLOCK) failed: " +
                               std::string(strerror(last_errno_)));
    }
  }

  // Disable Nagle's algorithm for low-latency applications.
  int enable = 1;
  if (setsockopt(sockfd_, IPPROTO_TCP, TCP_NODELAY, &enable, sizeof(enable)) ==
      -1) {
    last_errno_ = errno;
    AWARN << "setsockopt(TCP_NODELAY) failed: " << strerror(last_errno_)
          << ". Performance may be affected.";
  }

  return true;
}

void TcpStream::close() {
  if (sockfd_ >= 0) {
    AINFO << "Closing TCP socket fd: " << sockfd_;
    ::close(sockfd_);
    sockfd_ = -1;
  }
}

bool TcpStream::Reconnect() {
  if (!auto_reconnect_) {
    return false;
  }
  Disconnect();
  try {
    if (Connect()) {
      AINFO << "Reconnect tcp success.";
      return true;
    }
  } catch (const std::exception& e) {
    AERROR << "Reconnect failed (Connect threw): " << e.what();
  }
  return false;
}

bool TcpStream::Connect() {
  if (sockfd_ >= 0) {
    return true;
  }

  last_errno_ = 0;

  try {
    this->open();  // Throws on failure

    // Set non-blocking for the connect call itself
    int flags = fcntl(sockfd_, F_GETFL, 0);
    if (flags == -1 || fcntl(sockfd_, F_SETFL, flags | O_NONBLOCK) == -1) {
      last_errno_ = errno;
      throw std::runtime_error("fcntl for non-blocking connect failed: " +
                               std::string(strerror(last_errno_)));
    }

    struct sockaddr_in peer_sockaddr = {};
    peer_sockaddr.sin_family = AF_INET;
    peer_sockaddr.sin_port = peer_port_;
    peer_sockaddr.sin_addr.s_addr = peer_addr_;

    int ret = ::connect(sockfd_, reinterpret_cast<sockaddr*>(&peer_sockaddr),
                        sizeof(peer_sockaddr));

    if (ret < 0) {
      if (errno == EINPROGRESS) {
        // This is the expected path for non-blocking connect.
        // Wait for completion using select.
        fd_set writefds;
        FD_ZERO(&writefds);
        FD_SET(sockfd_, &writefds);

        struct timeval select_timeout = {
            static_cast<time_t>(timeout_usec_ / 1000000),
            static_cast<suseconds_t>(timeout_usec_ % 1000000)};

        AINFO << "Waiting for connect with timeout: " << timeout_usec_
              << " us.";
        ret = select(sockfd_ + 1, NULL, &writefds, NULL,
                     (timeout_usec_ > 0) ? &select_timeout : NULL);

        if (ret <= 0) {
          if (ret == 0) {  // Timeout
            last_errno_ = ETIMEDOUT;
            throw std::runtime_error("Tcp connect timeout.");
          } else {  // select error
            last_errno_ = errno;
            throw std::runtime_error("Wait connect failed (select error): " +
                                     std::string(strerror(last_errno_)));
          }
        }

        // select reported socket is writable, check for connection errors
        int socket_error = 0;
        socklen_t len = sizeof(socket_error);
        if (getsockopt(sockfd_, SOL_SOCKET, SO_ERROR, &socket_error, &len) <
                0 ||
            socket_error != 0) {
          last_errno_ = (socket_error != 0) ? socket_error : errno;
          throw std::runtime_error("Socket error after connect: " +
                                   std::string(strerror(last_errno_)));
        }
        AINFO << "Non-blocking connect completed successfully.";
      } else {
        // Immediate connect error
        last_errno_ = errno;
        throw std::runtime_error("Tcp connect failed immediately: " +
                                 std::string(strerror(last_errno_)));
      }
    }

    // Now that we are connected, apply final socket configurations
    InitSocket();  // Throws on failure

    char buf[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &peer_addr_, buf, sizeof(buf));
    AINFO << "Tcp connect success to " << buf << ":" << ntohs(peer_port_);

    status_ = Stream::Status::CONNECTED;
    return true;

  } catch (const std::exception& e) {
    AERROR << "Failed to Connect: " << e.what();
    if (sockfd_ >= 0) {
      ::close(sockfd_);
      sockfd_ = -1;
    }
    status_ = Stream::Status::ERROR;
    return false;
  }
}

bool TcpStream::Disconnect() {
  if (sockfd_ < 0) {
    return true;
  }
  this->close();
  last_errno_ = 0;
  status_ = Stream::Status::DISCONNECTED;
  return true;
}

size_t TcpStream::read(uint8_t* buffer, size_t max_length) {
  if (sockfd_ < 0) {
    if (auto_reconnect_ && Reconnect()) {
      AINFO << "Read called on disconnected socket, reconnected successfully.";
    } else {
      last_errno_ = ENOTCONN;
      return 0;
    }
  }

  if (buffer == nullptr || max_length == 0) {
    return 0;
  }

  // Step 1: Wait for data to be readable using pselect
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

  int r = pselect(sockfd_ + 1, &readfds, NULL, NULL, timeout_ptr, NULL);

  if (r < 0) {
    last_errno_ = errno;
    if (last_errno_ == EINTR) {
      return 0;  // Interrupted by signal, not an error. Return 0 bytes read.
    }
    // A more serious error occurred in pselect
    AERROR << "pselect failed during read: " << strerror(last_errno_);
    Disconnect();
    throw std::runtime_error("TCP pselect fatal error during read: " +
                             std::string(strerror(last_errno_)));
  } else if (r == 0) {
    // pselect timed out. This is not an error, just means no data arrived.
    return 0;
  }

  // Step 2: pselect succeeded, so data is available. Now call recv.
  ssize_t ret;
  do {
    ret = ::recv(sockfd_, buffer, max_length, 0);
  } while (ret < 0 && errno == EINTR);

  if (ret < 0) {
    // Since pselect guaranteed data, an error here (like EAGAIN) is unexpected
    // but possible in edge cases. Other errors are fatal.
    last_errno_ = errno;
    AERROR << "TCP read error after pselect: " << strerror(last_errno_);
    Disconnect();
    throw std::runtime_error("TCP read fatal error after select: " +
                             std::string(strerror(last_errno_)));
  }

  if (ret == 0) {
    // recv returning 0 means the remote side has closed the connection.
    AINFO << "TCP remote closed connection on fd: " << sockfd_;
    last_errno_ = 0;
    Disconnect();

    if (auto_reconnect_ && Reconnect()) {
      AINFO << "Read encountered remote close, reconnected successfully.";
      return 0;  // Return 0 bytes for this attempt, caller can retry.
    } else {
      throw std::runtime_error(
          "TCP remote closed connection and reconnect failed or is disabled.");
    }
  }

  return static_cast<size_t>(ret);
}

size_t TcpStream::write(const uint8_t* buffer, size_t length) {
  if (sockfd_ < 0) {
    if (auto_reconnect_ && Reconnect()) {
      AINFO << "Write called on disconnected socket, reconnected successfully.";
    } else {
      last_errno_ = ENOTCONN;
      return 0;
    }
  }

  if (buffer == nullptr || length == 0) {
    return 0;
  }

  size_t total_sent = 0;
  while (total_sent < length) {
    ssize_t sent_bytes;
    do {
      sent_bytes = ::send(sockfd_, buffer + total_sent, length - total_sent, 0);
    } while (sent_bytes < 0 && errno == EINTR);

    if (sent_bytes < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        // Send buffer is full. Wait until we can write again.
        fd_set writefds;
        FD_ZERO(&writefds);
        FD_SET(sockfd_, &writefds);

        timespec timeout_ts = {0, 0};
        timespec* timeout_ptr = nullptr;
        if (timeout_usec_ > 0) {
          timeout_ts.tv_sec = timeout_usec_ / 1000000;
          timeout_ts.tv_nsec = (timeout_usec_ % 1000000) * 1000;
          timeout_ptr = &timeout_ts;
        }

        int r = pselect(sockfd_ + 1, NULL, &writefds, NULL, timeout_ptr, NULL);

        if (r <= 0) {
          if (r == 0) {  // Timeout
            AWARN << "TCP write timeout after sending " << total_sent << " of "
                  << length << " bytes.";
            last_errno_ = ETIMEDOUT;
          } else {  // pselect error
            AERROR << "pselect failed during write: " << strerror(errno);
            last_errno_ = errno;
          }
          // In case of timeout or error, we return what we've sent so far.
          // The caller can check the return value against the requested length
          // to see if it failed.
          return total_sent;
        }
        // pselect says we can write, so continue the loop to try send again.
        continue;
      } else {
        // A real error occurred.
        last_errno_ = errno;
        AERROR << "TCP write fatal error: " << strerror(last_errno_);
        Disconnect();
        throw std::runtime_error("TCP write fatal error: " +
                                 std::string(strerror(last_errno_)));
      }
    }
    total_sent += sent_bytes;
  }

  last_errno_ = 0;
  return total_sent;
}

bool TcpStream::Readable(uint32_t timeout_us) {
  if (sockfd_ < 0) {
    last_errno_ = ENOTCONN;
    return false;
  }

  fd_set readfds;
  FD_ZERO(&readfds);
  FD_SET(sockfd_, &readfds);

  timespec timeout_ts = {static_cast<time_t>(timeout_us / 1000000),
                         static_cast<long>((timeout_us % 1000000) * 1000)};

  int r = pselect(sockfd_ + 1, &readfds, NULL, NULL, &timeout_ts, NULL);

  if (r < 0) {
    last_errno_ = errno;
    AERROR << "pselect failed in Readable(): " << strerror(last_errno_);
    return false;
  }

  return (r > 0) && FD_ISSET(sockfd_, &readfds);
}

}  // namespace hal
}  // namespace drivers
}  // namespace apollo
