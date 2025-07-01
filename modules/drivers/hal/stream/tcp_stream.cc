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
    // inet_pton returns 0 for invalid format, -1 for error and sets errno.
    if (rc == 0) {
      throw std::invalid_argument("Invalid IPv4 address format: " + address);
    } else {  // rc == -1
      // Check errno for system errors (less common for pton unless args
      // invalid)
      throw std::runtime_error("inet_pton failed for address " + address +
                               ": " + std::string(strerror(errno)));
    }
  }
  // Store the converted address (which is in network byte order)
  peer_addr_ = addr_struct.s_addr;

  // Convert port to network byte order
  peer_port_ = htons(port);
}

TcpStream::~TcpStream() { this->close(); }

void TcpStream::open() {
  if (sockfd_ >= 0) {
    // Already open
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
    // Should not happen if called correctly after open(), but defensive check
    throw std::runtime_error("InitSocket called with invalid sockfd_");
  }

  // Store original flags to restore after non-blocking connect
  int original_flags = fcntl(sockfd_, F_GETFL, 0);
  if (original_flags == -1) {
    last_errno_ = errno;
    ::close(sockfd_);
    sockfd_ = -1;
    throw std::runtime_error(
        "fcntl(F_GETFL) failed for socket configuration: " +
        std::string(strerror(last_errno_)));
  }

  // Configure blocking/non-blocking and timeouts based on timeout_usec_
  if (timeout_usec_ > 0) {  // Blocking with timeout
    // Ensure blocking mode (remove O_NONBLOCK if present)
    if (fcntl(sockfd_, F_SETFL, original_flags & ~O_NONBLOCK) == -1) {
      last_errno_ = errno;
      ::close(sockfd_);
      sockfd_ = -1;
      throw std::runtime_error(
          "fcntl(F_SETFL, blocking) failed for socket configuration: " +
          std::string(strerror(last_errno_)));
    }

    // Set receive and send timeouts
    struct timeval block_to = {
        static_cast<time_t>(timeout_usec_ / 1000000),      // seconds
        static_cast<suseconds_t>(timeout_usec_ % 1000000)  // microseconds
    };

    if (setsockopt(sockfd_, SOL_SOCKET, SO_RCVTIMEO, &block_to,
                   sizeof(block_to)) < 0) {
      last_errno_ = errno;
      ::close(sockfd_);
      sockfd_ = -1;
      throw std::runtime_error("setsockopt(SO_RCVTIMEO) failed: " +
                               std::string(strerror(last_errno_)));
    }

    if (setsockopt(sockfd_, SOL_SOCKET, SO_SNDTIMEO, &block_to,
                   sizeof(block_to)) < 0) {
      last_errno_ = errno;
      ::close(sockfd_);
      sockfd_ = -1;
      throw std::runtime_error("setsockopt(SO_SNDTIMEO) failed: " +
                               std::string(strerror(last_errno_)));
    }
  } else {  // Non-blocking (timeout_usec_ == 0)
    // Set non-blocking mode
    if (fcntl(sockfd_, F_SETFL, original_flags | O_NONBLOCK) == -1) {
      last_errno_ = errno;
      ::close(sockfd_);
      sockfd_ = -1;
      throw std::runtime_error(
          "fcntl(F_SETFL, non-blocking) failed for socket configuration: " +
          std::string(strerror(last_errno_)));
    }
    // Note: SO_RCVTIMEO/SO_SNDTIMEO are not needed for pure non-blocking.
    // EAGAIN/EWOULDBLOCK return codes indicate immediate operation would block.
  }

  // Disable Nagle's algorithm (TCP_NODELAY)
  int enable = 1;
  if (setsockopt(sockfd_, IPPROTO_TCP, TCP_NODELAY, &enable, sizeof(enable)) ==
      -1) {
    last_errno_ = errno;
    // Nagle disable failing is often non-fatal, just log warning.
    // Consider throwing if this is critical for the application.
    AERROR << "setsockopt disable Nagle failed, errno: " << last_errno_ << ", "
           << strerror(last_errno_) << ". Continuing.";
    // Don't throw here unless disabling Nagle is a strict requirement.
  }

  return true;  // Configuration successful
}

void TcpStream::close() {
  if (sockfd_ >= 0) {
    AINFO << "Closing TCP socket fd: " << sockfd_;
    if (::close(sockfd_) < 0) {
      last_errno_ = errno;
      AERROR << "Failed to close TCP socket fd " << sockfd_ << ": "
             << strerror(last_errno_);
    }
    sockfd_ = -1;
  }
}

bool TcpStream::Reconnect() {
  if (!auto_reconnect_) {
    return false;
  }

  Disconnect();  // Close current socket gracefully
  try {
    if (Connect()) {
      AINFO << "Reconnect tcp success.";
      return true;
    }
  } catch (const std::exception& e) {
    // Catch exception from Connect and log it as reconnection failure.
    AERROR << "Reconnect failed (Connect threw): " << e.what();
  }

  return false;
}

bool TcpStream::Connect() {
  if (sockfd_ >= 0) {
    // Already open
    // Assuming status_ is correctly CONNECTED if sockfd_ >= 0
    return true;
  }

  last_errno_ = 0;  // Reset error code for this attempt

  try {
    // 1. Create the socket
    this->open();  // Throws on failure

    // 2. Set socket to non-blocking temporarily for connection attempt
    int original_flags = fcntl(sockfd_, F_GETFL, 0);
    if (original_flags == -1) {
      last_errno_ = errno;
      ::close(sockfd_);
      sockfd_ = -1;
      throw std::runtime_error("fcntl(F_GETFL) failed before connect: " +
                               std::string(strerror(last_errno_)));
    }
    if (fcntl(sockfd_, F_SETFL, original_flags | O_NONBLOCK) == -1) {
      last_errno_ = errno;
      ::close(sockfd_);
      sockfd_ = -1;
      throw std::runtime_error(
          "fcntl(F_SETFL, non-blocking) failed before connect: " +
          std::string(strerror(last_errno_)));
    }

    // 3. Initiate non-blocking connect
    struct sockaddr_in peer_sockaddr = {};  // Initialize to zeros
    peer_sockaddr.sin_family = AF_INET;
    peer_sockaddr.sin_port = peer_port_;         // Network byte order
    peer_sockaddr.sin_addr.s_addr = peer_addr_;  // Network byte order

    int ret = ::connect(sockfd_, reinterpret_cast<sockaddr*>(&peer_sockaddr),
                        sizeof(peer_sockaddr));

    if (ret < 0) {
      if (errno == EINTR) {
        AINFO << "Tcp connect returned EINTR during initial call. Retrying or "
                 "waiting.";
        // EINTR during initial connect is rare but possible, treat as need to
        // wait.
      } else if (errno == EISCONN || errno == EALREADY) {
        // Already connected (shouldn't happen if sockfd_ < 0 check passed, but
        // defensive)
        AINFO << "Tcp connect reported already connected.";
        ret = 0;  // Treat as success for flow below
      } else if (errno != EINPROGRESS) {
        // Immediate error other than EINPROGRESS
        last_errno_ = errno;
        AERROR << "Tcp connect failed immediately, error: "
               << strerror(last_errno_);
        ::close(sockfd_);
        sockfd_ = -1;  // Clean up socket
        status_ = Stream::Status::ERROR;
        return false;  // Connection failed
      }
      // If errno == EINPROGRESS or EINTR, proceed to select wait
    }

    // If ret == 0 initially, or errno was EINPROGRESS/EINTR, wait for
    // connection completion
    if (ret < 0 && (errno == EINPROGRESS || errno == EINTR)) {
      // 4. Wait for connect to complete using select
      fd_set writefds;
      FD_ZERO(&writefds);
      FD_SET(sockfd_, &writefds);

      struct timeval select_timeout;
      if (timeout_usec_ == 0) {
        // Non-blocking connect, no wait (select timeout 0)
        select_timeout = {0, 0};
        AINFO << "Non-blocking connect, checking status immediately.";
      } else {
        // Blocking connect with timeout, use timeout_usec_ for select
        select_timeout = {static_cast<time_t>(timeout_usec_ / 1000000),
                          static_cast<suseconds_t>(timeout_usec_ % 1000000)};
        AINFO << "Waiting for connect with timeout: " << timeout_usec_
              << " us.";
      }

      ret = select(sockfd_ + 1, NULL, &writefds, NULL, &select_timeout);

      if (ret < 0) {
        last_errno_ = errno;
        AERROR << "Wait connect failed (select error): "
               << strerror(last_errno_);
        ::close(sockfd_);
        sockfd_ = -1;
        status_ = Stream::Status::ERROR;
        return false;  // Connection failed
      } else if (ret == 0) {
        // select timed out
        last_errno_ = ETIMEDOUT;  // Use ETIMEDOUT to indicate connect timeout
        AINFO << "Tcp connect timeout.";
        ::close(sockfd_);
        sockfd_ = -1;
        status_ = Stream::Status::DISCONNECTED;
        return false;  // Connection failed
      } else if (FD_ISSET(sockfd_, &writefds)) {
        // Socket is writable, check for connection errors using SO_ERROR
        int socket_error = 0;
        socklen_t len = sizeof(socket_error);
        if (getsockopt(sockfd_, SOL_SOCKET, SO_ERROR, &socket_error, &len) <
            0) {
          last_errno_ = errno;
          AERROR << "getsockopt(SO_ERROR) failed after select: "
                 << strerror(last_errno_);
          ::close(sockfd_);
          sockfd_ = -1;
          status_ = Stream::Status::ERROR;
          return false;  // Connection failed
        }

        if (socket_error != 0) {
          // Connection failed asynchronously
          last_errno_ = socket_error;  // Use the actual socket error
          AERROR << "Socket error after connect select: "
                 << strerror(last_errno_);
          ::close(sockfd_);
          sockfd_ = -1;  // Clean up
          status_ = Stream::Status::ERROR;
          return false;  // Connection failed
        }

        // If we reach here, non-blocking connect completed successfully
        AINFO << "Non-blocking connect completed successfully.";

      } else {
        // This case should ideally not be reached if select returns > 0 but
        // FD_ISSET is false
        last_errno_ = EIO;  // Indicate unexpected I/O error
        AERROR
            << "Select returned > 0 but FD_ISSET is false. Unexpected state.";
        ::close(sockfd_);
        sockfd_ = -1;
        status_ = Stream::Status::ERROR;
        return false;  // Connection failed
      }
    }

    // 5. Restore original socket flags (blocking/non-blocking as per
    // timeout_usec_)
    if (fcntl(sockfd_, F_SETFL, original_flags) == -1) {
      last_errno_ = errno;
      // Failing to restore flags is bad, but connection was successful. Log and
      // continue.
      AERROR << "Failed to restore socket flags after connect. Connection "
                "might behave unexpectedly: "
             << strerror(last_errno_);
      // Decide if this should be a fatal error requiring close/throw.
      // For now, log and continue assuming connection is primary goal.
    }

    // 6. Initialize other socket options (SO_RCVTIMEO, SO_SNDTIMEO, TCP_NODELAY
    // etc.) InitSocket() handles cleanup if it fails.
    InitSocket();  // Throws on failure

    // If we reach here, connection and configuration are successful
    char buf[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &peer_addr_, buf, sizeof(buf));
    AINFO << "Tcp connect success to " << buf << ":" << ntohs(peer_port_);

    status_ = Stream::Status::CONNECTED;
    return true;  // Connection successful
  } catch (const std::exception& e) {
    // Catch exceptions thrown by open() or InitSocket()
    AERROR << "Failed to Connect (initial setup or config threw): " << e.what();
    // Socket might already be closed by the throwing helper, but ensure it.
    if (sockfd_ >= 0) {
      ::close(sockfd_);
      sockfd_ = -1;
    }
    status_ = Stream::Status::ERROR;
    return false;  // Connection failed
  }
}

bool TcpStream::Disconnect() {
  if (sockfd_ < 0) {
    // not open
    return true;
  }

  this->close();
  last_errno_ = 0;
  status_ = Stream::Status::DISCONNECTED;
  return true;
}

size_t TcpStream::read(uint8_t* buffer, size_t max_length) {
  if (sockfd_ < 0) {
    // last_errno_ might hold previous error.
    if (auto_reconnect_ && Reconnect()) {
      // Successfully reconnected, proceed with read attempt
      AINFO << "Read called on disconnected socket, reconnected successfully.";
    } else {
      // Not open, cannot reconnect, or reconnect failed.
      last_errno_ = ENOTCONN;  // Use ENOTCONN to indicate not connected
      return 0;                // Return 0 as nothing was read
    }
  }

  if (buffer == nullptr || max_length == 0) {
    return 0;  // Nothing to read into
  }

  ssize_t ret;
  // Use a loop to handle EINTR (interrupted system calls)
  do {
    ret = ::recv(sockfd_, buffer, max_length, 0);
  } while (ret < 0 && errno == EINTR);

  if (ret < 0) {
    // Handle errors other than EINTR
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      // Timeout or non-blocking socket has no data available
      // Return 0 bytes read, which is standard for timeout/would block
      // last_errno_ = errno; // Optionally store EAGAIN/EWOULDBLOCK
      return 0;
    } else {
      // Other serious errors (e.g., socket closed, network error, permissions)
      last_errno_ = errno;
      AERROR << "TCP read error: " << strerror(last_errno_)
             << " (errno: " << last_errno_ << "), fd: " << sockfd_;
      // Fatal error, throw exception to indicate failure
      // Caller needs to handle this and decide whether to Reconnect.
      Disconnect();  // Clean up the broken socket
      throw std::runtime_error("TCP read fatal error: " +
                               std::string(strerror(last_errno_)));
    }
  }

  if (ret == 0) {
    // Remote closed the connection gracefully
    AINFO << "TCP remote closed connection on fd: " << sockfd_;
    last_errno_ = 0;  // No system error code for remote closed
    Disconnect();     // Clean up the socket

    // Attempt reconnect automatically if configured
    if (auto_reconnect_ && Reconnect()) {
      AINFO << "Read encountered remote close, reconnected successfully.";
      // Return 0 bytes read for this attempt, caller can retry read.
      return 0;
    } else {
      // Remote closed and auto-reconnect failed or disabled.
      // Throw exception to signal the caller that the connection is lost.
      throw std::runtime_error("TCP remote closed connection.");
    }
  }

  return ret;
}

size_t TcpStream::write(const uint8_t* buffer, size_t length) {
  if (sockfd_ < 0) {
    // last_errno_ might hold previous error.
    if (auto_reconnect_ && Reconnect()) {
      // Successfully reconnected, proceed with write attempt
      AINFO << "Write called on disconnected socket, reconnected successfully.";
    } else {
      // Not open, cannot reconnect, or reconnect failed.
      last_errno_ = ENOTCONN;  // Use ENOTCONN to indicate not connected
      return 0;                // Return 0 as nothing was written
    }
  }

  if (buffer == nullptr || length == 0) {
    return 0;  // Nothing to write
  }

  size_t total_nsent = 0;
  const uint8_t* current_data = buffer;
  size_t remaining_length = length;

  // TCP send can write less than requested, so loop is necessary.
  while (remaining_length > 0) {
    ssize_t nsent;
    // Use a loop for EINTR
    do {
      nsent = ::send(sockfd_, current_data, remaining_length, 0);
    } while (nsent < 0 && errno == EINTR);

    if (nsent < 0) {
      // Handle errors other than EINTR
      last_errno_ = errno;
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        // Timeout or non-blocking socket cannot send immediately
        // Return total bytes sent so far. Caller should retry sending the rest.
        AINFO << "TCP write would block (EAGAIN/EWOULDBLOCK) after sending "
              << total_nsent << " bytes.";
        return total_nsent;  // Return partial write count + indicate block
      } else {
        // Other serious errors (e.g., broken pipe, connection reset, network
        // error)
        AERROR << "TCP write error: " << strerror(last_errno_)
               << " (errno: " << last_errno_ << "), fd: " << sockfd_;
        // Assuming status_ is inherited. Set error status.
        // status_ = Stream::Status::ERROR; // Or DISCONNECTED for pipe/reset
        // errors Fatal error, throw exception. Caller needs to handle this and
        // potentially Reconnect.
        Disconnect();  // Clean up the broken socket
        throw std::runtime_error("TCP write fatal error: " +
                                 std::string(strerror(last_errno_)));
      }
    }

    // nsent >= 0 : Bytes were sent (could be 0 if length was 0, or partial).
    total_nsent += nsent;
    current_data += nsent;
    remaining_length -= nsent;
  }

  last_errno_ = 0;  // Clear error on success
  return total_nsent;
}

bool TcpStream::Readable(uint32_t timeout_us) {
  if (sockfd_ < 0) {
    last_errno_ = ENOTCONN;
    return false;
  }

  // Setup a select call to block for serial data or a timeout
  timespec timeout_ts;
  fd_set readfds;
  FD_ZERO(&readfds);
  FD_SET(sockfd_, &readfds);

  timeout_ts.tv_sec = timeout_us / 1000000;
  timeout_ts.tv_nsec = (timeout_us % 1000000) * 1000;
  int r = pselect(sockfd_ + 1, &readfds, NULL, NULL, &timeout_ts, NULL);
  if (r < 0) {
    status_ = Stream::Status::ERROR;
    last_errno_ = errno;
    AERROR << "Failed to wait tcp data: " << errno << ", " << strerror(errno);
    return false;
  } else if (r == 0 || !FD_ISSET(sockfd_, &readfds)) {
    return false;
  }
  // Data available to read.
  return true;
}

}  // namespace hal
}  // namespace drivers
}  // namespace apollo
