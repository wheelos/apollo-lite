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

#include "modules/drivers/hal/stream/can_stream.h"

#include "cyber/cyber.h"

namespace apollo {
namespace drivers {
namespace hal {

// Constructor: Stores interface name, timeout, and CAN FD flag. Doesn't open
// the socket. Throws std::invalid_argument if interface name is empty or too
// long.
CanStream::CanStream(const std::string &address, uint32_t timeout_usec,
                     bool enable_can_fd)
    : interface_name_(address),
      timeout_usec_(timeout_usec),
      fd_(-1),
      enable_can_fd_(enable_can_fd) {
  if (interface_name_.empty()) {
    throw std::invalid_argument("CAN interface name cannot be empty.");
  }

  if (interface_name_.length() >= IFNAMSIZ) {
    throw std::invalid_argument("CAN interface name '" + interface_name_ +
                                "' is too long.");
  }
  // Port is unused for basic CAN_RAW, ignore it.
  // Socket is not opened until Connect() is called.
  // status_ is assumed DISCONNECTED initially by the base class constructor.
  AINFO << "CanStream created for interface: " << interface_name_
        << ", timeout: " << timeout_usec << " us"
        << ", CAN FD enabled: " << (enable_can_fd ? "true" : "false");
}

// Destructor: Ensures the CAN socket is closed.
CanStream::~CanStream() {
  this->close();  // Safe to call multiple times
}

// Helper function to create, bind, and configure the CAN socket.
// Throws std::runtime_error on failure.
void CanStream::open() {
  if (fd_ >= 0) {
    // Already open
    return;
  }

  // 1. Create the socket
  fd_ = socket(AF_CAN, SOCK_RAW, CAN_RAW);
  if (fd_ < 0) {
    last_errno_ = errno;
    throw std::runtime_error("Failed to create CAN socket: " +
                             std::string(strerror(last_errno_)));
  }

  // 2. Enable CAN FD support if requested
  if (enable_can_fd_) {
    int enable_fd = 1;
    if (setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_fd,
                   sizeof(enable_fd)) < 0) {
      last_errno_ = errno;
      ::close(fd_);
      throw std::runtime_error("Failed to enable CAN FD frames on socket: " +
                               std::string(strerror(last_errno_)));
    }
    AINFO << "CAN FD frames enabled on socket.";
  }

  // 3. Get the interface index
  struct ifreq ifr;
  std::memset(&ifr, 0, sizeof(ifr));
  std::strncpy(ifr.ifr_name, interface_name_.c_str(), IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';  // Ensure null termination

  if (ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
    last_errno_ = errno;
    ::close(fd_);
    throw std::runtime_error("Failed to get CAN interface index for '" +
                             interface_name_ +
                             "': " + std::string(strerror(last_errno_)));
  }

  // 4. Bind the socket to the interface index
  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    last_errno_ = errno;
    ::close(fd_);  // Clean up socket on error
    throw std::runtime_error("Failed to bind CAN socket to interface index " +
                             std::to_string(addr.can_ifindex) + ": " +
                             std::string(strerror(last_errno_)));
  }

  // 5. Set read/write timeouts (optional, based on timeout_usec_)
  if (timeout_usec_ > 0) {
    struct timeval tv = {
        static_cast<time_t>(timeout_usec_ / 1000000),      // seconds
        static_cast<suseconds_t>(timeout_usec_ % 1000000)  // microseconds
    };

    if (setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
      last_errno_ = errno;
      ::close(fd_);
      throw std::runtime_error("Failed to set SO_RCVTIMEO on CAN socket: " +
                               std::string(strerror(last_errno_)));
    }

    if (setsockopt(fd_, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv)) < 0) {
      last_errno_ = errno;
      ::close(fd_);
      throw std::runtime_error("Failed to set SO_SNDTIMEO on CAN socket: " +
                               std::string(strerror(last_errno_)));
    }
  }
  // TODO(zero): If timeout_usec_ == 0, read/write will be non-blocking
  // (EAGAIN/EWOULDBLOCK on no data/buffer).

  // Socket is successfully created, bound, and configured
  last_errno_ = 0;
  status_ = Stream::Status::CONNECTED;
  AINFO << "Successfully opened CAN socket for interface: " << interface_name_
        << ", fd: " << fd_;
}

// Helper function to close the CAN socket. Safe to call multiple times.
void CanStream::close() {
  if (fd_ >= 0) {  // Check if socket is open
    AINFO << "Closing CAN socket for interface " << interface_name_
          << ", fd: " << fd_;
    if (::close(fd_) < 0) {
      last_errno_ = errno;
      AERROR << "Failed to close CAN socket fd " << fd_ << ": "
             << strerror(last_errno_);
      // Don't throw on close failure in destructor or Disconnect.
    }
    fd_ = -1;
  }

  status_ = Stream::Status::DISCONNECTED;
}

// Connect: Creates, binds, and configures the CAN socket.
// Returns true if successful or already connected.
// Returns false on connection failure (when not throwing).
// Throws std::runtime_error for fatal setup errors.
bool CanStream::Connect() {
  if (fd_ >= 0) {
    return true;
  }

  last_errno_ = 0;

  try {
    // open() handles socket creation, binding, and configuration.
    // It throws std::runtime_error on failure.
    this->open();
    AINFO << "CanStream connected successfully to interface: "
          << interface_name_ << ", fd: " << fd_;
    return true;
  } catch (const std::exception &e) {
    last_errno_ = (fd_ < 0 && errno != 0) ? errno : 0;
    AERROR << "Failed to Connect to CAN interface " << interface_name_ << ": "
           << e.what();
    // fd_ is already -1 if open failed before setting it, or set to -1 by
    // open's cleanup on config failure.
    status_ = Stream::Status::ERROR;
  }
  return false;  // Connection failed
}

bool CanStream::Disconnect() {
  if (fd_ < 0) {
    AINFO << "CanStream already disconnected for interface: "
          << interface_name_;
    return true;
  }

  AINFO << "Disconnecting CAN stream for interface " << interface_name_
        << ", fd: " << fd_;
  this->close();  // close() sets fd_ = -1 and updates status_
  last_errno_ = 0;
  return true;
}

// Reads CAN frames into the buffer.
// buffer must be large enough to hold at least one frame (sizeof(struct
// can_frame) or sizeof(struct canfd_frame)). max_length should be a multiple of
// the chosen frame size. Returns number of bytes read (multiple of frame size,
// >= 0). Returns 0 on timeout or non-blocking with no data
// (EAGAIN/EWOULDBLOCK). Throws std::runtime_error on fatal read errors or
// connection loss.
size_t CanStream::read(uint8_t *buffer, size_t max_length) {
  if (fd_ < 0) {
    last_errno_ = ENOTCONN;
    AERROR << "CanStream read failed: Not connected.";
    return 0;
  }

  if (buffer == nullptr || max_length == 0) {
    return 0;
  }

  // Determine frame size based on enabled mode
  const size_t frame_size = get_frame_size();

  // Ensure max_length is a multiple of frame size
  if (max_length < frame_size || (max_length % frame_size) != 0) {
    last_errno_ = EINVAL;  // Invalid argument
    AERROR << "CanStream read failed: max_length (" << max_length
           << ") must be a multiple of CAN frame size (" << frame_size << ").";
    throw std::invalid_argument(
        "max_length must be a multiple of CAN frame size.");
  }

  // Calculate how many frames can fit
  size_t max_frames = max_length / frame_size;
  size_t frames_read = 0;
  uint8_t *current_buffer_pos = buffer;

  // Read loop to fill the buffer with as many frames as possible (up to
  // max_frames) recv respects the timeout_usec_ set via SO_RCVTIMEO.
  // TODO(zero): non-blocking (timeout_usec_ == 0), recv returns
  // EAGAIN/EWOULDBLOCK immediately.
  while (frames_read < max_frames) {
    ssize_t bytes_received;
    // Use recv specifically for sockets.
    do {
      bytes_received = recv(fd_, current_buffer_pos, frame_size, 0);
    } while (bytes_received < 0 && errno == EINTR);  // Handle EINTR

    if (bytes_received < 0) {
      // Error during recv
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        // Timeout or non-blocking socket has no data available immediately.
        // This is not a fatal error; break the loop and return frames read so
        // far.
        last_errno_ = errno;  // Store EAGAIN/EWOULDBLOCK
        break;                // Exit while loop
      } else {
        // Other serious errors (e.g., socket closed, interface down,
        // permissions)
        last_errno_ = errno;
        AERROR << "CanStream read failed: " << strerror(last_errno_)
               << " (errno: " << last_errno_ << "), fd: " << fd_
               << ", interface: " << interface_name_;
        status_ = Stream::Status::ERROR;
        Disconnect();
        throw std::runtime_error("CanStream read fatal error: " +
                                 std::string(strerror(last_errno_)));
      }
    } else if (bytes_received == 0) {
      // recv returned 0. For sockets, this usually indicates the peer has
      // performed an orderly shutdown. For CAN, this might indicate the
      // interface went down or was closed. Treat as connection lost.
      last_errno_ = ENOTCONN;  // Use ENOTCONN (Not connected)
      AERROR << "CanStream read failed: Connection lost (recv returned 0). FD: "
             << fd_ << ", interface: " << interface_name_;
      status_ = Stream::Status::DISCONNECTED;
      Disconnect();  // Clean up
      throw std::runtime_error("CanStream read connection lost.");
    } else {  // bytes_received > 0
      // Successfully read data. Should be equal to frame_size for CAN_RAW.
      if (static_cast<size_t>(bytes_received) != frame_size) {
        // Received less than a full frame. This is unexpected for CAN_RAW recv.
        last_errno_ = EIO;  // I/O error
        AERROR << "CanStream read failed: Received partial CAN frame. Expected "
               << frame_size << " bytes, got " << bytes_received
               << ". FD: " << fd_ << ", interface: " << interface_name_;
        // This indicates a serious issue. Disconnect and throw.
        Disconnect();
        throw std::runtime_error(
            "CanStream read fatal error: Received partial frame.");
      }
      // Successfully read one frame
      frames_read++;
      current_buffer_pos += frame_size;
      last_errno_ = 0;  // Clear error on success
      // Loop continues to try reading the next frame if space allows.
    }
  }

  // Return total bytes read across all frames
  return frames_read * frame_size;
}

// Writes CAN frames from the data buffer.
// length must be a multiple of the chosen frame size.
// Returns number of bytes written (multiple of frame size, >= 0).
// Returns 0 if not connected or on non-blocking EAGAIN/timeout.
// Throws std::runtime_error on fatal write errors.
size_t CanStream::write(const uint8_t *data, size_t length) {
  if (fd_ < 0) {
    last_errno_ = ENOTCONN;
    AERROR << "CanStream write failed: Not connected.";
    return 0;
  }

  if (data == nullptr || length == 0) {
    return 0;
  }

  // Determine frame size based on enabled mode
  const size_t frame_size = get_frame_size();

  // Ensure length is a multiple of frame size
  if (length < frame_size || (length % frame_size) != 0) {
    last_errno_ = EINVAL;  // Invalid argument
    AERROR << "CanStream write failed: length (" << length
           << ") must be a multiple of CAN frame size (" << frame_size << ").";
    throw std::invalid_argument("length must be a multiple of CAN frame size.");
  }

  // Calculate how many frames to send
  size_t num_frames = length / frame_size;
  size_t frames_sent = 0;
  const uint8_t *current_data_pos = data;

  // Write loop to send all frames from the buffer
  // send respects the timeout_usec_ set via SO_SNDTIMEO.
  // In non-blocking (timeout_usec_ == 0), send returns EAGAIN/EWOULDBLOCK
  // immediately if buffer is full.
  while (frames_sent < num_frames) {
    ssize_t bytes_sent;
    do {
      bytes_sent = send(fd_, current_data_pos, frame_size, 0);
    } while (bytes_sent < 0 && errno == EINTR);  // Handle EINTR

    if (bytes_sent < 0) {
      // Error during send
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        // Kernel buffer is full or timeout occurred before sending the frame.
        // This is not a fatal error; break the loop and return frames sent so
        // far.
        last_errno_ = errno;  // Store EAGAIN/EWOULDBLOCK
        break;                // Exit while loop
      } else {
        // Other serious errors (e.g., socket closed, interface down,
        // permissions)
        last_errno_ = errno;
        AERROR << "CanStream write failed: " << strerror(last_errno_)
               << " (errno: " << last_errno_ << "), fd: " << fd_
               << ", interface: " << interface_name_;
        status_ = Stream::Status::ERROR;
        // Fatal error. Disconnect and throw.
        Disconnect();
        throw std::runtime_error("CanStream write fatal error: " +
                                 std::string(strerror(last_errno_)));
      }
    } else if (bytes_sent == 0) {
      // send returned 0. This is unusual for send on a healthy socket unless
      // length was 0. With frame_size > 0, 0 bytes sent with no error usually
      // indicates a problem. Treat as fatal error.
      last_errno_ = EIO;  // I/O error or similar connection issue
      AERROR << "CanStream write failed: Sent 0 bytes unexpectedly. FD: " << fd_
             << ", interface: " << interface_name_;
      status_ = Stream::Status::ERROR;
      Disconnect();
      throw std::runtime_error(
          "CanStream write fatal error: Sent 0 bytes unexpectedly.");
    } else {  // bytes_sent > 0
      // Successfully sent data. Should be equal to frame_size for CAN_RAW.
      if (static_cast<size_t>(bytes_sent) != frame_size) {
        // Sent less than a full frame. This is unexpected for CAN_RAW send.
        last_errno_ = EIO;  // I/O error
        AERROR << "CanStream write failed: Sent partial CAN frame. Expected "
               << frame_size << " bytes, got " << bytes_sent << ". FD: " << fd_
               << ", interface: " << interface_name_;
        // This indicates a serious issue. Disconnect and throw.
        Disconnect();
        throw std::runtime_error(
            "CanStream write fatal error: Sent partial frame.");
      }
      // Successfully sent one frame
      frames_sent++;
      current_data_pos += frame_size;
      last_errno_ = 0;  // Clear error on success
      // Loop continues to try sending the next frame if space allows.
    }
  }

  return frames_sent * frame_size;
}

}  // namespace hal
}  // namespace drivers
}  // namespace apollo
