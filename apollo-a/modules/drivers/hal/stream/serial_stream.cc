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

#include "modules/drivers/hal/stream/serial_stream.h"

#include "cyber/cyber.h"

namespace apollo {
namespace drivers {
namespace hal {

// Constructor: Stores configuration parameters. Doesn't open the port.
// Throws std::invalid_argument if device_name is empty.
SerialStream::SerialStream(const std::string& device_name,
                           baud_rate_t baud_rate, SerialByteSize bytesize,
                           SerialParity parity, SerialStopBits stopbits,
                           SerialFlowControl flowcontrol, uint32_t timeout_usec)
    : device_name_(device_name),
      baud_rate_(baud_rate),
      bytesize_(bytesize),
      parity_(parity),
      stopbits_(stopbits),
      flowcontrol_(flowcontrol),
      timeout_usec_(timeout_usec),
      fd_(-1),
      is_open_(false) {
  if (device_name_.empty()) {
    // Use exception for invalid constructor arguments
    throw std::invalid_argument("Serial device name cannot be empty.");
  }
}

SerialStream::~SerialStream() {
  this->close();  // Safe to call multiple times
}

// Helper function to open the serial port file descriptor.
// Throws std::runtime_error on failure.
void SerialStream::open(void) {
  if (fd_ >= 0) {
    // Already open
    return;
  }

  // Use O_RDWR for read/write access, O_NOCTTY to prevent it becoming the
  // controlling terminal, O_NONBLOCK for non-blocking operation (termios
  // VMIN/VTIME will control blocking behavior of read). Removed recursive call
  // on EINTR. open should just fail on error.
  fd_ = ::open(device_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

  if (fd_ == -1) {
    last_errno_ = errno;
    // Throw exception on failure to open device
    throw std::runtime_error("Failed to open device " + device_name_ +
                             ", error: " + std::string(strerror(last_errno_)));
  }

  // Configure the port settings using termios
  try {
    configure_port();  // configure_port throws on failure
  } catch (const std::exception& e) {
    // If configuration fails, close the file descriptor and re-throw
    AERROR << "Failed to configure port " << device_name_
           << " after opening: " << e.what();
    ::close(fd_);
    fd_ = -1;
    // Re-throw the configuration error
    throw;
  }

  // If open and configure_port were successful
  is_open_ = true;
  last_errno_ = 0;  // Clear last error on successful open
  AINFO << "Successfully opened and configured serial port: " << device_name_;
}

// Helper function to configure the serial port using termios.
// Assumes fd_ is valid and the port is open.
// Throws std::runtime_error on failure.
void SerialStream::configure_port() {
  if (fd_ < 0) {
    // Should not happen if called correctly after open(), but defensive check
    throw std::runtime_error("configure_port called with invalid fd_");
  }

  struct termios options;  // The options for the file descriptor
  if (tcgetattr(fd_, &options) == -1) {
    last_errno_ = errno;
    throw std::runtime_error("tcgetattr failed for device " + device_name_ +
                             ": " + std::string(strerror(last_errno_)));
  }

  // --- Apply Raw Mode / No Echo / Binary ---
  // Enable receiver, ignore modem control lines
  options.c_cflag |= (tcflag_t)(CLOCAL | CREAD);
  // Disable canonical mode, echo, signals etc.
  options.c_lflag &=
      (tcflag_t) ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | ISIG | IEXTEN);
  options.c_oflag &= (tcflag_t) ~(OPOST);  // Disable output postprocessing
  // Disable CR/NL handling, ignore break
  options.c_iflag &= (tcflag_t) ~(INLCR | IGNCR | ICRNL | IGNBRK);

#ifdef IUCLC  // Not common, typically cleared
  options.c_iflag &= (tcflag_t)~IUCLC;
#endif
#ifdef PARMRK  // Typically cleared in raw mode
  options.c_iflag &= (tcflag_t)~PARMRK;
#endif

  // --- Set Baud Rate ---
  // Use standard POSIX functions for setting speeds.
  if (cfsetispeed(&options, baud_rate_) == -1) {
    last_errno_ = errno;
    // Attempt to get numerical baud rate for logging if possible (requires
    // mapping)
    AERROR << "cfsetispeed failed for device " << device_name_
           << ", baud: " << baud_rate_ << ": "
           << std::string(strerror(last_errno_));
    throw std::runtime_error("cfsetispeed failed.");
  }
  if (cfsetospeed(&options, baud_rate_) == -1) {
    last_errno_ = errno;
    AERROR << "cfsetospeed failed for device " << device_name_
           << ", baud: " << baud_rate_ << ": "
           << std::string(strerror(last_errno_));
    throw std::runtime_error("cfsetospeed failed.");
  }

  // --- Set Character Size (Data Bits) ---
  options.c_cflag &= (tcflag_t)~CSIZE;  // Clear current size bits
  switch (bytesize_) {
    case SerialByteSize::B5:
      options.c_cflag |= CS5;
      break;
    case SerialByteSize::B6:
      options.c_cflag |= CS6;
      break;
    case SerialByteSize::B7:
      options.c_cflag |= CS7;
      break;
    case SerialByteSize::B8:
      options.c_cflag |= CS8;
      break;
    default:
      // This case should ideally not be reached if enum is used correctly,
      // but handle defensively or validate in constructor.
      throw std::invalid_argument("Unsupported byte size value.");
  }

  // --- Set Stop Bits ---
  switch (stopbits_) {
    case SerialStopBits::One:
      options.c_cflag &= (tcflag_t)~CSTOPB;
      break;  // 1 stop bit (clear CSTOPB)
    case SerialStopBits::Two:
      options.c_cflag |= CSTOPB;
      break;  // 2 stop bits (set CSTOPB)
    default:
      throw std::invalid_argument("Unsupported stop bits value.");
  }

  // --- Set Parity ---
  // Disable input parity checking and strip
  options.c_iflag &= (tcflag_t) ~(INPCK | ISTRIP);
  // Disable parity enable and odd parity
  options.c_cflag &= (tcflag_t) ~(PARENB | PARODD);
  switch (parity_) {
    case SerialParity::None:
      // PARENB is already cleared above
      break;
    case SerialParity::Even:
      options.c_cflag |= PARENB;  // Enable parity
      // PARODD is already cleared above for Even
      break;
    case SerialParity::Odd:
      options.c_cflag |= PARENB | PARODD;  // Enable parity and set Odd
      break;
    default:
      throw std::invalid_argument("Unsupported parity value.");
  }

// --- Set Flow Control ---
// Clear all flow control flags first
#ifdef IXANY
  // Clear XON/XOFF flags
  options.c_iflag &= (tcflag_t) ~(IXON | IXOFF | IXANY);
#else
  options.c_iflag &= (tcflag_t) ~(IXON | IXOFF);
#endif
#ifdef CRTSCTS  // Clear hardware flow control flags (RTS/CTS)
  options.c_cflag &= static_cast<tcflag_t>(~(CRTSCTS));
#elif defined CNEW_RTSCTS
  options.c_cflag &= static_cast<tcflag_t>(~(CNEW_RTSCTS));
#else
  AINFO << "Neither CRTSCTS nor CNEW_RTSCTS defined. Hardware flow control "
           "might not be supported.";
#endif

  switch (flowcontrol_) {
    case SerialFlowControl::None:
      // All flow control flags are cleared above
      break;
    case SerialFlowControl::XONXOFF:
      options.c_iflag |= (tcflag_t)(IXON | IXOFF);  // Enable XON/XOFF
#ifdef IXANY  // If available, allow any character to restart output
      options.c_iflag |= (tcflag_t)IXANY;
#endif
      break;
    case SerialFlowControl::RTSCTS:
#ifdef CRTSCTS  // Standard POSIX hardware flow control
      options.c_cflag |= static_cast<tcflag_t>(CRTSCTS);
#elif defined CNEW_RTSCTS  // Alternative non-standard macro
      options.c_cflag |= static_cast<tcflag_t>(CNEW_RTSCTS);
#else
      // Hardware flow control requested but not supported by OS macros
      throw std::runtime_error(
          "RTS/CTS flow control requested but not supported by OS.");
#endif
      break;
    default:
      throw std::invalid_argument("Unsupported flow control value.");
  }

  // --- Set Read Timeout and VMIN/VTIME ---
  // http://www.unixwiz.net/techtips/termios-vmin-vtime.html
  // VMIN=0, VTIME=0: Non-blocking read in the kernel. read returns immediately
  // with available data, or 0 if no data. This is used with select/poll/epoll.
  // The timeout_usec_ parameter in this class controls the wait duration
  // *before* calling read(), implemented using wait_readable/wait_writable.
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 0;

  // --- Activate Settings ---
  // TCSANOW: apply changes immediately
  // TCSAFLUSH: flush input and output queues and apply changes (good practice
  // after configuring)
  if (tcsetattr(fd_, TCSAFLUSH, &options) == -1) {
    last_errno_ = errno;
    throw std::runtime_error("tcsetattr failed for device " + device_name_ +
                             ": " + std::string(strerror(last_errno_)));
  }

  // --- Calculate Byte Time ---
  // Calculate approximate time to transmit one byte. Used potentially for write
  // timeouts. Need a reliable way to get numerical baud rate from speed_t
  // constant. Assuming a helper get_numerical_baudrate(speed_t constant) ->
  // uint32_t exists. For demonstration, using a simplified approximate
  // calculation based on a potentially available numerical value.
  // DANGEROUS - speed_t is not always numerical baud rate
  uint32_t numerical_baud = static_cast<uint32_t>(baud_rate_);
  // **Real solution needs a reliable speed_t to numerical baud rate mapping.**
  // Example: uint32_t numerical_baud = get_numerical_baudrate(baud_rate_);

  if (numerical_baud > 0) {
    // Total bits per char approx = 1 start + data bits + parity + stop bits
    // Common approximation is 10 bits/byte.
    uint32_t total_bits_per_char = 10;
    // microseconds per char
    byte_time_us_ = (total_bits_per_char * 1000000) / numerical_baud;
    AINFO << "Calculated byte time for device " << device_name_ << " at baud "
          << numerical_baud << " is approx " << byte_time_us_ << " us.";
  } else {
    byte_time_us_ = 0;
    AERROR << "Cannot calculate byte time for device " << device_name_
           << " due to unknown or zero baud rate value.";
  }
}

// Helper function to close the serial port. Safe to call multiple times.
void SerialStream::close(void) {
  if (fd_ >= 0) {  // Check if port is open
    AINFO << "Closing serial port " << device_name_ << " fd: " << fd_;
    // Flush any pending output before closing
    tcflush(fd_, TCOFLUSH);

    if (::close(fd_) < 0) {
      last_errno_ = errno;
      AERROR << "Failed to close serial port " << device_name_ << " fd " << fd_
             << ": " << strerror(last_errno_);
      // Don't throw on close failure in destructor or Disconnect.
    }
    fd_ = -1;
    is_open_ = false;
  }
  status_ = Stream::Status::DISCONNECTED;
}

// Connect: Opens and configures the serial port.
// Returns true if successful or already connected.
// Returns false on connection failure (when not throwing).
// Throws std::runtime_error for fatal setup errors during open/configure.
bool SerialStream::Connect() {
  if (is_open_) {
    status_ = Stream::Status::CONNECTED;  // Ensure status is correct
    return true;
  }

  last_errno_ = 0;  // Reset error code for this attempt

  try {
    // open() handles opening the FD and calls configure_port().
    // Both open() and configure_port() throw on failure.
    this->open();
    status_ = Stream::Status::CONNECTED;
    AINFO << "Serial port " << device_name_ << " connected successfully.";
    last_errno_ = 0;  // Clear error on success
    return true;

  } catch (const std::exception& e) {
    // Catch exceptions thrown by open() or configure_port()
    AERROR << "Failed to Connect to serial port " << device_name_ << ": "
           << e.what();
    // fd_ might already be -1 if configure_port failed after open, but ensure
    // it.
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
    is_open_ = false;
    status_ = Stream::Status::ERROR;
    return false;  // Connection failed
  }
}

bool SerialStream::Disconnect() {
  if (!is_open_) {
    status_ = Stream::Status::DISCONNECTED;
    return true;  // Already disconnected
  }

  AINFO << "Disconnecting serial port " << device_name_ << " fd: " << fd_;
  this->close();  // close() sets fd_ = -1, is_open_=false, and updates status_
  status_ = Stream::Status::DISCONNECTED;
  last_errno_ = 0;  // Clear last error on successful disconnect
  return true;
}

// Heuristic check to detect if the device has been removed.
// Attempts a zero-byte write. If certain errors occur (EBADF, EIO),
// logs, disconnects, and attempts to reconnect.
// Returns true if device likely removed and disconnected.
// Returns false otherwise.
bool SerialStream::check_remove() {
  if (!is_open_ || fd_ < 0) return false;  // Cannot check if not open

  char data = 0;
  // Attempt a zero-byte write to provoke an EBADF or EIO if the device is gone.
  // Using ::write with 0 length is a common trick.
  ssize_t nsent = ::write(fd_, &data, 0);

  if (nsent < 0) {
    // Error occurred. Check if it indicates device removal.
    switch (errno) {
      case EBADF:  // Bad file descriptor
      case EIO:    // I/O error (common when device is removed)
      case ENXIO:  // No such device or address (less common on write after
                   // open)
        // case EACCES: // Permissions error (less common after successful open)
        last_errno_ = errno;
        AERROR << "Serial stream detect write failed for device "
               << device_name_ << ", error: " << strerror(last_errno_)
               << ". Device likely removed.";
        // Device is likely removed, disconnect and attempt reconnect.
        Disconnect();
        // Note: Reconnect is attempted by the caller (read/write loop) if
        // enabled.
        return true;  // Indicate device was likely removed
      case EAGAIN:    // Same as EWOULDBLOCK
      case EINTR:     // Interrupted system call - shouldn't happen for 0 bytes
        // These are not device removal errors for a 0-byte write.
        // Log a warning if seen, but don't treat as removed.
        AINFO << "Serial check_remove got unexpected error " << strerror(errno)
              << " for 0-byte write.";
        return false;
      default:
        last_errno_ = errno;
        AERROR << "Serial stream detect write failed with unhandled error for "
                  "device "
               << device_name_ << ", error: " << strerror(last_errno_)
               << " (errno: " << last_errno_ << ").";
        // Could be a temporary error or a different fatal error.
        // Decide if this state should also trigger a Disconnect/Reconnect.
        // For now, just return false and let read/write handle other errors.
        return false;
    }
  }
  return false;
}

// Read data from the serial port.
// Throws std::runtime_error for fatal read errors or remote closed/device
// removed. Returns number of bytes read on success (> 0). Returns 0 on timeout
// or non-blocking with no data (EAGAIN/EWOULDBLOCK). Returns 0 if device not
// open and auto_reconnect fails or is disabled.
size_t SerialStream::read(uint8_t* buffer, size_t max_length) {
  // Attempt auto-connect/reconnect if needed and enabled (Stream base likely
  // handles this) Original code pattern: if (!is_open_) { if (!Connect())
  // return 0; } Let's keep this pattern, assuming Stream base doesn't handle
  // auto-connect on read/write.
  if (!is_open_) {
    last_errno_ = ENOTCONN;  // Set error before attempting connect
    if (!Connect()) {
      AERROR << "Read called when not connected and failed to connect.";
      return 0;
    }
    AINFO << "Read called on disconnected port, connected successfully.";
  }

  if (buffer == nullptr || max_length == 0) {
    return 0;  // Nothing to read into
  }

  ssize_t bytes_current_read = 0;

  // Use wait_readable to handle timeout or non-blocking wait based on
  // timeout_usec_ wait_readable returns false on timeout or select error. It
  // sets last_errno_ on select error (r < 0).
  if (!wait_readable(timeout_usec_)) {
    // wait_readable returned false. Check if it was an error or a timeout.
    if (last_errno_ != 0 &&
        last_errno_ != ETIMEDOUT) {  // Check if wait failed with a real error
                                     // (not just timeout)
      AERROR << "Serial read wait_readable failed with select error: "
             << strerror(last_errno_) << " (errno: " << last_errno_
             << "), fd: " << fd_;
      status_ = Stream::Status::ERROR;
      Disconnect();  // Clean up
      throw std::runtime_error("Serial read wait failed: " +
                               std::string(strerror(last_errno_)));
    }
    check_remove();
    return 0;
  }

  // Data is ready (wait_readable returned true)
  // Read available data. With VMIN=0, VTIME=0, read will return immediately
  // with however many bytes are in the buffer (up to max_length), or 0 if
  // buffer emptied *after* wait_readable indicated data was coming (race
  // condition possible but rare). Use a loop to handle EINTR in read itself,
  // although less likely after pselect.
  do {
    bytes_current_read = ::read(fd_, buffer, max_length);
  } while (bytes_current_read < 0 && errno == EINTR);

  if (bytes_current_read < 0) {
    // Error during the actual read() call
    switch (errno) {
      case EAGAIN:  // Same as EWOULDBLOCK
        // Treat as no data read in this instant, although unexpected.
        last_errno_ = errno;  // Store for debugging
        AINFO << "Serial read returned EAGAIN/EWOULDBLOCK after wait_readable. "
                 "FD: "
              << fd_ << ", Error: " << strerror(last_errno_);
        return 0;   // Return 0 bytes read
      case EINVAL:  // Invalid argument (shouldn't happen with valid
                    // fd/buffer/length)
        last_errno_ = errno;
        AERROR << "Serial read returned EINVAL. FD: " << fd_
               << ", Error: " << strerror(last_errno_);
        // Fatal error
        Disconnect();  // Clean up
        throw std::runtime_error("Serial read fatal error (EINVAL): " +
                                 std::string(strerror(last_errno_)));
      case EBADF:  // Bad file descriptor - socket likely closed underneath
      case EIO:    // I/O error - device likely removed
        last_errno_ = errno;
        AERROR << "Serial stream read data failed, likely disconnected or "
                  "removed. Error: "
               << strerror(last_errno_) << " (errno: " << last_errno_
               << "), fd: " << fd_;
        status_ = Stream::Status::ERROR;
        Disconnect();  // Clean up the broken fd
        // Throw exception to signal fatal error
        throw std::runtime_error("Serial read fatal error: " +
                                 std::string(strerror(last_errno_)));
      default:
        last_errno_ = errno;
        AERROR << "Serial stream read data failed with unhandled error: "
               << strerror(last_errno_) << " (errno: " << last_errno_
               << "), fd: " << fd_;
        status_ = Stream::Status::ERROR;
        Disconnect();  // Clean up in case it's fatal
        throw std::runtime_error("Serial read unhandled fatal error: " +
                                 std::string(strerror(last_errno_)));
    }
  }

  if (bytes_current_read == 0) {
    // read returned 0 bytes. With VMIN=0, VTIME=0 after wait_readable,
    // this means no data was in the buffer at that exact moment, OR port
    // closed. Treat as no data read in this call. check_remove provides a
    // heuristic for closed.
    last_errno_ = 0;  // Clear errno for successful 0-byte read
    status_ = Stream::Status::CONNECTED;
    AINFO << "Serial read returned 0 bytes after wait_readable indicated data. "
             "FD: "
          << fd_ << ". Checking device status.";
    // Call check_remove() to see if it indicates disconnection
    if (check_remove()) {
      // check_remove already logged, disconnected, and potentially attempted
      // reconnect. If check_remove returned true, the device is gone. Throw
      // exception.
      throw std::runtime_error("Serial read detected device removed.");
    }
    return 0;
  }

  // Success: bytes_current_read > 0
  // Return the actual number of bytes read in this call.
  return bytes_current_read;
}

// Write data to the serial port.
// Throws std::runtime_error for fatal write errors.
// Returns number of bytes written on success (equal to length if successful).
// Returns total bytes sent so far on timeout (EAGAIN/EWOULDBLOCK) or fatal
// error during partial write. Returns 0 if device not open and auto_reconnect
// fails or is disabled.
size_t SerialStream::write(const uint8_t* data, size_t length) {
  // Attempt auto-connect/reconnect if needed and enabled
  if (!is_open_) {
    last_errno_ = ENOTCONN;  // Set error before attempting connect
    if (!Connect()) {
      AERROR << "Write called when not connected and failed to connect.";
      return 0;
    }
    AINFO << "Write called on disconnected port, connected successfully.";
  }

  if (data == nullptr || length == 0) {
    return 0;  // Nothing to write
  }

  size_t total_nsent = 0;
  const uint8_t* current_data = data;
  size_t remaining_length = length;

  // Write loop for partial writes (possible depending on kernel buffer)
  while (remaining_length > 0) {
    ssize_t nsent;
    // Use a loop for EINTR
    do {
      nsent = ::write(fd_, current_data, remaining_length);
    } while (nsent < 0 && errno == EINTR);

    if (nsent < 0) {
      // Handle errors other than EINTR
      if (errno == EAGAIN) {
        // Non-blocking socket cannot send immediately (buffer full)
        // Need to wait for buffer space using wait_writable.
        last_errno_ = errno;  // Store EAGAIN/EWOULDBLOCK
        AINFO << "Serial write would block (EAGAIN/EWOULDBLOCK) after sending "
              << total_nsent << " bytes. Waiting for buffer. FD: " << fd_;
        // Use wait_writable with timeout_usec_
        if (wait_writable(timeout_usec_)) {
          // Buffer is writable, continue the write loop to retry send
          last_errno_ = 0;  // Clear error if wait succeeded
          continue;
        } else {
          // wait_writable timed out or had an error.
          // last_errno_ is set by wait_writable on error/timeout.
          AERROR << "Serial write wait_writable failed after sending "
                 << total_nsent << " bytes. Error: " << strerror(last_errno_)
                 << " (errno: " << last_errno_ << "), fd: " << fd_;
          // Decide if wait_writable failure on EAGAIN should be fatal.
          // Returning total_nsent indicates partial write and couldn't write
          // more within timeout. Check if wait_writable failed with a real
          // select error or just timeout.
          if (last_errno_ != EAGAIN && last_errno_ != ETIMEDOUT &&
              last_errno_ != 0) {
            // This is an error *during* the wait (not timeout), or a non-AGAIN
            // error from send.
            status_ = Stream::Status::ERROR;
            Disconnect();  // Clean up
            throw std::runtime_error("Serial write wait_writable failed: " +
                                     std::string(strerror(last_errno_)));
          }
          if (last_errno_ == ETIMEDOUT) {
            AWARN << "serial write timedout, disconnect it, it will attempt to "
                     "re-connect next time.";
            Disconnect();
          }
          // If we are here, it was EAGAIN on write, and wait_writable timed out
          // or got EAGAIN/EWOULDBLOCK. Return partial write count. Caller
          // should handle retry.
          return total_nsent;
        }
      } else {
        // Other serious errors (e.g., EIO, EBADF, ENXIO etc.)
        last_errno_ = errno;
        AERROR << "Serial stream write fatal error: " << strerror(last_errno_)
               << " (errno: " << last_errno_ << "), fd: " << fd_;
        status_ = Stream::Status::ERROR;
        Disconnect();  // Clean up
        // Throw exception after sending partial data.
        throw std::runtime_error("Serial write fatal error: " +
                                 std::string(strerror(last_errno_)));
      }
    }

    // nsent >= 0 : Bytes were sent (could be 0 if length was 0 or buffer full
    // with non-blocking, but handled by EAGAIN above)
    total_nsent += nsent;
    current_data += nsent;
    remaining_length -= nsent;

    // If nsent == 0 and not EAGAIN, this is strange. Could be an error.
    // Treat similar to EAGAIN - wait for writable and retry.
    if (nsent == 0 && remaining_length > 0) {
      AINFO << "Serial write returned 0 bytes (not EAGAIN) after sending "
            << total_nsent << " bytes. Waiting for buffer. FD: " << fd_;
      last_errno_ = 0;  // Clear errno as 0 return isn't necessarily error
      if (wait_writable(timeout_usec_)) {
        continue;  // Retry write loop
      } else {
        // wait_writable failed or timed out
        AERROR << "Serial write wait_writable failed after 0-byte write: "
               << strerror(last_errno_) << " (errno: " << last_errno_
               << "), fd: " << fd_;
        if (last_errno_ != EAGAIN && last_errno_ != ETIMEDOUT &&
            last_errno_ != 0) {  // Real select error?
          Disconnect();          // Clean up
          throw std::runtime_error(
              "Serial write wait_writable failed after 0-byte write: " +
              std::string(strerror(last_errno_)));
        }
        // Timeout or EAGAIN from wait. Return partial write count.
        return total_nsent;
      }
    }
  }

  // If loop completes, all bytes were sent.
  // last_errno_ = 0; // Clear error on success
  return total_nsent;  // Should be equal to original 'length'
}

// Waits for the file descriptor to become readable using pselect.
// Returns true if readable, false on timeout or error.
// Sets last_errno_ on select error (r < 0) or timeout (ETIMEDOUT).
bool SerialStream::wait_readable(uint32_t timeout_us) {
  if (fd_ < 0) {
    last_errno_ = EBADF;  // Indicate bad file descriptor
    return false;         // Cannot wait on invalid fd
  }

  timespec timeout_ts;
  fd_set readfds;
  FD_ZERO(&readfds);
  FD_SET(fd_, &readfds);

  // Correct pselect call: pass &timeout_ts only if timeout_us > 0 for a finite
  // timeout. If timeout_us == 0, {0,0} timespec is used for immediate check
  // (non-blocking wait).
  struct timespec immediate_timeout = {0, 0};  // For timeout_us == 0 case
  const struct timespec* pselect_timeout_ptr =
      (timeout_us == 0) ? &immediate_timeout : &timeout_ts;

  if (timeout_us > 0) {
    timeout_ts.tv_sec = timeout_us / 1000000;
    timeout_ts.tv_nsec = (timeout_us % 1000000) * 1000;
  }

  // Use pselect for signal-safe wait
  int r = pselect(fd_ + 1, &readfds, NULL, NULL, pselect_timeout_ptr, NULL);

  if (r < 0) {
    // Error during select/pselect call (e.g., bad file descriptor, signal)
    if (errno == EINTR) {
      // Interrupted by signal. Return false. Caller can decide to loop or
      // handle.
      last_errno_ = errno;
      AINFO << "Serial read wait_readable interrupted by signal (EINTR).";
      return false;  // Interrupted
    } else {
      last_errno_ = errno;
      AERROR << "Serial read wait_readable failed with select error: "
             << strerror(last_errno_) << " (errno: " << last_errno_
             << "), fd: " << fd_;
      status_ = Stream::Status::ERROR;
      // Fatal select error. Return false. Caller should check last_errno_.
      return false;  // Error
    }
  } else if (r == 0) {
    // Timeout occurred
    last_errno_ = ETIMEDOUT;  // Use ETIMEDOUT to indicate timeout
    status_ = Stream::Status::CONNECTED;
    return false;  // Timeout
  } else {         // r > 0
    // File descriptor is ready
    // This shouldn't happen if r > 0 our fd has to be in the list, but
    // defensive check
    if (FD_ISSET(fd_, &readfds)) {
      last_errno_ = 0;  // Clear error on success
      return true;      // Readable
    } else {
      // Should not be reachable if r > 0 for a single FD set.
      // Indicates an unexpected state or issue with the file descriptor.
      last_errno_ = EBADF;  // Indicate unexpected state / bad fd
      AERROR << "Serial read wait_readable: pselect returned > 0 but FD_ISSET "
                "is false. Unexpected. FD: "
             << fd_;
      status_ = Stream::Status::ERROR;
      return false;  // Unexpected state / Error
    }
  }
}

// Waits for the file descriptor to become writable using pselect.
// Returns true if writable, false on timeout or error.
// Sets last_errno_ on select error (r < 0) or timeout (ETIMEDOUT).
bool SerialStream::wait_writable(uint32_t timeout_us) {
  if (fd_ < 0) {
    last_errno_ = EBADF;  // Indicate bad file descriptor
    return false;         // Cannot wait on invalid fd
  }

  timespec timeout_ts;
  fd_set writefds;
  FD_ZERO(&writefds);
  FD_SET(fd_, &writefds);

  // Correct pselect call: pass &timeout_ts only if timeout_us > 0 for a finite
  // timeout. If timeout_us == 0, {0,0} timespec is used for immediate check
  // (non-blocking wait).
  struct timespec immediate_timeout = {0, 0};  // For timeout_us == 0 case
  const struct timespec* pselect_timeout_ptr =
      (timeout_us == 0) ? &immediate_timeout : &timeout_ts;

  if (timeout_us > 0) {
    timeout_ts.tv_sec = timeout_us / 1000000;
    timeout_ts.tv_nsec = (timeout_us % 1000000) * 1000;
  }

  // Use pselect for signal-safe wait
  // Only ONE pselect call.
  int r = pselect(fd_ + 1, NULL, &writefds, NULL, pselect_timeout_ptr, NULL);

  if (r < 0) {
    // Error during select/pselect call
    if (errno == EINTR) {
      last_errno_ = errno;
      AINFO << "Serial write wait_writable interrupted by signal (EINTR).";
      return false;  // Interrupted
    } else {
      last_errno_ = errno;
      AERROR << "Serial write wait_writable failed with select error: "
             << strerror(last_errno_) << " (errno: " << last_errno_
             << "), fd: " << fd_;
      status_ = Stream::Status::ERROR;
      return false;  // Error
    }
  } else if (r == 0) {
    // Timeout occurred
    last_errno_ = ETIMEDOUT;  // Use ETIMEDOUT
    status_ = Stream::Status::CONNECTED;
    return false;  // Timeout
  } else {         // r > 0
                   // File descriptor is ready
    if (FD_ISSET(fd_, &writefds)) {
      last_errno_ = 0;  // Clear error
      return true;      // Writable
    } else {
      last_errno_ = EBADF;  // Indicate unexpected state / bad fd
      AERROR << "Serial write wait_writable: pselect returned > 0 but FD_ISSET "
                "is false. Unexpected. FD: "
             << fd_;
      status_ = Stream::Status::ERROR;
      return false;  // Unexpected state / Error
    }
  }
}

}  // namespace hal
}  // namespace drivers
}  // namespace apollo
