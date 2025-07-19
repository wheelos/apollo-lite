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

#include "modules/drivers/hal/stream/ntrip_stream.h"

#include <chrono>

#include "cyber/cyber.h"

namespace apollo {
namespace drivers {
namespace hal {

// Constructor: Initializes parameters and creates the underlying TcpStream.
// Throws std::runtime_error if TcpStream construction fails.
NtripStream::NtripStream(const std::string& address, uint16_t port,
                         const std::string& mountpoint, const std::string& user,
                         const std::string& passwd, uint32_t timeout_s)
    : mountpoint_(mountpoint),
      address_(address),
      port_(port),
      write_data_prefix_("GET /" + mountpoint +
                         " HTTP/1.1\r\n"
                         "User-Agent: NTRIP gnss_driver/0.0\r\n"
                         "accept: */* \r\n\r\n"),
      login_data_("GET /" + mountpoint +
                  " HTTP/1.1\r\n"
                  "User-Agent: NTRIP gnss_driver/0.0\r\n"
                  "accept: */* \r\n"
                  "Authorization: Basic " +
                  common::util::EncodeBase64(user + ":" + passwd) +
                  "\r\n"
                  "Connection: keep-alive\r\n"
                  "\r\n"),
      timeout_s_(timeout_s) {
  // Construct the initial GET request string (with or without authentication)
  std::string auth_header;
  if (!user.empty() || !passwd.empty()) {
    auth_header = "Authorization: Basic " +
                  common::util::EncodeBase64(user + ":" + passwd) + "\r\n";
  }

  // Create the underlying TcpStream. Use timeout_usec = 0 for non-blocking
  // reads/writes handled by NtripStream's select/pselect logic or its own
  // timeouts. Let's pass a small timeout to TcpStream constructor, or use its
  // default? Original used 0. Using 0 makes TcpStream non-blocking. NtripStream
  // will manage the blocking/timeout using wait_readable/wait_writable if
  // needed. Let's configure TcpStream with the *data activity timeout* for its
  // read/write, or keep it non-blocking and handle waiting here. The most
  // common is NtripStream handles timeouts. Let's pass 0 to TcpStream.
  try {
    // The TcpStream constructor should throw on invalid address/port format or
    // other setup errors.
    // use timeout_s_ as connect timeout for non-blocking issue in TcpStream.
    // TODO(All): fix issue in tcp_stream and re-enable non-blocking
    // TODO(All): distinguish read/write timeout and connection timeout
    tcp_stream_ = std::make_unique<TcpStream>(
        address_, port_,
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::seconds(timeout_s_))
            .count(),
        false);
  } catch (const std::exception& e) {
    throw std::runtime_error("NtripStream failed to create TcpStream: " +
                             std::string(e.what()));
  }

  AINFO << "NtripStream created for " << address_ << ":" << port_ << "/"
        << mountpoint_;
}

// Destructor: Disconnects the stream.
NtripStream::~NtripStream() {
  AINFO << "NtripStream destructing for " << address_ << ":" << port_ << "/"
        << mountpoint_ << ", disconnecting.";
  Disconnect();  // Safe to call even if not connected
}

// Connect: Establishes the TCP connection and performs the NTRIP handshake.
// Throws std::runtime_error for fatal errors during setup or handshake.
// Returns true on success. Returns false on failure (when not throwing).
bool NtripStream::Connect() {
  std::lock_guard<std::mutex> lock(internal_mutex_);

  if (is_login_) {
    AINFO << "NtripStream already logged in.";
    status_ = Stream::Status::CONNECTED;
    return true;
  }

  AINFO << "Attempting to connect and login to NTRIP caster " << address_ << ":"
        << port_ << "/" << mountpoint_;
  last_errno_ = 0;  // Reset error code

  // Ensure tcp_stream_ is valid before use (should be valid due to constructor
  // exception handling)
  if (!tcp_stream_) {
    last_errno_ = EBADF;  // Use EBADF to indicate invalid internal state
    AERROR << "NtripStream internal tcp_stream_ is invalid.";
    status_ = Stream::Status::ERROR;
    return false;  // Cannot connect if internal state is bad
  }

  try {
    // 1. Connect the underlying TCP stream.
    // TcpStream::Connect should throw on fatal setup/connect errors, return
    // false on timeout/refused.
    if (!tcp_stream_->Connect()) {
      // tcp_stream_->Connect returned false (e.g., timeout, connection refused)
      last_errno_ = tcp_stream_->get_last_error_code();
      AERROR << "NtripStream failed to connect TCP: " << strerror(last_errno_)
             << " (errno: " << last_errno_ << ").";
      status_ = Stream::Status::DISCONNECTED;
      // set this, but ensure
      is_login_ = false;  // Ensure login state is false
      return false;  // Connection failed (gracefully reported by TcpStream)
    }

    // If TcpStream::Connect succeeded (returned true), proceed with NTRIP
    // handshake.

    // 2. Send the NTRIP GET request (with or without authentication).
    // TcpStream::write throws on fatal errors.
    size_t sent_size =
        tcp_stream_->write(reinterpret_cast<const uint8_t*>(login_data_.data()),
                           login_data_.size());
    if (sent_size != login_data_.size()) {
      // This case might indicate a partial write which TcpStream::write should
      // handle, or it could indicate an error before throwing. The optimized
      // TcpStream::write throws on fatal errors and returns partial size on
      // EAGAIN/timeout. If it returns less than size *without* throwing, it
      // implies EAGAIN/timeout. For handshake, we need the whole request sent.
      last_errno_ = tcp_stream_->get_last_error_code();  // Get specific error
      AERROR << "NtripStream failed to send full GET request during handshake. "
                "Sent: "
             << sent_size << ", expected: " << login_data_.size()
             << ". Error: " << strerror(last_errno_);
      tcp_stream_->Disconnect();  // Clean up connection
      status_ = Stream::Status::ERROR;
      is_login_ = false;
      // Throw indicating handshake failure
      throw std::runtime_error(
          "NtripStream failed to send full handshake request.");
    }
    AINFO << "NtripStream sent GET request.";

    // 3. Read and parse the HTTP response header.
    std::string response_header = read_http_header_locked(timeout_s_);

    AINFO << "NtripStream received response header (first few lines):";
    // Log first few lines of header
    size_t end_of_first_line = response_header.find('\n');
    if (end_of_first_line != std::string::npos) {
      AINFO << response_header.substr(0, end_of_first_line + 1);
      size_t end_of_second_line =
          response_header.find('\n', end_of_first_line + 1);
      if (end_of_second_line != std::string::npos) {
        AINFO << response_header.substr(end_of_first_line + 1,
                                        end_of_second_line - end_of_first_line);
      }
    } else {
      AINFO << response_header.substr(
          0, std::min(response_header.size(),
                      (size_t)100));  // Log up to 100 chars if no newline
    }

    // 4. Parse the response header to check for success/failure.
    bool handshake_success = false;
    if (response_header.find("ICY 200 OK\r\n") != std::string::npos) {
      // Successful NTRIP stream handshake
      handshake_success = true;
      AINFO << "Ntrip login successfully to mountpoint " << mountpoint_ << ".";
    } else if (response_header.find("SOURCETABLE 200 OK\r\n") !=
               std::string::npos) {
      // Received Source Table instead of stream (mountpoint probably wrong)
      AERROR << "NtripStream received Source Table. Mountpoint '" << mountpoint_
             << "' may not exist or is incorrect.";
      // This is a handshake failure, but not a network error. Log and fail.
    } else if (response_header.find("HTTP/") != std::string::npos) {
      // Received a standard HTTP error response (e.g., 401 Unauthorized, 404
      // Not Found) Check for specific codes if needed, but general HTTP
      // response indicates failure.
      AERROR << "NtripStream received HTTP error response during handshake.";
      // Log the relevant part of the response if possible
      size_t http_start = response_header.find("HTTP/");
      size_t end_of_status_line = response_header.find('\n', http_start);
      if (http_start != std::string::npos &&
          end_of_status_line != std::string::npos) {
        AERROR << "Status line: "
               << response_header.substr(http_start,
                                         end_of_status_line - http_start);
      }
      // This is a handshake failure.
    } else {
      // Unexpected response
      AERROR << "NtripStream received unexpected response during handshake.";
      // Log the response for debugging
      AERROR << "Full unexpected response header:\n" << response_header;
    }

    // 5. Finalize connection state based on handshake result.
    if (handshake_success) {
      is_login_ = true;
      status_ = Stream::Status::CONNECTED;
      data_active_s_ =
          cyber::Time::Now().ToSecond();  // Initialize data activity timestamp
      last_errno_ = 0;                    // Clear error
      return true;                        // Connect successful
    } else {
      // Handshake failed based on response parsing.
      tcp_stream_->Disconnect();  // Clean up TCP connection
      status_ = Stream::Status::ERROR;
      is_login_ = false;  // Ensure login state is false
      last_errno_ =
          EPROTO;  // Use EPROTO (Protocol error) or a custom error code
      // Throw exception indicating handshake failure
      throw std::runtime_error("NtripStream handshake failed.");
      // return false; // Could return false instead of throwing for handshake
      // failure if desired, but throwing is clearer.
    }

  } catch (const std::exception& e) {
    last_errno_ = tcp_stream_ ? tcp_stream_->get_last_error_code() : EBADF;
    AERROR << "NtripStream Connect failed: " << e.what()
           << " (Underlying errno: " << last_errno_ << ", "
           << strerror(last_errno_) << ").";
    // Ensure disconnect and status update
    if (tcp_stream_) {
      tcp_stream_->Disconnect();
    }
    is_login_ = false;
    status_ = Stream::Status::ERROR;

    // Decide whether to rethrow the exception or just return false.
    // Throwing is generally better for signaling *why* it failed setup/connect.
    throw std::runtime_error("NtripStream Connect failed: " +
                             std::string(e.what()));
  }
}

bool NtripStream::Disconnect() {
  std::lock_guard<std::mutex> lock(
      internal_mutex_);  // Lock during disconnection

  if (!is_login_) {
    status_ = Stream::Status::DISCONNECTED;
    AINFO << "NtripStream already disconnected.";
    return true;
  }

  AINFO << "Disconnecting NTRIP stream from " << address_ << ":" << port_ << "/"
        << mountpoint_;
  is_login_ = false;  // Set login state to false immediately

  bool ret = true;
  if (tcp_stream_) {
    ret = tcp_stream_->Disconnect();  // Disconnect underlying TCP stream
    if (!ret) {
      last_errno_ = tcp_stream_->get_last_error_code();
      AERROR << "NtripStream failed to disconnect underlying TCP stream: "
             << strerror(last_errno_);
    }
  } else {
    // tcp_stream_ is null, considered already disconnected
    AINFO << "NtripStream underlying tcp_stream_ is null, considered "
             "disconnected.";
  }

  status_ = Stream::Status::DISCONNECTED;
  last_errno_ = 0;  // Clear last error on successful disconnect

  return ret;
}

// Attempts to disconnect and reconnect the NTRIP stream.
// Called internally on timeout or read/write errors.
// Assumes the caller holds the internal_mutex_ lock.
void NtripStream::Reconnect() {
  // Mutex is already locked by the caller (read or write)
  AINFO << "Reconnect: Attempting to reconnect ntrip caster " << address_ << ":"
        << port_ << "/" << mountpoint_;

  // Disconnect the current stream state
  // No need for lock guard here as we already hold the lock.
  // Disconnect() method acquires lock internally, avoid double locking issues
  // if possible. Or, make Disconnect_locked helper that assumes lock is held.
  // Let's call the public Disconnect(). If it acquires lock internally, it's
  // fine. Need to release lock before calling Disconnect() if Disconnect()
  // acquires lock internally. A pattern could be: {
  // std::unique_lock<std::mutex> unlock(internal_mutex_); unlock.unlock();
  // Disconnect(); } lock.lock(); Or, make Disconnect_internal() that doesn't
  // lock. Let's assume Disconnect() is safe to call even if lock is held (e.g.,
  // tries to acquire lock, or detects it's held). Or simplify: just call
  // close() on the underlying tcp_stream_
  if (tcp_stream_) {
    tcp_stream_->Disconnect();  // Call disconnect on underlying stream
  }
  is_login_ = false;  // Ensure login state is false
  status_ = Stream::Status::DISCONNECTED;

  // Attempt to connect again
  try {
    // Connect() acquires the lock internally, so need to release it here first.
    // Releasing and re-acquiring the lock around the Connect call in
    // Reconnect is crucial to avoid deadlock if Connect needs the lock
    // internally, and to allow other threads to potentially acquire the lock if
    // Reconnect blocks. Let's pass the lock to Connect, or ensure Connect
    // doesn't block on the same lock. Simpler: Call Connect() which handles its
    // own locking (if designed to be externally callable and thread-safe). If
    // Connect() *must* be called with the mutex locked, then Reconnect
    // needs Connect_locked helper. Let's assume Connect() can be called
    // externally and handles its own required locking. So, release the lock
    // *around* the Connect call.
    std::unique_lock<std::mutex> relock(internal_mutex_);
    relock.unlock();  // Release lock before calling Connect

    bool connect_success = Connect();

    relock.lock();  // Re-acquire lock after Connect returns or throws

    if (connect_success) {
      data_active_s_ = cyber::Time::Now().ToSecond();
      AINFO << "Reconnect: Reconnected successfully.";
    } else {
      // Connect returned false
      AERROR << "Reconnect: Connect attempt returned false.";
    }

  } catch (const std::exception& e) {
    // Catch exception from Connect attempt
    last_errno_ = tcp_stream_ ? tcp_stream_->get_last_error_code() : EBADF;
    AERROR << "Reconnect: Connect attempt threw: " << e.what()
           << " (Underlying errno: " << last_errno_ << ", "
           << strerror(last_errno_) << ").";
  }
}

// Reads the HTTP response header from the TCP stream until "\r\n\r\n" or
// timeout. Assumes the caller holds the internal_mutex_ lock. Throws
// std::runtime_error on read error, timeout during header read, or if
// connection closes prematurely. Returns the received header string.
std::string NtripStream::read_http_header_locked(uint32_t timeout_s) {
  if (!tcp_stream_ || !tcp_stream_->Connect()) {
    last_errno_ = ENOTCONN;
    throw std::runtime_error(
        "read_http_header_locked called when tcp stream is not connected.");
  }

  auto start_time = std::chrono::steady_clock::now();
  auto deadline = start_time + std::chrono::seconds(timeout_s);

  const std::string delimiter = "\r\n\r\n";
  const size_t max_header_size = 8192;

  std::string header_buffer;
  header_buffer.reserve(2048);

  std::vector<uint8_t> tmp_buf(512);

  constexpr uint32_t poll_interval_us = 10000;

  while (true) {
    if (header_buffer.find(delimiter) != std::string::npos) {
      break;
    }

    auto now = std::chrono::steady_clock::now();
    if (now > deadline) {
      last_errno_ = ETIMEDOUT;
      throw std::runtime_error("read_http_header_locked timed out.");
    }

    if (header_buffer.size() > max_header_size) {
      last_errno_ = EMSGSIZE;
      throw std::runtime_error(
          "read_http_header_locked exceeded max header size.");
    }

    bool can_read = false;
    try {
      can_read = tcp_stream_->Readable(poll_interval_us);
    } catch (const std::exception& e) {
      last_errno_ = tcp_stream_->get_last_error_code();
      throw std::runtime_error(
          std::string("read_http_header_locked failed in Readable(): ") +
          e.what());
    }

    if (!can_read) {
      continue;
    }

    ssize_t n = 0;
    try {
      n = tcp_stream_->read(tmp_buf.data(), tmp_buf.size());
    } catch (const std::exception& e) {
      last_errno_ = tcp_stream_->get_last_error_code();
      throw std::runtime_error(
          std::string("read_http_header_locked read() error: ") + e.what());
    }

    if (n > 0) {
      header_buffer.append(reinterpret_cast<char*>(tmp_buf.data()), n);
      last_errno_ = 0;
      continue;
    }

    if (n == 0) {
      last_errno_ = ECONNRESET;
      throw std::runtime_error(
          "read_http_header_locked: connection closed by remote peer.");
    }

    if (n < 0) {
      int err = tcp_stream_->get_last_error_code();
      if (err == EAGAIN || err == EWOULDBLOCK) {
        continue;
      }
      last_errno_ = err;
      throw std::runtime_error(
          std::string("read_http_header_locked fatal read error: ") +
          std::strerror(err));
    }
  }  // end while

  return header_buffer;
}

// Read data from the NTRIP stream.
size_t NtripStream::read(uint8_t* buffer, size_t max_length) {
  // tcp stream is duplex, so we can read and write concurrently. no need to
  // share the mutex between read and write operation
  // TODO(All): separator mutex and make thread safety for multiple read
  // std::lock_guard<std::mutex> lock(internal_mutex_);  // Lock for thread
  // safety

  // Check connection status and potentially reconnect
  if (!is_login_) {
    // Not logged in. Attempt reconnect. Reconnect handles its own state
    // and logging.
    Reconnect();
    if (!is_login_) {
      // Still not logged in after reconnect attempt.
      last_errno_ = ENOTCONN;  // Not connected
      AERROR
          << "NtripStream read failed: Not connected after reconnect attempt.";
      return 0;  // Cannot read
    }
    AINFO << "NtripStream read: Reconnected successfully.";
  }

  // Check if tcp_stream_ is still valid after reconnect
  if (!tcp_stream_) {
    last_errno_ = EBADF;
    AERROR << "NtripStream read failed: internal tcp_stream_ is null after "
              "connect check.";
    status_ = Stream::Status::ERROR;
    return 0;  // Cannot read
  }

  size_t ret = 0;
  last_errno_ = 0;  // Reset errno before read attempt

  try {
    // Delegate read to the underlying TcpStream.
    // TcpStream::read returns >0 on data, 0 on timeout/would block, throws on
    // fatal error/remote closed. The timeout_usec_ in TcpStream is 0
    // (non-blocking read kernel). This read call will return immediately with
    // available data or 0 if none.
    ret = tcp_stream_->read(buffer, max_length);

    if (ret > 0) {
      // Successfully read data, update activity timestamp
      data_active_s_ = cyber::Time::Now().ToSecond();
      last_errno_ = 0;  // Clear error
    } else if (ret == 0) {
      // Read returned 0. This indicates timeout or non-blocking with no data
      // available. TcpStream::read throws on remote closed.
      last_errno_ = tcp_stream_->get_last_error_code();
      status_ = Stream::Status::CONNECTED;
    } else {
      // ret < 0 is not expected from optimized TcpStream::read (it throws).
      // Defensive check, treat as error if it happens.
      last_errno_ = tcp_stream_->get_last_error_code();  // Get underlying error
      AERROR << "NtripStream read got unexpected negative return from "
                "tcp_stream_->read: "
             << ret << ". Underlying error: " << strerror(last_errno_);
      // Throw to signal unexpected state.
      Disconnect();  // Clean up
      throw std::runtime_error(
          "NtripStream read got unexpected return value from "
          "tcp_stream_->read.");
    }

  } catch (const std::exception& e) {
    // Catch exceptions from TcpStream::read (fatal error, remote closed)
    last_errno_ = tcp_stream_ ? tcp_stream_->get_last_error_code() : EBADF;
    AERROR << "NtripStream read failed from tcp_stream_: " << e.what()
           << " (Underlying errno: " << last_errno_ << ", "
           << strerror(last_errno_) << ").";
    // This is a fatal error, need to disconnect and potentially reconnect.
    Disconnect();  // Clean up
    status_ = Stream::Status::ERROR;
    is_login_ = false;  // Ensure login state is false

    // Reconnect is handled by the check at the start of the read method on the
    // *next* call. Throw exception to signal fatal error to the caller.
    throw std::runtime_error("NtripStream read fatal error: " +
                             std::string(e.what()));
  }

  // Check data activity timeout *after* handling the read result.
  // Use cyber::Time::Now().ToSecond() directly for current time.
  double current_time = cyber::Time::Now().ToSecond();
  if (current_time - data_active_s_ > timeout_s_) {
    AINFO << "NtripStream data activity timeout (" << timeout_s_
          << " s). Last data: " << data_active_s_
          << ", Current time: " << current_time;
    last_errno_ = ETIMEDOUT;  // Set timeout error
    // Reconnect if timeout occurs. Reconnect handles its own state.
    Reconnect();
    // Reconnect updates is_login_ and status_.
    // If reconnect fails, the next call to read will handle it.
    // Return 0 bytes read for this timed-out call.
    return 0;
  }

  // Return the number of bytes read (0 on timeout/would_block, >0 on data).
  return ret;
}

// Write data to the NTRIP stream (intended for NMEA commands).
// Throws std::runtime_error for fatal write errors.
// Returns number of bytes written on success (equal to length if successful).
// Returns total bytes sent so far on timeout (EAGAIN/EWOULDBLOCK) or fatal
// error during partial write. Returns 0 if not connected.
size_t NtripStream::write(const uint8_t* buffer, size_t length) {
  if (!tcp_stream_) {
    return 0;
  }
  std::unique_lock<std::mutex> lock(internal_mutex_, std::defer_lock);
  if (!lock.try_lock()) {
    AINFO << "Try lock failed.";
    return 0;
  }

  if (!is_login_ || tcp_stream_->get_status() != Stream::Status::CONNECTED) {
    AERROR << "NtripStream write failed: Not connected or not logged in.";
    return 0;
  }

  size_t ret = tcp_stream_->write(buffer, length);
  if (ret != length) {
    AERROR << "Send ntrip data size " << length << ", return " << ret;
    status_ = Stream::Status::ERROR;
    return 0;
  }

  return length;
}

}  // namespace hal
}  // namespace drivers
}  // namespace apollo
