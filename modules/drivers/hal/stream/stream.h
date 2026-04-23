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

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "cyber/cyber.h"

namespace apollo {
namespace drivers {
namespace hal {

// An abstract class of Stream.
class Stream {
 public:
  // =========================================================================
  // Stream Class Overview and Responsibilities
  // =========================================================================
  // This class serves as an abstract base class defining the interface
  // for various stream types (e.g., serial, network).
  //
  // Responsibilities of Derived Classes:
  // 1. Implement the pure virtual methods (Connect, Disconnect, read, write)
  //    according to the specific stream type.
  // 2. **Crucially, derived classes MUST update the 'status_' member**
  //    based on the results of connection attempts and read/write operations.
  //    - Set status_ to CONNECTED after a successful Connect.
  //    - Set status_ to DISCONNECTED after a successful Disconnect or clean
  //    closure.
  //    - Set status_ to ERROR if a non-recoverable error occurs during any
  //    operation.
  //
  // Thread Safety:
  // By default, instances of this class and its derived classes are NOT
  // thread-safe. If a single Stream object is accessed concurrently by
  // multiple threads, external synchronization (e.g., mutexes) is required,
  // or the derived class must provide its own internal synchronization.
  // =========================================================================
  virtual ~Stream() {}

  // Stream status.
  enum class Status {
    DISCONNECTED = 0,
    CONNECTED,
    ERROR,
  };

  static constexpr size_t NUM_STATUS =
      static_cast<int>(Stream::Status::ERROR) + 1;

  Status get_status() const { return status_; }

  // Attempts to connect the stream.
  // Derived classes must implement this method.
  // Returns true if connection was successful, false otherwise.
  // **Derived classes should update status_ to CONNECTED on success
  // or ERROR/DISCONNECTED on failure.**
  virtual bool Connect() = 0;

  // Attempts to disconnect the stream.
  // Derived classes must implement this method.
  // Returns true if disconnection was successful, false otherwise.
  // **Derived classes should update status_ to DISCONNECTED on success.**
  virtual bool Disconnect() = 0;

  // Registers data required for the login process.
  // This data is copied internally.
  void RegisterLoginData(const std::vector<std::string> &login_data) {
    login_data_.assign(login_data.begin(), login_data.end());
  }

  // Executes the login process by writing registered data sequentially
  // with delays. This method assumes a simple write-based login protocol.
  //
  // **This method is BLOCKING** due to the internal `cyber::Duration().Sleep()`
  // calls and the sequential nature of writing login data.
  //
  // Derived classes' `write` method must be correctly implemented for this
  // login logic to function.
  //
  // Returns true if all login data chunks were successfully written to the
  // stream buffer. Returns false if the stream is not connected, or if any
  // `write` operation fails or is incomplete for a login data chunk.
  //
  // Note: Success here only guarantees the data was sent to the stream
  // interface. It does NOT guarantee that the remote end processed the login
  // data successfully (which might require reading responses, not handled in
  // this base method).
  bool Login() {
    if (status_ != Status::CONNECTED) {
      AERROR << "Login failed: Stream is not CONNECTED. Current status: "
             << static_cast<int>(status_);
      return false;
    }

    for (size_t i = 0; i < login_data_.size(); ++i) {
      const auto &data = login_data_[i];
      size_t bytes_written = write(data);

      AINFO << "Login step " << (i + 1) << "/" << login_data_.size()
            << ": Attempted to write " << data.size() << " bytes, wrote "
            << bytes_written << " bytes.";

      if (bytes_written != data.size()) {
        // Error or partial write during this step of login
        AERROR << "Login failed at step " << (i + 1)
               << ": Could not write complete data chunk.";
        return false;  // Abort login sequence on first write failure
      }

      // sleep a little to avoid overrun of the slow serial interface.
      cyber::Duration(0.5).Sleep();
    }

    // All data chunks successfully written
    return true;
  }

  // Reads up to max_length bytes into the buffer.
  // Derived classes must implement this method.
  // Returns the number of bytes actually read (0 if no data available or
  // error).
  // **Derived classes should update status_ to ERROR or DISCONNECTED if a fatal
  // read error occurs.**
  virtual size_t read(uint8_t *buffer, size_t max_length) = 0;

  // Writes up to length bytes from the buffer.
  // Derived classes must implement this method.
  // Returns the number of bytes successfully written.
  // A return value less than 'length' or 0 indicates a partial write or
  // failure.
  // **Derived classes should update status_ to ERROR or DISCONNECTED if a fatal
  // write error occurs.**
  virtual size_t write(const uint8_t *buffer, size_t length) = 0;

  size_t write(const std::string &buffer) {
    return write(reinterpret_cast<const uint8_t *>(buffer.data()),
                 buffer.size());
  }

  // Get the system errno of the last significant error.
  // Returns 0 if no recent error or error was EAGAIN/EINTR.
  int get_last_error_code() const { return last_errno_; }

 protected:
  Stream() {}

  // The current status of the stream.
  // Derived classes are responsible for keeping this updated
  // based on the results of Connect, Disconnect, read, and write.
  Status status_ = Status::DISCONNECTED;

  int last_errno_ = 0;

 private:
  std::vector<std::string> login_data_;

  DISALLOW_COPY_AND_ASSIGN(Stream);
};

}  // namespace hal
}  // namespace drivers
}  // namespace apollo
