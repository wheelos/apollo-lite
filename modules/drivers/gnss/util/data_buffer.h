// Copyright 2025 WheelOS All Rights Reserved.
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

//  Created Date: 2025-4-15
//  Author: daohu527

#pragma once

#include <algorithm>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "cyber/common/macros.h"

namespace apollo {
namespace drivers {
namespace gnss {

/**
 * @class DataBuffer
 * @brief A high-performance, resizable data buffer optimized for stream I/O and
 * parsing.
 *
 * This buffer follows modern C++ practices, inspired by high-performance
 * networking libraries. It minimizes data copying by providing views into its
 * internal storage and allowing direct writes into its writable space.
 */
class DataBuffer {
 public:
  static constexpr size_t kDefaultInitialSize = 4096;
  static constexpr size_t kPrependSize = 8;

  explicit DataBuffer(size_t initial_size = kDefaultInitialSize)
      : storage_(kPrependSize + initial_size),
        read_pos_(kPrependSize),
        write_pos_(kPrependSize) {}

  // --- Read Operations ---

  /** @brief Returns the number of bytes available for reading. */
  size_t ReadableBytes() const { return write_pos_ - read_pos_; }

  /** @brief Checks if there is no data to read. */
  bool IsEmpty() const { return ReadableBytes() == 0; }

  /** @brief Returns a view of all readable data. Does not consume data. */
  std::string_view Peek() const {
    return std::string_view(reinterpret_cast<const char*>(BeginRead()),
                            ReadableBytes());
  }

  std::optional<uint8_t> Poll();

  /** @brief Finds the first occurrence of a pattern within the readable data.
   */
  std::optional<size_t> Find(std::string_view pattern) const;

  // --- Data Consumption (Retrieval) ---

  /** @brief Consumes (discards) `len` bytes from the read buffer. */
  void Drain(size_t len);

  /** @brief Consumes all readable data. */
  void DrainAll() {
    read_pos_ = kPrependSize;
    write_pos_ = kPrependSize;
  }

  /**
   * @brief Reads `len` bytes into a new vector and consumes them from the
   * buffer.
   * @return A vector containing the data. Throws if not enough data is
   * available.
   */
  std::vector<std::uint8_t> RetrieveAsVector(size_t len);

  // --- Write Operations ---

  /** @brief Returns a pointer to the start of the writable memory area. */
  std::uint8_t* BeginWrite() { return Begin() + write_pos_; }
  const std::uint8_t* BeginWrite() const { return Begin() + write_pos_; }

  /** @brief Ensures the buffer has at least `len` writable bytes. */
  void EnsureWritableBytes(size_t len);

  /** @brief Notifies the buffer that `len` bytes have been written. */
  void CommitWrite(size_t len) { write_pos_ += len; }

  /**
   * @brief A convenience function to append data from an external source.
   * This is less efficient than the Ensure/Commit pattern but simpler to use.
   */
  void Append(const void* data, size_t len);
  void Append(std::string_view data);

 private:
  std::uint8_t* Begin() { return storage_.data(); }
  const std::uint8_t* Begin() const { return storage_.data(); }

  const std::uint8_t* BeginRead() const { return Begin() + read_pos_; }

  void MakeSpace(size_t len);

  std::vector<std::uint8_t> storage_;
  size_t read_pos_;
  size_t write_pos_;
};

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
