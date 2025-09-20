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

#include "modules/drivers/gnss/util/data_buffer.h"

namespace apollo {
namespace drivers {
namespace gnss {

void DataBuffer::Append(const void* data, size_t len) {
  EnsureWritableBytes(len);
  std::copy(static_cast<const uint8_t*>(data),
            static_cast<const uint8_t*>(data) + len, BeginWrite());
  CommitWrite(len);
}

void DataBuffer::Append(std::string_view data) {
  Append(data.data(), data.size());
}

std::optional<uint8_t> DataBuffer::Poll() {
  if (IsEmpty()) {
    return std::nullopt;
  }
  // Retrieve the byte from the read position and then advance the pointer
  uint8_t byte = storage_[read_pos_++];
  return byte;
}

std::optional<size_t> DataBuffer::Find(std::string_view pattern) const {
  if (pattern.empty() || IsEmpty()) {
    return std::nullopt;
  }
  const auto view = Peek();
  auto pos = view.find(pattern);
  if (pos == std::string_view::npos) {
    return std::nullopt;
  }
  return pos;
}

void DataBuffer::Drain(size_t len) {
  if (len >= ReadableBytes()) {
    DrainAll();
  } else {
    read_pos_ += len;
  }
}

std::vector<uint8_t> DataBuffer::RetrieveAsVector(size_t len) {
  if (len > ReadableBytes()) {
    throw std::out_of_range("Not enough data to retrieve in DataBuffer");
  }
  std::vector<uint8_t> result(BeginRead(), BeginRead() + len);
  Drain(len);
  return result;
}

void DataBuffer::EnsureWritableBytes(size_t len) {
  if (storage_.size() - write_pos_ < len) {
    MakeSpace(len);
  }
}

void DataBuffer::MakeSpace(size_t len) {
  // If unused space at the front is enough, move data to the front.
  if (read_pos_ - kPrependSize + (storage_.size() - write_pos_) >= len) {
    size_t readable = ReadableBytes();
    std::move(Begin() + read_pos_, Begin() + write_pos_,
              Begin() + kPrependSize);
    read_pos_ = kPrependSize;
    write_pos_ = read_pos_ + readable;
  } else {
    // Otherwise, resize the vector.
    storage_.resize(write_pos_ + len);
  }
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
