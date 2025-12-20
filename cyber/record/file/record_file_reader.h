/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#ifndef CYBER_RECORD_FILE_RECORD_FILE_READER_H_
#define CYBER_RECORD_FILE_RECORD_FILE_READER_H_

#include <fcntl.h>
#include <liburing.h>

#include <limits>
#include <memory>
#include <mutex>
#include <string>

#include "google/protobuf/io/zero_copy_stream_impl.h"

#include "cyber/proto/record.pb.h"

#include "cyber/record/file/aligned_buffer.h"
#include "cyber/record/file/record_file_base.h"
#include "cyber/record/file/section.h"

namespace apollo {
namespace cyber {
namespace record {

class RecordFileReader : public RecordFileBase {
 public:
  RecordFileReader();
  virtual ~RecordFileReader();

  bool Open(const std::string& path) override;
  void Close() override;
  bool Reset();

  // 核心读取接口
  bool ReadSection(Section* section);
  bool SkipSection(int64_t size);
  template <typename T>
  bool ReadSection(int64_t size, T* message);

  bool ReadIndex();
  bool EndOfFile() { return end_of_file_; }

  // 获取当前逻辑位置
  int64_t CurrentPosition() { return static_cast<int64_t>(logical_offset_); }

 private:
  bool ReadHeader();
  bool SetPosition(uint64_t target_pos);

  // io_uring 异步管理
  void SubmitPrefetch(uint64_t aligned_offset);
  void WaitAndSwapBuffer();
  bool EnsureDataAvailable(size_t size);
  void ClearBuffers();

  int fd_ = -1;
  bool end_of_file_ = false;

  struct io_uring ring_;
  bool ring_initialized_ = false;

  // 偏移量维护
  uint64_t logical_offset_ = 0;    // 应用层的逻辑偏移
  uint64_t disk_read_offset_ = 0;  // 磁盘物理读取的起始位置（4K对齐）

  // 双缓冲
  std::unique_ptr<AlignedBuffer> active_buf_;
  std::unique_ptr<AlignedBuffer> prefetch_buf_;
  size_t active_buf_pos_ = 0;  // 逻辑位置在 active_buf_ 中的偏移
  size_t active_buf_valid_size_ = 0;
  bool is_prefetching_ = false;

  static constexpr size_t kBufferSize = 16 * 1024 * 1024;  // 16MB
  static constexpr uint64_t kAlignment = 4096;
};

template <typename T>
bool RecordFileReader::ReadSection(int64_t size, T* message) {
  if (size <= 0) return false;
  if (!EnsureDataAvailable(static_cast<size_t>(size))) return false;

  google::protobuf::io::ArrayInputStream array_input(
      active_buf_->data + active_buf_pos_, static_cast<int>(size));

  if (!message->ParseFromZeroCopyStream(&array_input)) {
    AERROR << "Parse message failed at pos: " << logical_offset_;
    return false;
  }

  active_buf_pos_ += size;
  logical_offset_ += size;
  return true;
}

}  // namespace record
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_RECORD_FILE_RECORD_FILE_READER_H_
