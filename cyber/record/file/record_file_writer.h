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

#ifndef CYBER_RECORD_FILE_RECORD_FILE_WRITER_H_
#define CYBER_RECORD_FILE_RECORD_FILE_WRITER_H_

#include <liburing.h>

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "cyber/proto/record.pb.h"

#include "cyber/common/log.h"
#include "cyber/record/file/aligned_buffer.h"
#include "cyber/record/file/record_file_base.h"
#include "cyber/record/file/section.h"

namespace apollo {
namespace cyber {
namespace record {

struct Chunk {
  Chunk() { clear(); }

  inline void clear() {
    body_.reset(new proto::ChunkBody());
    header_.set_begin_time(0);
    header_.set_end_time(0);
    header_.set_message_number(0);
    header_.set_raw_size(0);
  }

  inline void add(const proto::SingleMessage& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto* p_message = body_->add_messages();
    *p_message = message;
    if (header_.begin_time() == 0 || header_.begin_time() > message.time()) {
      header_.set_begin_time(message.time());
    }
    if (header_.end_time() < message.time()) {
      header_.set_end_time(message.time());
    }
    header_.set_message_number(header_.message_number() + 1);
    header_.set_raw_size(header_.raw_size() + message.content().size());
  }

  inline bool empty() { return header_.message_number() == 0; }

  std::mutex mutex_;
  proto::ChunkHeader header_;
  std::unique_ptr<proto::ChunkBody> body_ = nullptr;
};

class RecordFileWriter : public RecordFileBase {
 public:
  RecordFileWriter();
  virtual ~RecordFileWriter();

  bool Open(const std::string& path) override;
  void Close() override;
  bool WriteHeader(const proto::Header& header);
  bool WriteChannel(const proto::Channel& channel);
  bool WriteMessage(const proto::SingleMessage& message);
  uint64_t GetMessageNumber(const std::string& channel_name) const;

 private:
  bool WriteChunk(const proto::ChunkHeader& chunk_header,
                  const proto::ChunkBody& chunk_body);
  template <typename T>
  bool WriteSection(const T& message);
  bool WriteIndex();
  void Flush();

  // io_uring 核心逻辑
  bool SubmitCurrentBuffer();
  void PollCompletions(bool wait);
  std::unique_ptr<AlignedBuffer> GetFreeBuffer();

  // I/O 状态
  int fd_ = -1;
  uint64_t logical_position_ = 0;  // 对应原本的 CurrentPosition()
  uint64_t disk_offset_ = 0;       // 实际落盘偏移

  struct io_uring ring_;
  bool ring_initialized_ = false;
  std::atomic<int32_t> in_flight_io_{0};

  // 缓冲管理
  std::unique_ptr<AlignedBuffer> current_buffer_;
  std::queue<std::unique_ptr<AlignedBuffer>> buffer_pool_;
  std::mutex pool_mutex_;
  std::mutex write_mutex_;

  // 原有逻辑成员
  std::atomic_bool is_writing_{false};
  std::unique_ptr<Chunk> chunk_active_ = nullptr;
  std::unique_ptr<Chunk> chunk_flush_ = nullptr;
  std::unique_ptr<std::thread> flush_thread_ = nullptr;
  std::mutex flush_mutex_;
  std::condition_variable flush_cv_;
  std::unordered_map<std::string, uint64_t> channel_message_number_map_;

  static constexpr size_t kBufferSize = 64 * 1024 * 1024;
  static constexpr size_t kPoolSize = 3;
};

template <typename T>
bool RecordFileWriter::WriteSection(const T& message) {
  proto::SectionType type;
  if (std::is_same<T, proto::ChunkHeader>::value)
    type = proto::SectionType::SECTION_CHUNK_HEADER;
  else if (std::is_same<T, proto::ChunkBody>::value)
    type = proto::SectionType::SECTION_CHUNK_BODY;
  else if (std::is_same<T, proto::Channel>::value)
    type = proto::SectionType::SECTION_CHANNEL;
  else if (std::is_same<T, proto::Header>::value)
    type = proto::SectionType::SECTION_HEADER;
  else if (std::is_same<T, proto::Index>::value)
    type = proto::SectionType::SECTION_INDEX;
  else
    return false;

  size_t msg_size = message.ByteSizeLong();
  size_t section_size = sizeof(Section) + msg_size;

  // Header 特殊处理：占位 2048 字节
  size_t write_size = (type == proto::SectionType::SECTION_HEADER)
                          ? (sizeof(Section) + HEADER_LENGTH)
                          : section_size;

  // 检查缓冲区
  if (current_buffer_->size + write_size > current_buffer_->capacity) {
    SubmitCurrentBuffer();
    current_buffer_ = GetFreeBuffer();
  }

  // 1. 记录当前逻辑位置（用于 Index）
  uint64_t current_pos = logical_position_;

  // 2. 写入 Section 结构
  Section section = {type, static_cast<int64_t>(msg_size)};
  memcpy(current_buffer_->data + current_buffer_->size, &section,
         sizeof(section));
  current_buffer_->size += sizeof(section);
  logical_position_ += sizeof(section);

  // 3. 序列化消息
  if (type == proto::SectionType::SECTION_HEADER) {
    char* header_start = current_buffer_->data + current_buffer_->size;
    memset(header_start, '0', HEADER_LENGTH);
    message.SerializeToArray(header_start, msg_size);
    current_buffer_->size += HEADER_LENGTH;
    logical_position_ += HEADER_LENGTH;
  } else {
    message.SerializeToArray(current_buffer_->data + current_buffer_->size,
                             msg_size);
    current_buffer_->size += msg_size;
    logical_position_ += msg_size;
  }

  header_.set_size(logical_position_);
  return true;
}

}  // namespace record
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_RECORD_FILE_RECORD_FILE_WRITER_H_
