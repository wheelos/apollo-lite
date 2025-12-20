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

#include "cyber/record/file/record_file_writer.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

namespace apollo {
namespace cyber {
namespace record {

RecordFileWriter::RecordFileWriter() {
  chunk_active_.reset(new Chunk());
  chunk_flush_.reset(new Chunk());
}

RecordFileWriter::~RecordFileWriter() { Close(); }

bool RecordFileWriter::Open(const std::string& path) {
  std::lock_guard<std::mutex> lock(write_mutex_);

  // 1. O_DIRECT 打开
  fd_ = open(path.c_str(), O_CREAT | O_WRONLY | O_DIRECT | O_TRUNC, 0644);
  if (fd_ < 0) return false;

  // 2. 预分配 2GB 空间
  fallocate(fd_, 0, 0, 2UL * 1024 * 1024 * 1024);

  // 3. 初始化 io_uring
  struct io_uring_params params;
  memset(&params, 0, sizeof(params));
  params.flags = IORING_SETUP_SQPOLL;
  if (io_uring_queue_init_params(256, &ring_, &params) < 0) return false;
  ring_initialized_ = true;

  // 4. 初始化缓冲池
  for (int i = 0; i < kPoolSize; ++i) {
    buffer_pool_.push(std::make_unique<AlignedBuffer>(kBufferSize));
  }
  current_buffer_ = GetFreeBuffer();

  // 5. 启动 Flush 线程
  is_writing_ = true;
  flush_thread_.reset(new std::thread(&RecordFileWriter::Flush, this));

  return true;
}

void RecordFileWriter::Close() {
  if (!is_writing_.exchange(false)) return;

  // 唤醒并停止 Flush 线程
  flush_cv_.notify_all();
  if (flush_thread_ && flush_thread_->joinable()) {
    flush_thread_->join();
  }

  // 写入索引并提交最后的缓冲区
  WriteIndex();

  {
    std::lock_guard<std::mutex> lock(write_mutex_);
    SubmitCurrentBuffer();
  }

  // 等待所有 IO 完成
  while (in_flight_io_.load() > 0) {
    PollCompletions(true);
  }

  // 物理截断文件
  ftruncate(fd_, disk_offset_);

  if (ring_initialized_) {
    io_uring_queue_exit(&ring_);
    ring_initialized_ = false;
  }
  if (fd_ != -1) {
    close(fd_);
    fd_ = -1;
  }
}

bool RecordFileWriter::WriteMessage(const proto::SingleMessage& message) {
  chunk_active_->add(message);
  auto channel_name = message.channel_name();
  channel_message_number_map_[channel_name]++;

  // 达到 64MB 触发 Chunk 切换
  if (chunk_active_->header_.raw_size() > kBufferSize) {
    std::unique_lock<std::mutex> lock(flush_mutex_);
    chunk_active_.swap(chunk_flush_);
    chunk_active_->clear();
    flush_cv_.notify_one();
  }
  return true;
}

void RecordFileWriter::Flush() {
  while (is_writing_) {
    std::unique_lock<std::mutex> lock(flush_mutex_);
    flush_cv_.wait(lock,
                   [this] { return !chunk_flush_->empty() || !is_writing_; });

    if (!is_writing_ && chunk_flush_->empty()) break;

    if (!chunk_flush_->empty()) {
      WriteChunk(chunk_flush_->header_, *(chunk_flush_->body_));
      chunk_flush_->clear();
    }
  }
}

bool RecordFileWriter::WriteChunk(const proto::ChunkHeader& chunk_header,
                                  const proto::ChunkBody& chunk_body) {
  std::lock_guard<std::mutex> lock(write_mutex_);

  // 分别写入 ChunkHeader 和 ChunkBody Section
  if (!WriteSection(chunk_header)) return false;
  if (!WriteSection(chunk_body)) return false;

  PollCompletions(false);  // 顺便检查 IO
  return true;
}

bool RecordFileWriter::SubmitCurrentBuffer() {
  if (!current_buffer_ || current_buffer_->size == 0) return true;

  current_buffer_->Pad();  // 对齐到 4K

  struct io_uring_sqe* sqe = io_uring_get_sqe(&ring_);
  if (!sqe) {
    PollCompletions(true);
    sqe = io_uring_get_sqe(&ring_);
  }

  AlignedBuffer* raw_ptr = current_buffer_.release();
  io_uring_prep_write(sqe, fd_, raw_ptr->data, raw_ptr->size, disk_offset_);
  io_uring_sqe_set_data(sqe, raw_ptr);

  disk_offset_ += raw_ptr->size;
  in_flight_io_++;
  io_uring_submit(&ring_);

  return true;
}

void RecordFileWriter::PollCompletions(bool wait) {
  struct io_uring_cqe* cqe;
  if (wait) io_uring_wait_cqe(&ring_, &cqe);

  while (io_uring_peek_cqe(&ring_, &cqe) == 0) {
    auto* buf = static_cast<AlignedBuffer*>(io_uring_cqe_get_data(cqe));
    buf->size = 0;
    {
      std::lock_guard<std::mutex> lock(pool_mutex_);
      buffer_pool_.push(std::unique_ptr<AlignedBuffer>(buf));
    }
    in_flight_io_--;
    io_uring_cqe_seen(&ring_, cqe);
  }
}

std::unique_ptr<AlignedBuffer> RecordFileWriter::GetFreeBuffer() {
  std::unique_lock<std::mutex> lock(pool_mutex_);
  if (buffer_pool_.empty()) {
    return std::make_unique<AlignedBuffer>(kBufferSize);
  }
  auto buf = std::move(buffer_pool_.front());
  buffer_pool_.pop();
  return buf;
}

bool RecordFileWriter::WriteHeader(const Header& header) {
  std::lock_guard<std::mutex> lock(mutex_);
  header_ = header;
  if (!WriteSection<Header>(header_)) {
    AERROR << "Write header section fail";
    return false;
  }
  return true;
}

bool RecordFileWriter::WriteIndex() {
  std::lock_guard<std::mutex> lock(mutex_);
  for (int i = 0; i < index_.indexes_size(); i++) {
    SingleIndex* single_index = index_.mutable_indexes(i);
    if (single_index->type() == SectionType::SECTION_CHANNEL) {
      ChannelCache* channel_cache = single_index->mutable_channel_cache();
      if (channel_message_number_map_.find(channel_cache->name()) !=
          channel_message_number_map_.end()) {
        channel_cache->set_message_number(
            channel_message_number_map_[channel_cache->name()]);
      }
    }
  }
  header_.set_index_position(CurrentPosition());
  if (!WriteSection<proto::Index>(index_)) {
    AERROR << "Write section fail";
    return false;
  }
  return true;
}

bool RecordFileWriter::WriteChannel(const Channel& channel) {
  std::lock_guard<std::mutex> lock(mutex_);
  uint64_t pos = CurrentPosition();
  if (!WriteSection<Channel>(channel)) {
    AERROR << "Write section fail";
    return false;
  }
  header_.set_channel_number(header_.channel_number() + 1);
  SingleIndex* single_index = index_.add_indexes();
  single_index->set_type(SectionType::SECTION_CHANNEL);
  single_index->set_position(pos);
  ChannelCache* channel_cache = new ChannelCache();
  channel_cache->set_name(channel.name());
  channel_cache->set_message_number(0);
  channel_cache->set_message_type(channel.message_type());
  channel_cache->set_proto_desc(channel.proto_desc());
  single_index->set_allocated_channel_cache(channel_cache);
  return true;
}

uint64_t RecordFileWriter::GetMessageNumber(
    const std::string& channel_name) const {
  auto search = channel_message_number_map_.find(channel_name);
  if (search != channel_message_number_map_.end()) {
    return search->second;
  }
  return 0;
}

}  // namespace record
}  // namespace cyber
}  // namespace apollo
