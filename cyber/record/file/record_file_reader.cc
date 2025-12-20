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

#include "cyber/record/file/record_file_reader.h"

#include <algorithm>

namespace apollo {
namespace cyber {
namespace record {

RecordFileReader::RecordFileReader() : fd_(-1) {}

RecordFileReader::~RecordFileReader() { Close(); }

bool RecordFileReader::Open(const std::string& path) {
  fd_ = open(path.c_str(), O_RDONLY | O_DIRECT);
  if (fd_ < 0) return false;

  if (io_uring_queue_init(64, &ring_, 0) < 0) return false;
  ring_initialized_ = true;

  active_buf_ = std::make_unique<AlignedBuffer>(kBufferSize);
  prefetch_buf_ = std::make_unique<AlignedBuffer>(kBufferSize);

  // 初始定位：从文件头开始对齐读取
  SetPosition(0);

  if (!ReadHeader()) {
    AERROR << "Read header failed.";
    return false;
  }
  return true;
}

void RecordFileReader::ClearBuffers() {
  // 如果有正在进行的异步请求，必须等待其完成，否则会写坏内存
  if (is_prefetching_) {
    struct io_uring_cqe* cqe;
    io_uring_wait_cqe(&ring_, &cqe);
    io_uring_cqe_seen(&ring_, cqe);
    is_prefetching_ = false;
  }
  active_buf_valid_size_ = 0;
  active_buf_pos_ = 0;
}

bool RecordFileReader::SetPosition(uint64_t target_pos) {
  ClearBuffers();

  // 1. 计算对齐后的物理偏移
  uint64_t aligned_offset = (target_pos / kAlignment) * kAlignment;

  // 2. 计算逻辑位置在对齐块中的偏移
  active_buf_pos_ = target_pos - aligned_offset;
  logical_offset_ = target_pos;

  // 3. 重新发起预取
  SubmitPrefetch(aligned_offset);

  // 4. 立即加载第一个 Buffer 供后续 ReadSection 使用
  WaitAndSwapBuffer();

  end_of_file_ = false;
  return true;
}

void RecordFileReader::SubmitPrefetch(uint64_t aligned_offset) {
  if (is_prefetching_) return;

  struct io_uring_sqe* sqe = io_uring_get_sqe(&ring_);
  io_uring_prep_read(sqe, fd_, prefetch_buf_->data, kBufferSize,
                     aligned_offset);
  io_uring_submit(&ring_);

  is_prefetching_ = true;
  disk_read_offset_ = aligned_offset + kBufferSize;
}

void RecordFileReader::WaitAndSwapBuffer() {
  if (!is_prefetching_) return;

  struct io_uring_cqe* cqe;
  int ret = io_uring_wait_cqe(&ring_, &cqe);

  size_t bytes_read = 0;
  if (ret >= 0 && cqe->res >= 0) {
    bytes_read = static_cast<size_t>(cqe->res);
  } else {
    AERROR << "Async read error or EOF. Res: " << (cqe ? cqe->res : ret);
  }

  io_uring_cqe_seen(&ring_, cqe);
  is_prefetching_ = false;

  // 交换到 Active
  active_buf_.swap(prefetch_buf_);
  active_buf_valid_size_ = bytes_read;
  // 注意：如果是通过 SetPosition 调用的，active_buf_pos_ 已经在 SetPosition
  // 设置好了 如果是流式读取自然耗尽，active_buf_pos_ 应重置为 0

  // 发起下一轮
  SubmitPrefetch(disk_read_offset_);
}

bool RecordFileReader::EnsureDataAvailable(size_t size) {
  if (active_buf_pos_ + size > active_buf_valid_size_) {
    // 检查是否是真的文件末尾
    if (active_buf_valid_size_ < kBufferSize &&
        active_buf_pos_ >= active_buf_valid_size_) {
      end_of_file_ = true;
      return false;
    }

    // 处理跨 Buffer 边界的情况：
    // 如果一个 Section Header 跨过了 16MB 的边界，我们需要将剩余部分拷贝到新
    // Buffer 的开头
    size_t remaining = active_buf_valid_size_ - active_buf_pos_;
    if (remaining > 0) {
      std::vector<char> temp(active_buf_->data + active_buf_pos_,
                             active_buf_->data + active_buf_valid_size_);
      WaitAndSwapBuffer();
      // 将旧 Buffer 末尾的数据挪到新 Buffer 前面
      // 实际上工业级更稳妥的做法是使用环形缓冲区或更复杂的对齐逻辑
      // 这里采用一种简化的方式：如果不足，重新对齐读取
      SetPosition(logical_offset_);
    } else {
      active_buf_pos_ = 0;
      WaitAndSwapBuffer();
    }
  }
  return true;
}

bool RecordFileReader::Reset() {
  return SetPosition(sizeof(Section) + HEADER_LENGTH);
}

bool RecordFileReader::ReadHeader() {
  Section section;
  if (!ReadSection(&section)) return false;
  if (section.type != proto::SectionType::SECTION_HEADER) return false;
  if (!ReadSection<proto::Header>(section.size, &header_)) return false;

  // Header 占位符跳过逻辑
  return SetPosition(sizeof(Section) + HEADER_LENGTH);
}

bool RecordFileReader::ReadIndex() {
  if (!header_.is_complete()) {
    AERROR << "Record file is not complete.";
    return false;
  }
  // 跳转到 Index 位置
  if (!SetPosition(header_.index_position())) return false;

  Section section;
  if (!ReadSection(&section)) return false;
  if (section.type != proto::SectionType::SECTION_INDEX) return false;
  if (!ReadSection<proto::Index>(section.size, &index_)) return false;

  return Reset();  // 读取完索引回到开头准备回放
}

bool RecordFileReader::SkipSection(int64_t size) {
  return SetPosition(logical_offset_ + size);
}

bool RecordFileReader::ReadSection(Section* section) {
  if (!EnsureDataAvailable(sizeof(Section))) return false;
  memcpy(section, active_buf_->data + active_buf_pos_, sizeof(Section));
  active_buf_pos_ += sizeof(Section);
  logical_offset_ += sizeof(Section);
  return true;
}

void RecordFileReader::Close() {
  ClearBuffers();  // 确保安全退出
  if (ring_initialized_) {
    io_uring_queue_exit(&ring_);
    ring_initialized_ = false;
  }
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }
}

}  // namespace record
}  // namespace cyber
}  // namespace apollo
