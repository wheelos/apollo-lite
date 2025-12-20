#pragma once

namespace apollo {
namespace cyber {
namespace record {

#include <cstdlib>
#include <cstring>
#include <memory>
#include <stdexcept>

// 封装 posix_memalign，确保 4K 对齐
struct AlignedBuffer {
  char* data = nullptr;
  size_t size = 0;
  size_t capacity = 0;

  explicit AlignedBuffer(size_t cap) : capacity(cap) {
    // Jetson 推荐 4KB 对齐，适配 Page Size
    if (posix_memalign((void**)&data, 4096, capacity) != 0) {
      throw std::runtime_error("Aligned alloc failed");
    }
    memset(data, 0, capacity);
  }

  ~AlignedBuffer() { free(data); }

  // 禁止拷贝，只许移动
  AlignedBuffer(const AlignedBuffer&) = delete;
  AlignedBuffer(AlignedBuffer&& other) noexcept {
    data = other.data;
    size = other.size;
    capacity = other.capacity;
    other.data = nullptr;
  }

  // 填充对齐 (O_DIRECT 要求写入大小必须是 Block Size 整数倍)
  void Pad() {
    size_t block_size = 4096;
    size_t remainder = size % block_size;
    if (remainder != 0) {
      size_t padding = block_size - remainder;
      if (size + padding <= capacity) {
        memset(data + size, 0, padding);  // 填充0
        size += padding;
      }
    }
  }
};

}  // namespace record
}  // namespace cyber
}  // namespace apollo
