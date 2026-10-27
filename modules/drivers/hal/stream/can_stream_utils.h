// Copyright 2025 WheelOS. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include <fcntl.h>

#include <cstddef>
#include <cstdint>
#include <cstring>

#include <linux/can.h>

namespace apollo {
namespace drivers {
namespace hal {
namespace internal {

inline int SetCanSocketNonBlocking(int fd) {
  const int flags = fcntl(fd, F_GETFL, 0);
  return flags < 0 ? -1 : fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

inline bool IsCanFrameSizeValid(std::size_t size, bool can_fd_enabled) {
  return size == sizeof(struct can_frame) ||
         (can_fd_enabled && size == sizeof(struct canfd_frame));
}

inline void StoreCanFrame(std::uint8_t* slot, std::size_t slot_size,
                          const std::uint8_t* frame, std::size_t frame_size) {
  std::memset(slot, 0, slot_size);
  std::memcpy(slot, frame, frame_size);
}

}  // namespace internal
}  // namespace hal
}  // namespace drivers
}  // namespace apollo
