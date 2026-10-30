// Copyright 2026 The WheelOS Authors. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstddef>
#include <cstdint>

#include "wheelos_msgs/sensor_msgs/sensor_image.pb.h"

namespace apollo {
namespace drivers {
namespace video {

constexpr size_t H265_FRAME_PACKAGE_SIZE = 4 * 1024 * 1024;
constexpr size_t H265_PDU_SIZE = 1500;

class FrameAssembler {
 public:
  // Returns true only for a complete frame. Invalid packets discard the
  // current frame; assembly resumes at the next valid camera header.
  bool AddPacket(const uint8_t* data, size_t size, CompressedImage* output);
  void Reset();

 private:
  CompressedImage frame_;
  size_t expected_size_ = 0;
  uint16_t sequence_ = 0;
  uint32_t frame_id_ = 0;
};

}  // namespace video
}  // namespace drivers
}  // namespace apollo
