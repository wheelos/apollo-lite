// Copyright 2026 The WheelOS Authors. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0

#include "modules/drivers/video/frame_assembler.h"

#include <arpa/inet.h>

#include <cstring>

#include "cyber/common/log.h"
#include "modules/drivers/video/input.h"

namespace apollo {
namespace drivers {
namespace video {

void FrameAssembler::Reset() {
  expected_size_ = 0;
  frame_.Clear();
}

bool FrameAssembler::AddPacket(const uint8_t* data, size_t size,
                               CompressedImage* output) {
  if (size <= sizeof(RtpHeader) || size > H265_PDU_SIZE) {
    AERROR << "Invalid camera datagram size: " << size;
    Reset();
    return false;
  }
  RtpHeader rtp;
  std::memcpy(&rtp, data, sizeof(rtp));
  const uint16_t sequence = ntohs(rtp.seq);
  if (size >= sizeof(RtpHeader) + 2 * sizeof(uint32_t)) {
    uint32_t magic[2];
    std::memcpy(magic, data + sizeof(RtpHeader), sizeof(magic));
    if (ntohl(magic[0]) == HW_CAMERA_MAGIC0 &&
        ntohl(magic[1]) == HW_CAMERA_MAGIC1) {
      if (expected_size_ != 0) {
        AERROR << "Incomplete camera frame replaced by a new header";
      }
      Reset();
      if (size < sizeof(HwPduPacket)) {
        AERROR << "Truncated camera frame header";
        return false;
      }
      FrameHeader header;
      std::memcpy(&header, data + sizeof(RtpHeader), sizeof(header));
      const size_t frame_size = ntohl(header.frame_size);
      const uint32_t microseconds = ntohl(header.ts_usec);
      if (frame_size == 0 || frame_size > H265_FRAME_PACKAGE_SIZE ||
          microseconds >= 1000000) {
        AERROR << "Invalid camera frame size or timestamp";
        return false;
      }
      const uint32_t frame_id = ntohl(header.frame_id);
      if (frame_id - frame_id_ != 1 && frame_id_ > 1 && frame_id > 1) {
        AERROR << "Camera frame ID discontinuity: previous/current "
               << frame_id_ << "/" << frame_id;
      }
      frame_id_ = frame_id;
      expected_size_ = frame_size;
      sequence_ = sequence;
      const uint64_t timestamp = uint64_t{ntohl(header.ts_sec)} * 1000000000 +
                                 uint64_t{microseconds} * 1000;
      frame_.mutable_header()->set_camera_timestamp(timestamp);
      frame_.set_measurement_time(static_cast<double>(timestamp) * 1e-9);
      frame_.set_format("h265");
      frame_.set_frame_type(header.frame_type);
      frame_.mutable_data()->reserve(expected_size_);
      return false;
    }
  }
  if (expected_size_ == 0) {
    return false;
  }
  const size_t payload_size = size - sizeof(RtpHeader);
  if (sequence != static_cast<uint16_t>(sequence_ + 1) ||
      payload_size > expected_size_ - frame_.data().size()) {
    AERROR << "Camera fragment sequence or length mismatch";
    Reset();
    return false;
  }
  sequence_ = sequence;
  frame_.mutable_data()->append(
      reinterpret_cast<const char*>(data + sizeof(RtpHeader)), payload_size);
  if (frame_.data().size() != expected_size_) {
    return false;
  }
  output->CopyFrom(frame_);
  Reset();
  return true;
}

}  // namespace video
}  // namespace drivers
}  // namespace apollo
