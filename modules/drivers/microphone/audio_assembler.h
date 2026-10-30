// Copyright 2026 The WheelOS Authors. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstddef>

#include "modules/drivers/microphone/proto/audio.pb.h"

namespace apollo {
namespace drivers {
namespace microphone {

class AudioAssembler {
 public:
  bool Init(const config::MicrophoneConfig& config);
  size_t chunk_bytes() const { return chunk_bytes_; }
  // Input contains exactly chunk_bytes() interleaved sample bytes.
  bool AddChunk(const char* data, config::AudioData* output);

 private:
  config::AudioData audio_;
  size_t chunk_bytes_ = 0;
  int chunks_ = 0;
  int chunk_index_ = 0;
};

}  // namespace microphone
}  // namespace drivers
}  // namespace apollo
