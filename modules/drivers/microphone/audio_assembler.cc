// Copyright 2026 The WheelOS Authors. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0

#include "modules/drivers/microphone/audio_assembler.h"

#include <cmath>
#include <cstring>
#include <limits>

#include "cyber/common/log.h"

namespace apollo {
namespace drivers {
namespace microphone {

bool AudioAssembler::Init(const config::MicrophoneConfig& config) {
  audio_.Clear();
  chunk_index_ = 0;
  chunks_ = 0;
  chunk_bytes_ = 0;
  if (config.microphone_model() != config::MicrophoneConfig::RESPEAKER ||
      config.chunk() <= 0 || config.channel_type_size() == 0 ||
      config.sample_width() < 1 || config.sample_width() > 4 ||
      !std::isfinite(config.sample_rate()) || config.sample_rate() <= 0 ||
      !std::isfinite(config.record_seconds()) || config.record_seconds() <= 0 ||
      config.channel_name().empty()) {
    AERROR << "Invalid microphone capture configuration";
    return false;
  }
  const double chunks = std::ceil(config.sample_rate() *
                                  config.record_seconds() / config.chunk());
  const double chunk_bytes = static_cast<double>(config.chunk()) *
                             config.channel_type_size() * config.sample_width();
  const double channel_bytes = chunks * config.chunk() * config.sample_width();
  const double limit = std::numeric_limits<int>::max();
  if (chunks < 1 || chunks > limit || chunk_bytes > limit ||
      channel_bytes > limit ||
      channel_bytes * config.channel_type_size() > limit) {
    AERROR << "Microphone buffer size exceeds supported range";
    return false;
  }
  chunks_ = static_cast<int>(chunks);
  chunk_bytes_ = static_cast<size_t>(chunk_bytes);
  audio_.mutable_microphone_config()->CopyFrom(config);
  audio_.mutable_header()->set_frame_id(config.frame_id());
  for (const auto type : config.channel_type()) {
    auto* channel = audio_.add_channel_data();
    channel->set_channel_type(static_cast<config::ChannelType>(type));
    channel->mutable_data()->resize(static_cast<size_t>(channel_bytes));
  }
  return true;
}

bool AudioAssembler::AddChunk(const char* data, config::AudioData* output) {
  const auto& config = audio_.microphone_config();
  const size_t width = config.sample_width();
  const size_t offset =
      static_cast<size_t>(chunk_index_) * config.chunk() * width;
  for (int channel = 0; channel < config.channel_type_size(); ++channel) {
    auto* destination = audio_.mutable_channel_data(channel)->mutable_data();
    for (int frame = 0; frame < config.chunk(); ++frame) {
      const size_t input =
          (static_cast<size_t>(frame) * config.channel_type_size() + channel) *
          width;
      std::memcpy(&(*destination)[offset + frame * width], data + input, width);
    }
  }
  if (++chunk_index_ != chunks_) {
    return false;
  }
  output->CopyFrom(audio_);
  chunk_index_ = 0;
  return true;
}

}  // namespace microphone
}  // namespace drivers
}  // namespace apollo
