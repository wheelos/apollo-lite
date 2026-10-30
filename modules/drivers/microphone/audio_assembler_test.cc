// Copyright 2026 The WheelOS Authors. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0

#include "modules/drivers/microphone/audio_assembler.h"

#include <limits>
#include <string>

#include "gtest/gtest.h"

namespace apollo {
namespace drivers {
namespace microphone {
namespace {

config::MicrophoneConfig Config() {
  config::MicrophoneConfig config;
  config.set_microphone_model(config::MicrophoneConfig::RESPEAKER);
  config.set_chunk(2);
  config.set_sample_rate(8);
  config.set_record_seconds(0.375);  // Three frames round up to two chunks.
  config.set_sample_width(2);
  config.set_channel_name("/test/audio");
  config.set_frame_id("microphone");
  config.add_channel_type(config::ASR);
  config.add_channel_type(config::RAW);
  return config;
}

TEST(AudioAssemblerTest, PreservesEverySampleByteAndRoundsToWholeChunks) {
  for (const int width : {1, 2, 3, 4}) {
    SCOPED_TRACE(width);
    auto config = Config();
    config.set_sample_width(width);
    AudioAssembler assembler;
    ASSERT_TRUE(assembler.Init(config));
    std::string input(assembler.chunk_bytes(), '\0');
    for (size_t i = 0; i < input.size(); ++i) {
      input[i] = static_cast<char>(i * 43);
    }
    config::AudioData output;
    EXPECT_FALSE(assembler.AddChunk(input.data(), &output));
    EXPECT_EQ(output.channel_data_size(), 0);
    ASSERT_TRUE(assembler.AddChunk(input.data(), &output));
    EXPECT_EQ(output.microphone_config().SerializeAsString(),
              config.SerializeAsString());
    EXPECT_EQ(output.header().frame_id(), "microphone");
    ASSERT_EQ(output.channel_data_size(), 2);
    EXPECT_EQ(output.channel_data(0).channel_type(), config::ASR);
    EXPECT_EQ(output.channel_data(1).channel_type(), config::RAW);
    for (int channel = 0; channel < 2; ++channel) {
      const std::string expected = input.substr(channel * width, width) +
                                   input.substr((channel + 2) * width, width);
      EXPECT_EQ(output.channel_data(channel).data(), expected + expected);
    }
    const auto retained = output.SerializeAsString();
    input.assign(input.size(), '\xFF');
    EXPECT_FALSE(assembler.AddChunk(input.data(), &output));
    EXPECT_EQ(output.SerializeAsString(), retained);
    ASSERT_TRUE(assembler.AddChunk(input.data(), &output));
    EXPECT_EQ(output.channel_data(0).data(), std::string(4 * width, '\xFF'));
  }
}

TEST(AudioAssemblerTest, RejectsInvalidAndOverflowingCaptureDimensions) {
  for (int invalid = 0; invalid < 11; ++invalid) {
    SCOPED_TRACE(invalid);
    auto config = Config();
    switch (invalid) {
      case 0:
        config.set_chunk(0);
        break;
      case 1:
        config.clear_channel_type();
        break;
      case 2:
        config.set_sample_width(0);
        break;
      case 3:
        config.set_sample_width(5);
        break;
      case 4:
        config.set_sample_rate(-1);
        break;
      case 5:
        config.set_record_seconds(0);
        break;
      case 6:
        config.set_record_seconds(std::numeric_limits<float>::quiet_NaN());
        break;
      case 7:
        config.set_sample_rate(std::numeric_limits<float>::infinity());
        break;
      case 8:
        config.set_chunk(std::numeric_limits<int>::max());
        break;
      case 9:
        config.set_record_seconds(std::numeric_limits<float>::max());
        break;
      case 10:
        config.clear_microphone_model();
        break;
    }
    AudioAssembler assembler;
    EXPECT_FALSE(assembler.Init(config));
  }
}

TEST(AudioAssemblerTest, KeepsExistingSingleChunkFloatRounding) {
  auto config = Config();
  config.set_chunk(4096);
  config.set_sample_rate(16000);
  config.set_record_seconds(0.256F);
  AudioAssembler assembler;
  ASSERT_TRUE(assembler.Init(config));
  const std::string input(assembler.chunk_bytes(), 'x');
  config::AudioData output;
  ASSERT_TRUE(assembler.AddChunk(input.data(), &output));
  EXPECT_EQ(output.channel_data(0).data().size(), 8192U);
}

}  // namespace
}  // namespace microphone
}  // namespace drivers
}  // namespace apollo
