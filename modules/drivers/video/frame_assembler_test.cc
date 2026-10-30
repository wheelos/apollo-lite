// Copyright 2026 The WheelOS Authors. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0

#include "modules/drivers/video/frame_assembler.h"

#include <cstdint>
#include <string>
#include <vector>

#include "gtest/gtest.h"

namespace apollo {
namespace drivers {
namespace video {
namespace {

std::vector<uint8_t> Header() {
  // Frozen wire header: sequence 65535, IDR, six bytes, frame 7, 10.125 s.
  return {0x80, 0x60, 0xFF, 0xFF, 0,    0,    0,    0,    0, 0, 0, 0,
          0xBB, 0xAA, 0xBB, 0xAA, 0xAA, 0xBB, 0xAA, 0xBB, 0, 1, 0, 0,
          0,    0,    0,    6,    0,    0,    0,    7,    0, 0, 0, 10,
          0,    1,    0xE8, 0x48, 0,    0,    0,    0,    0, 0, 0, 0,
          0,    0,    0,    0,    0,    0,    0,    0};
}

std::vector<uint8_t> Fragment(uint16_t sequence, const std::string& payload) {
  std::vector<uint8_t> bytes(12, 0);
  bytes[0] = 0x80;
  bytes[1] = 0x60;
  bytes[2] = sequence >> 8;
  bytes[3] = sequence & 0xFF;
  bytes.insert(bytes.end(), payload.begin(), payload.end());
  return bytes;
}

bool Feed(FrameAssembler* assembler, const std::vector<uint8_t>& bytes,
          CompressedImage* output) {
  return assembler->AddPacket(bytes.data(), bytes.size(), output);
}

TEST(FrameAssemblerTest,
     ReassemblesBinaryPayloadAndMetadataAcrossSequenceWrap) {
  FrameAssembler assembler;
  CompressedImage output;
  ASSERT_FALSE(Feed(&assembler, Header(), &output));
  ASSERT_FALSE(
      Feed(&assembler, Fragment(0, std::string("\0\0\1", 3)), &output));
  ASSERT_TRUE(
      Feed(&assembler, Fragment(1, std::string("\x26\1\0", 3)), &output));
  EXPECT_EQ(output.data(), std::string("\0\0\1\x26\1\0", 6));
  EXPECT_EQ(output.format(), "h265");
  EXPECT_EQ(output.frame_type(), 1);
  EXPECT_EQ(output.header().camera_timestamp(), 10125000000ULL);
  EXPECT_DOUBLE_EQ(output.measurement_time(), 10.125);
  const auto retained = output.SerializeAsString();
  EXPECT_FALSE(Feed(&assembler, Fragment(2, "orphan"), &output));
  EXPECT_EQ(output.SerializeAsString(), retained);
}

TEST(FrameAssemblerTest, RejectsInvalidHeadersAndRecoversWithoutPublishing) {
  auto short_header = Header();
  short_header.resize(20);
  auto zero_size = Header();
  zero_size[27] = 0;
  auto oversized = Header();
  oversized[25] = 0x40;
  oversized[27] = 1;
  auto bad_time = Header();
  bad_time[36] = 1;
  for (const auto& invalid :
       {std::vector<uint8_t>{}, std::vector<uint8_t>(12), short_header,
        zero_size, oversized, bad_time, std::vector<uint8_t>(1501)}) {
    SCOPED_TRACE(invalid.size());
    FrameAssembler assembler;
    CompressedImage output;
    output.set_data("unchanged");
    EXPECT_FALSE(Feed(&assembler, invalid, &output));
    EXPECT_FALSE(Feed(&assembler, Fragment(0, "orphan"), &output));
    EXPECT_EQ(output.data(), "unchanged");
    ASSERT_FALSE(Feed(&assembler, Header(), &output));
    ASSERT_TRUE(Feed(&assembler, Fragment(0, "abcdef"), &output));
    EXPECT_EQ(output.data(), "abcdef");
  }
}

TEST(FrameAssemblerTest, DiscardsBrokenFramesAndResynchronizesAtNewHeader) {
  for (const auto& invalid :
       {Fragment(2, "abc"), Fragment(65535, "abc"), Fragment(0, "toolong"),
        std::vector<uint8_t>(8)}) {
    FrameAssembler assembler;
    CompressedImage output;
    ASSERT_FALSE(Feed(&assembler, Header(), &output));
    EXPECT_FALSE(Feed(&assembler, invalid, &output));
    EXPECT_FALSE(Feed(&assembler, Fragment(1, "abcdef"), &output));
    EXPECT_TRUE(output.data().empty());
    ASSERT_FALSE(Feed(&assembler, Header(), &output));
    ASSERT_FALSE(Feed(&assembler, Fragment(0, "old"), &output));
    ASSERT_FALSE(Feed(&assembler, Header(), &output));
    ASSERT_TRUE(Feed(&assembler, Fragment(0, "newest"), &output));
    EXPECT_EQ(output.data(), "newest");
  }
}

}  // namespace
}  // namespace video
}  // namespace drivers
}  // namespace apollo
