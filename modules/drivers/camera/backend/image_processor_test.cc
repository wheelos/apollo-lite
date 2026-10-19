/******************************************************************************
 * Copyright 2026 The WheelOS Team. All Rights Reserved.
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

#include "modules/drivers/camera/backend/image_processor.h"

#include <array>
#include <cstdint>
#include <memory>
#include <string>

#include "gtest/gtest.h"

namespace apollo {
namespace drivers {
namespace camera {
namespace {

std::shared_ptr<Image> MakeImage(int width, int height, size_t data_size) {
  auto image = std::make_shared<Image>();
  image->set_width(width);
  image->set_height(height);
  image->mutable_data()->resize(data_size);
  return image;
}

TEST(YuvProcessorTest, NormalizesSupportedPackedFormatsToYuyv) {
  const std::array<uint8_t, 4> expected = {10, 20, 30, 40};
  struct FormatCase {
    YuvProcessor::YuvFormat format;
    std::array<uint8_t, 4> input;
  };
  const std::array<FormatCase, 4> cases = {{
      {YuvProcessor::YuvFormat::YUYV, {10, 20, 30, 40}},
      {YuvProcessor::YuvFormat::YVYU, {10, 40, 30, 20}},
      {YuvProcessor::YuvFormat::UYVY, {20, 10, 40, 30}},
      {YuvProcessor::YuvFormat::VYUY, {40, 10, 20, 30}},
  }};

  for (const auto& test_case : cases) {
    YuvProcessor processor(YuvProcessor::OutputFormat::YUYV, test_case.format);
    auto image = MakeImage(2, 1, 0);

    ASSERT_TRUE(processor.Process(test_case.input.data(),
                                  test_case.input.size(), image));
    EXPECT_EQ(image->data(),
              std::string(reinterpret_cast<const char*>(expected.data()),
                          expected.size()));
  }
}

TEST(YuvProcessorTest, SwapsUvAndProducesRgbPixels) {
  const std::array<uint8_t, 4> input = {50, 20, 60, 220};
  YuvProcessor swap_processor(YuvProcessor::OutputFormat::YUYV,
                              YuvProcessor::YuvFormat::YUYV, true);
  auto yuyv_image = MakeImage(2, 1, 0);
  ASSERT_TRUE(swap_processor.Process(input.data(), input.size(), yuyv_image));
  const std::array<uint8_t, 4> swapped = {50, 220, 60, 20};
  EXPECT_EQ(yuyv_image->data(),
            std::string(reinterpret_cast<const char*>(swapped.data()),
                        swapped.size()));

  const std::array<uint8_t, 4> neutral_chroma = {100, 128, 100, 128};
  YuvProcessor rgb_processor(YuvProcessor::OutputFormat::RGB);
  auto rgb_image = MakeImage(2, 1, 6);
  ASSERT_TRUE(rgb_processor.Process(neutral_chroma.data(),
                                    neutral_chroma.size(), rgb_image));
  ASSERT_EQ(rgb_image->data().size(), 6U);
  for (size_t pixel = 0; pixel < 2; ++pixel) {
    const auto offset = pixel * 3;
    EXPECT_NEAR(static_cast<uint8_t>(rgb_image->data()[offset]), 98, 2);
    EXPECT_NEAR(static_cast<uint8_t>(rgb_image->data()[offset + 1]), 98, 2);
    EXPECT_NEAR(static_cast<uint8_t>(rgb_image->data()[offset + 2]), 98, 2);
  }
}

TEST(YuvProcessorTest, RejectsIncompleteOrInvalidYuv422Frame) {
  YuvProcessor processor(YuvProcessor::OutputFormat::RGB);
  const std::array<uint8_t, 3> short_frame = {16, 128, 16};
  auto image = MakeImage(2, 1, 6);
  EXPECT_FALSE(
      processor.Process(short_frame.data(), short_frame.size(), image));

  image->set_width(3);
  const std::array<uint8_t, 6> odd_width_frame = {16, 128, 16, 128, 16, 128};
  EXPECT_FALSE(
      processor.Process(odd_width_frame.data(), odd_width_frame.size(), image));
}

}  // namespace
}  // namespace camera
}  // namespace drivers
}  // namespace apollo
