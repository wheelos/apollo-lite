/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
#include <vector>

#include "gtest/gtest.h"

#include "opencv2/imgcodecs.hpp"

#include "modules/drivers/tools/image_decompress/image_decoder.h"

namespace apollo {
namespace image_decompress {

TEST(ImageDecoderTest, DecodesRgbPixelsAndPreservesCaptureMetadata) {
  cv::Mat source(2, 4, CV_8UC3, cv::Scalar(10, 20, 200));
  std::vector<uint8_t> bytes;
  ASSERT_TRUE(cv::imencode(".png", source, bytes));
  drivers::CompressedImage compressed;
  compressed.set_data(bytes.data(), bytes.size());
  compressed.mutable_header()->set_frame_id("camera");
  compressed.mutable_header()->set_timestamp_sec(12.5);
  drivers::Image image;
  for (const bool explicit_time : {false, true}) {
    if (explicit_time) {
      compressed.set_measurement_time(11.25);
    }
    ASSERT_TRUE(DecodeImage(compressed, &image));
    EXPECT_EQ(image.header().frame_id(), "camera");
    EXPECT_DOUBLE_EQ(image.header().timestamp_sec(), 12.5);
    EXPECT_DOUBLE_EQ(image.measurement_time(), explicit_time ? 11.25 : 12.5);
    EXPECT_EQ(image.width(), 4U);
    EXPECT_EQ(image.height(), 2U);
    EXPECT_EQ(image.step(), 12U);
    EXPECT_EQ(image.encoding(), "rgb8");
    ASSERT_EQ(image.data().size(), 24U);
    EXPECT_EQ(static_cast<uint8_t>(image.data()[0]), 200);
    EXPECT_EQ(static_cast<uint8_t>(image.data()[1]), 20);
    EXPECT_EQ(static_cast<uint8_t>(image.data()[2]), 10);
  }
}

TEST(ImageDecoderTest, RejectsMissingAndCorruptBytesWithoutMutatingOutput) {
  drivers::CompressedImage compressed;
  drivers::Image image;
  image.set_data("retained");
  EXPECT_FALSE(DecodeImage(compressed, &image));
  compressed.set_data("not an encoded image");
  EXPECT_FALSE(DecodeImage(compressed, &image));
  EXPECT_FALSE(DecodeImage(compressed, nullptr));
  EXPECT_EQ(image.data(), "retained");
}

}  // namespace image_decompress
}  // namespace apollo
