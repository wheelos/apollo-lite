// Copyright 2026 WheelOS. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//  Created Date: 2026-08-28
//  Author: daohu527

#include "modules/lane/types/image_frame.h"

#include <cstdint>
#include <string>
#include <vector>

#include "gtest/gtest.h"

namespace apollo {
namespace lane {
namespace {

TEST(ImageViewTest, AcceptsExactlySizedRgbPayload) {
  std::vector<uint8_t> bytes(18U);
  ImageView image;
  image.bytes = bytes.data();
  image.byte_count = bytes.size();
  image.width = 3U;
  image.height = 2U;
  image.camera_name = "front";
  std::string error;
  EXPECT_TRUE(image.Validate(&error));
}

TEST(ImageViewTest, RejectsShortPayloadBeforeAccess) {
  std::vector<uint8_t> bytes(17U);
  ImageView image;
  image.bytes = bytes.data();
  image.byte_count = bytes.size();
  image.width = 3U;
  image.height = 2U;
  image.camera_name = "front";
  std::string error;
  EXPECT_FALSE(image.Validate(&error));
}

TEST(ImageViewTest, RejectsDimensionsOutsideSafeLimit) {
  size_t bytes = 0;
  EXPECT_FALSE(ImageView::ExpectedByteCount(16385U, 1U, &bytes));
  EXPECT_FALSE(ImageView::ExpectedByteCount(0U, 1U, &bytes));
}

TEST(ImageViewTest, ParsesOnlySupportedPackedEncodings) {
  ImageEncoding encoding;
  EXPECT_TRUE(ParseImageEncoding("rgb8", &encoding));
  EXPECT_EQ(encoding, ImageEncoding::kRgb8);
  EXPECT_TRUE(ParseImageEncoding("bgr8", &encoding));
  EXPECT_FALSE(ParseImageEncoding("yuyv", &encoding));
}

}  // namespace
}  // namespace lane
}  // namespace apollo
