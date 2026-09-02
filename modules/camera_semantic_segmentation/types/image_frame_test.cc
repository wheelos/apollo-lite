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

#include "modules/camera_semantic_segmentation/types/image_frame.h"

#include <string>
#include <vector>

#include "gtest/gtest.h"

namespace apollo {
namespace camera_semantic_segmentation {

TEST(ImageViewTest, ExpectedByteCountValid) {
  std::size_t byte_count = 0;
  EXPECT_TRUE(ImageView::ExpectedByteCount(1920, 1080, &byte_count));
  EXPECT_EQ(byte_count, 1920U * 1080U * 3U);
}

TEST(ImageViewTest, ExpectedByteCountInvalid) {
  std::size_t byte_count = 0;
  EXPECT_FALSE(ImageView::ExpectedByteCount(0, 1080, &byte_count));
  EXPECT_FALSE(ImageView::ExpectedByteCount(1920, 0, &byte_count));
}

TEST(ImageViewTest, Validate) {
  std::vector<uint8_t> buffer(100 * 100 * 3, 128);
  ImageView view;
  view.bytes = buffer.data();
  view.byte_count = buffer.size();
  view.width = 100;
  view.height = 100;
  view.encoding = ImageEncoding::kRgb8;
  view.camera_name = "front_6mm";

  std::string error;
  EXPECT_TRUE(view.Validate(&error));

  view.camera_name = "";
  EXPECT_FALSE(view.Validate(&error));
}

TEST(ImageViewTest, ParseEncoding) {
  ImageEncoding encoding;
  EXPECT_TRUE(ParseImageEncoding("rgb8", &encoding));
  EXPECT_EQ(encoding, ImageEncoding::kRgb8);

  EXPECT_TRUE(ParseImageEncoding("bgr8", &encoding));
  EXPECT_EQ(encoding, ImageEncoding::kBgr8);

  EXPECT_FALSE(ParseImageEncoding("yuv422", &encoding));
}

}  // namespace camera_semantic_segmentation
}  // namespace apollo
