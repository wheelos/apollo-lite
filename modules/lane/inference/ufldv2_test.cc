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

#include "modules/lane/inference/ufldv2.h"

#include <cmath>
#include <cstdint>
#include <vector>

#include "gtest/gtest.h"

namespace apollo {
namespace lane {
namespace {

size_t RowIndex(int grid, int anchor, int lane) {
  return ((static_cast<size_t>(grid) * 72U + static_cast<size_t>(anchor)) * 4U +
          static_cast<size_t>(lane));
}

size_t ExistenceIndex(int label, int anchor, int lane, int anchor_count) {
  return ((static_cast<size_t>(label) * static_cast<size_t>(anchor_count) +
           static_cast<size_t>(anchor)) *
              4U +
          static_cast<size_t>(lane));
}

Ufldv2Tensor MakeTensor(std::initializer_list<int64_t> shape) {
  Ufldv2Tensor tensor;
  tensor.shape.assign(shape.begin(), shape.end());
  size_t size = 1U;
  for (int64_t dimension : tensor.shape) {
    size *= static_cast<size_t>(dimension);
  }
  tensor.values.assign(size, 0.0F);
  return tensor;
}

Ufldv2TensorOutputs MakeOutputs() {
  Ufldv2TensorOutputs outputs;
  outputs.loc_row = MakeTensor({1, 200, 72, 4});
  outputs.loc_col = MakeTensor({1, 100, 81, 4});
  outputs.exist_row = MakeTensor({1, 2, 72, 4});
  outputs.exist_col = MakeTensor({1, 2, 81, 4});
  for (int anchor = 0; anchor < 72; ++anchor) {
    for (int lane = 0; lane < 4; ++lane) {
      outputs.exist_row.values[ExistenceIndex(0, anchor, lane, 72)] = 10.0F;
    }
  }
  for (int anchor = 0; anchor < 81; ++anchor) {
    for (int lane = 0; lane < 4; ++lane) {
      outputs.exist_col.values[ExistenceIndex(0, anchor, lane, 81)] = 10.0F;
    }
  }
  for (const int anchor : {30, 50, 71}) {
    outputs.exist_row.values[ExistenceIndex(1, anchor, 1, 72)] = 20.0F;
    outputs.loc_row.values[RowIndex(100, anchor, 1)] = 20.0F;
  }
  return outputs;
}

ImageView MakeImage(std::vector<uint8_t>* bytes) {
  bytes->assign(1600U * 1000U * 3U, 0U);
  ImageView image;
  image.bytes = bytes->data();
  image.byte_count = bytes->size();
  image.width = 1600U;
  image.height = 1000U;
  image.camera_name = "front";
  return image;
}

TEST(Ufldv2PreprocessTest, NormalizesRgbChannelsIntoChwPlanes) {
  std::vector<uint8_t> bytes(4U * 4U * 3U);
  for (size_t index = 0; index < bytes.size(); index += 3U) {
    bytes[index] = 255U;
    bytes[index + 1U] = 128U;
    bytes[index + 2U] = 0U;
  }
  ImageView image;
  image.bytes = bytes.data();
  image.byte_count = bytes.size();
  image.width = 4U;
  image.height = 4U;
  image.encoding = ImageEncoding::kRgb8;
  image.camera_name = "front";

  std::vector<float> input;
  ASSERT_TRUE(Ufldv2Detector::Preprocess(image, &input));
  ASSERT_EQ(input.size(), kUfldv2InputElementCount);
  const size_t plane_size =
      static_cast<size_t>(kUfldv2ModelWidth) * kUfldv2ModelHeight;
  EXPECT_NEAR(input[0], (1.0F - 0.485F) / 0.229F, 1.0e-5F);
  EXPECT_NEAR(input[plane_size], (128.0F / 255.0F - 0.456F) / 0.224F,
              1.0e-5F);
  EXPECT_NEAR(input[2U * plane_size], (0.0F - 0.406F) / 0.225F, 1.0e-5F);
}

TEST(Ufldv2PreprocessTest, ConvertsBgrInputToRgbPlanes) {
  std::vector<uint8_t> rgb_bytes(4U * 4U * 3U);
  std::vector<uint8_t> bgr_bytes(4U * 4U * 3U);
  for (size_t index = 0; index < rgb_bytes.size(); index += 3U) {
    rgb_bytes[index] = 200U;
    rgb_bytes[index + 1U] = 100U;
    rgb_bytes[index + 2U] = 20U;
    bgr_bytes[index] = 20U;
    bgr_bytes[index + 1U] = 100U;
    bgr_bytes[index + 2U] = 200U;
  }
  ImageView rgb;
  rgb.bytes = rgb_bytes.data();
  rgb.byte_count = rgb_bytes.size();
  rgb.width = 4U;
  rgb.height = 4U;
  rgb.encoding = ImageEncoding::kRgb8;
  rgb.camera_name = "front";
  ImageView bgr = rgb;
  bgr.bytes = bgr_bytes.data();
  bgr.encoding = ImageEncoding::kBgr8;

  std::vector<float> rgb_input;
  std::vector<float> bgr_input;
  ASSERT_TRUE(Ufldv2Detector::Preprocess(rgb, &rgb_input));
  ASSERT_TRUE(Ufldv2Detector::Preprocess(bgr, &bgr_input));
  EXPECT_EQ(rgb_input, bgr_input);
}

TEST(Ufldv2DecoderTest, DecodesRowAnchorsIntoSourceCoordinates) {
  std::vector<uint8_t> bytes;
  const ImageView image = MakeImage(&bytes);
  const Ufldv2TensorOutputs outputs = MakeOutputs();
  LaneDetectionResult result;

  EXPECT_TRUE(Ufldv2Decoder().Decode(image, outputs, &result));
  ASSERT_EQ(result.lanes.size(), 1U);
  EXPECT_EQ(result.lanes[0].candidate_id, 1U);
  ASSERT_EQ(result.lanes[0].image_points.size(), 3U);
  EXPECT_NEAR(result.lanes[0].image_points.front().y, 664.4F, 1.0F);
  EXPECT_NEAR(result.lanes[0].image_points.back().x, 803.5F, 1.0F);
  EXPECT_NEAR(result.lanes[0].image_points.back().y, 999.0F, 0.01F);
  EXPECT_EQ(result.lanes[0].position, LanePosition::kEgoRight);
}

TEST(Ufldv2DecoderTest, RejectsUnexpectedTensorShape) {
  std::vector<uint8_t> bytes;
  const ImageView image = MakeImage(&bytes);
  Ufldv2TensorOutputs outputs = MakeOutputs();
  outputs.loc_col.shape[1] = 99;
  LaneDetectionResult result;

  EXPECT_FALSE(Ufldv2Decoder().Decode(image, outputs, &result));
}

}  // namespace
}  // namespace lane
}  // namespace apollo
