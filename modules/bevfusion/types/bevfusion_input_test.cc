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

#include "modules/bevfusion/types/bevfusion_input.h"

#include <array>
#include <limits>
#include <string>
#include <vector>

#include "gtest/gtest.h"

namespace apollo {
namespace bevfusion {
namespace {

BevFusionInputView MakeValidInput(std::vector<std::uint8_t>* image_bytes,
                                  std::array<float, 5>* point_features) {
  BevFusionInputView input;
  input.timestamp_ns = 1000000;
  for (std::size_t camera = 0; camera < kCameraCount; ++camera) {
    input.cameras[camera] = CameraImageView{
        kCameraOrder[camera], input.timestamp_ns, kImageWidth,
        kImageHeight,         kImageStride,       PixelFormat::kRgb8Hwc,
        image_bytes->data(),  kImageByteCount};
    for (std::size_t row = 0; row < 4; ++row) {
      input.camera_to_lidar[camera * 16 + row * 4 + row] = 1.0F;
      input.camera_intrinsics[camera * 16 + row * 4 + row] = 1.0F;
      input.lidar_to_image[camera * 16 + row * 4 + row] = 1.0F;
      input.image_augmentation[camera * 16 + row * 4 + row] = 1.0F;
    }
  }
  input.lidar_points = point_features->data();
  input.point_count = 1;
  input.point_feature_count = kPointFeatureCount;
  return input;
}

InputLimits TestLimits() { return InputLimits{1000, kMaxPointCount}; }

TEST(BevFusionInputTest, AcceptsConfiguredSixCameraInput) {
  std::vector<std::uint8_t> image(kImageByteCount);
  std::array<float, 5> features{};
  const auto input = MakeValidInput(&image, &features);
  std::string error;

  EXPECT_TRUE(ValidateInput(input, TestLimits(), &error)) << error;
}

TEST(BevFusionInputTest, RejectsNonCanonicalCameraOrder) {
  std::vector<std::uint8_t> image(kImageByteCount);
  std::array<float, 5> features{};
  auto input = MakeValidInput(&image, &features);
  input.cameras[1].camera_name = kCameraOrder[2];
  std::string error;

  EXPECT_FALSE(ValidateInput(input, TestLimits(), &error));
  EXPECT_NE(error.find("camera order"), std::string::npos);
}

TEST(BevFusionInputTest, RejectsInvalidRgbStrideAndLayout) {
  std::vector<std::uint8_t> image(kImageByteCount);
  std::array<float, 5> features{};
  auto input = MakeValidInput(&image, &features);
  input.cameras[0].stride_bytes -= 3;
  std::string error;

  EXPECT_FALSE(ValidateInput(input, TestLimits(), &error));
  EXPECT_NE(error.find("packed RGB8 HWC"), std::string::npos);
}

TEST(BevFusionInputTest, RejectsMissingAndUnsynchronizedCamera) {
  std::vector<std::uint8_t> image(kImageByteCount);
  std::array<float, 5> features{};
  auto input = MakeValidInput(&image, &features);
  input.cameras[5].timestamp_ns = 0;
  std::string error;

  EXPECT_FALSE(ValidateInput(input, TestLimits(), &error));
  EXPECT_NE(error.find("unsynchronized"), std::string::npos);

  input.cameras[5].timestamp_ns = input.timestamp_ns + 1001;
  error.clear();
  EXPECT_FALSE(ValidateInput(input, TestLimits(), &error));
}

TEST(BevFusionInputTest, RejectsNonFiveFeatureAndOversizePointClouds) {
  std::vector<std::uint8_t> image(kImageByteCount);
  std::array<float, 5> features{};
  auto input = MakeValidInput(&image, &features);
  std::string error;
  input.point_feature_count = 4;

  EXPECT_FALSE(ValidateInput(input, TestLimits(), &error));
  input.point_feature_count = kPointFeatureCount;
  input.point_count = kMaxPointCount + 1;
  error.clear();
  EXPECT_FALSE(ValidateInput(input, TestLimits(), &error));
  EXPECT_NE(error.find("exceeds"), std::string::npos);
}

TEST(BevFusionInputTest, RejectsNonFiniteAndSingularCalibration) {
  std::vector<std::uint8_t> image(kImageByteCount);
  std::array<float, 5> features{};
  auto input = MakeValidInput(&image, &features);
  input.camera_intrinsics[5] = 0.0F;
  std::string error;

  EXPECT_FALSE(ValidateInput(input, TestLimits(), &error));
  input = MakeValidInput(&image, &features);
  input.lidar_to_image[0] = std::numeric_limits<float>::infinity();
  error.clear();
  EXPECT_FALSE(ValidateInput(input, TestLimits(), &error));
}

TEST(BevFusionInputTest, RejectsNonFinitePointFeature) {
  std::vector<std::uint8_t> image(kImageByteCount);
  std::array<float, 5> features{};
  auto input = MakeValidInput(&image, &features);
  features[2] = std::numeric_limits<float>::quiet_NaN();
  std::string error;

  EXPECT_FALSE(ValidateInput(input, TestLimits(), &error));
  EXPECT_NE(error.find("non-finite"), std::string::npos);
}

}  // namespace
}  // namespace bevfusion
}  // namespace apollo
