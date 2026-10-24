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

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace apollo {
namespace bevfusion {

constexpr std::size_t kCameraCount = 6;
constexpr std::size_t kPointFeatureCount = 5;
constexpr std::size_t kMaxPointCount = 300000;
constexpr std::uint32_t kImageWidth = 1600;
constexpr std::uint32_t kImageHeight = 900;
constexpr std::size_t kImageStride = kImageWidth * 3;
constexpr std::size_t kImageByteCount = kImageStride * kImageHeight;

extern const std::array<const char*, kCameraCount> kCameraOrder;

enum class PixelFormat { kRgb8Hwc, kOther };

struct CameraImageView {
  const char* camera_name = nullptr;
  std::uint64_t timestamp_ns = 0;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::size_t stride_bytes = 0;
  PixelFormat pixel_format = PixelFormat::kOther;
  const std::uint8_t* data = nullptr;
  std::size_t byte_count = 0;
};

struct BevFusionInputView {
  std::uint64_t timestamp_ns = 0;
  std::array<CameraImageView, kCameraCount> cameras;
  const float* lidar_points = nullptr;
  std::size_t point_count = 0;
  std::size_t point_feature_count = 0;
  std::array<float, kCameraCount * 16> camera_to_lidar{};
  std::array<float, kCameraCount * 16> camera_intrinsics{};
  std::array<float, kCameraCount * 16> lidar_to_image{};
  std::array<float, kCameraCount * 16> image_augmentation{};
};

struct InputLimits {
  std::uint64_t max_timestamp_skew_ns = 0;
  std::size_t max_point_count = kMaxPointCount;
};

bool ValidateInput(const BevFusionInputView& input, const InputLimits& limits,
                   std::string* error);

}  // namespace bevfusion
}  // namespace apollo
