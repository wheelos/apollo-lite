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

#include <cmath>
#include <sstream>

namespace apollo {
namespace bevfusion {

const std::array<const char*, kCameraCount> kCameraOrder = {
    "FRONT", "FRONT_RIGHT", "FRONT_LEFT", "BACK", "BACK_LEFT", "BACK_RIGHT"};

namespace {

void SetError(const std::string& message, std::string* error) {
  if (error != nullptr) {
    *error = message;
  }
}

bool IsFiniteMatrix(const std::array<float, kCameraCount * 16>& matrices) {
  for (float value : matrices) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

bool HasInvertibleUpper3x3(const float* matrix) {
  const double determinant = static_cast<double>(matrix[0]) *
                                 (static_cast<double>(matrix[5]) * matrix[10] -
                                  static_cast<double>(matrix[9]) * matrix[6]) -
                             static_cast<double>(matrix[1]) *
                                 (static_cast<double>(matrix[4]) * matrix[10] -
                                  static_cast<double>(matrix[6]) * matrix[8]) +
                             static_cast<double>(matrix[2]) *
                                 (static_cast<double>(matrix[4]) * matrix[9] -
                                  static_cast<double>(matrix[5]) * matrix[8]);
  return std::isfinite(determinant) && std::abs(determinant) > 1.0e-8;
}

std::uint64_t TimestampDistance(std::uint64_t left, std::uint64_t right) {
  return left >= right ? left - right : right - left;
}

}  // namespace

bool ValidateInput(const BevFusionInputView& input, const InputLimits& limits,
                   std::string* error) {
  if (input.timestamp_ns == 0) {
    SetError("reference timestamp must be nonzero", error);
    return false;
  }
  if (limits.max_timestamp_skew_ns == 0) {
    SetError("maximum timestamp skew must be configured and nonzero", error);
    return false;
  }
  if (limits.max_point_count == 0 || limits.max_point_count > kMaxPointCount) {
    SetError("configured point limit must be in [1, 300000]", error);
    return false;
  }
  if (input.point_count == 0 || input.point_count > limits.max_point_count) {
    std::ostringstream message;
    message << "LiDAR point count " << input.point_count
            << " is empty or exceeds configured limit "
            << limits.max_point_count;
    SetError(message.str(), error);
    return false;
  }
  if (input.point_feature_count != kPointFeatureCount) {
    SetError("LiDAR input must contain exactly five packed features per point",
             error);
    return false;
  }
  if (input.lidar_points == nullptr) {
    SetError("LiDAR point buffer is missing", error);
    return false;
  }

  for (std::size_t camera = 0; camera < kCameraCount; ++camera) {
    const CameraImageView& image = input.cameras[camera];
    if (image.camera_name == nullptr ||
        std::string(image.camera_name) != kCameraOrder[camera]) {
      SetError(
          "camera order must be FRONT, FRONT_RIGHT, FRONT_LEFT, BACK, "
          "BACK_LEFT, BACK_RIGHT",
          error);
      return false;
    }
    if (image.timestamp_ns == 0 ||
        TimestampDistance(image.timestamp_ns, input.timestamp_ns) >
            limits.max_timestamp_skew_ns) {
      SetError(std::string("missing or unsynchronized camera timestamp for ") +
                   kCameraOrder[camera],
               error);
      return false;
    }
    if (image.width != kImageWidth || image.height != kImageHeight ||
        image.stride_bytes != kImageStride ||
        image.pixel_format != PixelFormat::kRgb8Hwc ||
        image.byte_count != kImageByteCount || image.data == nullptr) {
      SetError(std::string("camera image must be packed RGB8 HWC 1600x900 "
                           "with 4800-byte rows: ") +
                   kCameraOrder[camera],
               error);
      return false;
    }
  }

  const std::array<const std::array<float, kCameraCount * 16>*, 4> matrices = {
      &input.camera_to_lidar, &input.camera_intrinsics, &input.lidar_to_image,
      &input.image_augmentation};
  for (const auto* matrix : matrices) {
    if (!IsFiniteMatrix(*matrix)) {
      SetError("calibration matrices must contain only finite values", error);
      return false;
    }
  }
  for (std::size_t camera = 0; camera < kCameraCount; ++camera) {
    if (!HasInvertibleUpper3x3(&input.camera_intrinsics[camera * 16]) ||
        !HasInvertibleUpper3x3(&input.image_augmentation[camera * 16])) {
      SetError(std::string("singular camera calibration matrix for ") +
                   kCameraOrder[camera],
               error);
      return false;
    }
  }

  const std::size_t feature_value_count =
      input.point_count * input.point_feature_count;
  for (std::size_t i = 0; i < feature_value_count; ++i) {
    if (!std::isfinite(input.lidar_points[i])) {
      SetError("LiDAR input contains a non-finite feature value", error);
      return false;
    }
  }
  return true;
}

}  // namespace bevfusion
}  // namespace apollo
