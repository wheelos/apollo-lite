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

#include <limits>
#include <string>

namespace apollo {
namespace camera_semantic_segmentation {

namespace {

constexpr uint32_t kMaxImageDimension = 16384;
constexpr std::size_t kRgbChannels = 3;

bool SafeMultiply(std::size_t left, std::size_t right, std::size_t* product) {
  if (product == nullptr ||
      (right != 0 && left > std::numeric_limits<std::size_t>::max() / right)) {
    return false;
  }
  *product = left * right;
  return true;
}

}  // namespace

bool ImageView::ExpectedByteCount(uint32_t width, uint32_t height,
                                  std::size_t* byte_count) {
  std::size_t pixels = 0;
  return width != 0 && height != 0 && width <= kMaxImageDimension &&
         height <= kMaxImageDimension &&
         SafeMultiply(static_cast<std::size_t>(width),
                      static_cast<std::size_t>(height), &pixels) &&
         SafeMultiply(pixels, kRgbChannels, byte_count);
}

bool ImageView::Validate(std::string* error) const {
  std::size_t expected_byte_count = 0;
  if (!ExpectedByteCount(width, height, &expected_byte_count)) {
    if (error != nullptr) {
      *error = "image dimensions are invalid or overflow the payload size";
    }
    return false;
  }
  if (bytes == nullptr) {
    if (error != nullptr) {
      *error = "image payload pointer is null";
    }
    return false;
  }
  if (byte_count != expected_byte_count) {
    if (error != nullptr) {
      *error = "image payload byte count does not match packed RGB/BGR pixels";
    }
    return false;
  }
  if (camera_name.empty()) {
    if (error != nullptr) {
      *error = "camera name is empty";
    }
    return false;
  }
  return true;
}

bool ParseImageEncoding(const std::string& encoding, ImageEncoding* result) {
  if (result == nullptr) {
    return false;
  }
  if (encoding == "rgb8") {
    *result = ImageEncoding::kRgb8;
    return true;
  }
  if (encoding == "bgr8") {
    *result = ImageEncoding::kBgr8;
    return true;
  }
  return false;
}

}  // namespace camera_semantic_segmentation
}  // namespace apollo
