// Copyright 2026 WheelOS. All Rights Reserved.
#include "modules/perception/camera/common/image_input_validation.h"

#include <cstdint>

#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace camera {

bool ValidateImageInput(const drivers::Image& image, int expected_height,
                        int expected_width) {
  if (expected_height <= 0 || expected_width <= 0 ||
      image.height() != static_cast<uint32_t>(expected_height) ||
      image.width() != static_cast<uint32_t>(expected_width)) {
    AERROR << "Image resolution disagrees with initialized calibration: "
           << image.width() << "x" << image.height() << ", expected "
           << expected_width << "x" << expected_height;
    return false;
  }
  uint64_t channels = 0;
  if (image.encoding() == "rgb8" || image.encoding() == "bgr8") {
    channels = 3;
  } else if (image.encoding() == "gray" || image.encoding() == "y") {
    channels = 1;
  } else {
    AERROR << "Unsupported image encoding: " << image.encoding();
    return false;
  }
  const uint64_t packed_step = static_cast<uint64_t>(image.width()) * channels;
  if (image.step() != packed_step ||
      image.data().size() != packed_step * image.height()) {
    AERROR << "Image requires packed rows and an exact payload: step="
           << image.step() << ", packed_step=" << packed_step
           << ", data_size=" << image.data().size();
    return false;
  }
  return true;
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
