// Copyright 2026 WheelOS. All Rights Reserved.
#pragma once

#include "wheelos_msgs/sensor_msgs/sensor_image.pb.h"

namespace apollo {
namespace perception {
namespace camera {

// The current provider accepts packed 8-bit images at calibration resolution.
bool ValidateImageInput(const drivers::Image& image, int expected_height,
                        int expected_width);

}  // namespace camera
}  // namespace perception
}  // namespace apollo
