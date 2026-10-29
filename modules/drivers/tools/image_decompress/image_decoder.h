// Copyright 2026 The WheelOS Authors. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "wheelos_msgs/sensor_msgs/sensor_image.pb.h"

namespace apollo {
namespace image_decompress {

bool DecodeImage(const drivers::CompressedImage& compressed,
                 drivers::Image* image);

}  // namespace image_decompress
}  // namespace apollo
