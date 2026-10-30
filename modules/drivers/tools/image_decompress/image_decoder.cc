// Copyright 2026 The WheelOS Authors. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0

#include "modules/drivers/tools/image_decompress/image_decoder.h"

#include <vector>

#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

#include "cyber/common/log.h"

namespace apollo {
namespace image_decompress {

bool DecodeImage(const drivers::CompressedImage& compressed,
                 drivers::Image* image) {
  if (image == nullptr || compressed.data().empty()) {
    AERROR << "Missing image output or compressed bytes.";
    return false;
  }
  try {
    const std::vector<uint8_t> bytes(compressed.data().begin(),
                                     compressed.data().end());
    cv::Mat decoded = cv::imdecode(bytes, cv::IMREAD_COLOR);
    if (decoded.empty()) {
      AERROR << "Unable to decode compressed image.";
      return false;
    }
    cv::cvtColor(decoded, decoded, cv::COLOR_BGR2RGB);
    image->Clear();
    image->mutable_header()->CopyFrom(compressed.header());
    image->set_measurement_time(compressed.has_measurement_time()
                                    ? compressed.measurement_time()
                                    : compressed.header().timestamp_sec());
    image->set_width(decoded.cols);
    image->set_height(decoded.rows);
    image->set_encoding("rgb8");
    image->set_step(3 * image->width());
    image->set_data(decoded.data, decoded.step * decoded.rows);
    return true;
  } catch (const cv::Exception& error) {
    AERROR << "Compressed image decoding failed: " << error.what();
    return false;
  }
}

}  // namespace image_decompress
}  // namespace apollo
