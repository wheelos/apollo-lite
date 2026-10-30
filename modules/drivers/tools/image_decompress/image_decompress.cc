/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/drivers/tools/image_decompress/image_decompress.h"

#include <memory>

#include "modules/drivers/tools/image_decompress/image_decoder.h"

namespace apollo {
namespace image_decompress {

using apollo::drivers::Image;

bool ImageDecompressComponent::Init() {
  if (!GetProtoConfig(&config_)) {
    AERROR << "Parse config file failed: " << ConfigFilePath();
    return false;
  }
  AINFO << "Decompress config: \n" << config_.DebugString();
  writer_ = node_->CreateWriter<Image>(config_.channel_name());
  if (!writer_) {
    AERROR << "Failed to create decompressed image writer.";
    return false;
  }
  return true;
}

bool ImageDecompressComponent::Proc(
    const std::shared_ptr<apollo::drivers::CompressedImage>& compressed_image) {
  auto image = std::make_shared<Image>();
  if (!compressed_image) {
    AERROR << "Missing compressed image.";
    return false;
  }
  if (!DecodeImage(*compressed_image, image.get())) {
    return false;
  }
  if (!writer_ || !writer_->Write(image)) {
    AERROR << "Failed to publish decompressed image.";
    return false;
  }
  return true;
}

}  // namespace image_decompress
}  // namespace apollo
