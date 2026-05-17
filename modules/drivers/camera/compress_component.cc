/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/drivers/camera/compress_component.h"

#include <exception>
#include <vector>

#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc/imgproc.hpp"

namespace apollo {
namespace drivers {
namespace camera {

bool CompressComponent::Init() {
  if (!GetProtoConfig(&config_)) {
    AERROR << "Parse config file failed: " << ConfigFilePath();
    return false;
  }
  AINFO << "Camera config: \n" << config_.DebugString();
  try {
    image_pool_.reset(new CCObjectPool<CompressedImage>(
        config_.compress_conf().image_pool_size()));
    image_pool_->ConstructAll();
  } catch (const std::bad_alloc& e) {
    AERROR << e.what();
    return false;
  }

  // Reserve a reusable compression buffer capacity for typical 1080p frames.
  compressed_buffer_reserve_bytes_ = 2 * 1024 * 1024;

  // JPEG quality tuned for throughput/size balance.
  compress_params_ = {
      cv::IMWRITE_JPEG_QUALITY, 80,
      cv::IMWRITE_JPEG_OPTIMIZE, 1,
  };

  writer_ = node_->CreateWriter<CompressedImage>(
      config_.compress_conf().output_channel());
  return true;
}

bool CompressComponent::Proc(const std::shared_ptr<Image>& image) {
  if (!image) {
    AERROR << "Input image is null";
    return false;
  }

  const double resize_ratio = config_.compress_conf().resize_ratio();
  const bool need_resize = resize_ratio > 0.0 && resize_ratio < 1.0;
  const std::string& encoding = image->encoding();

  cv::Mat image_view;
  cv::Mat resized_mat;
  cv::Mat bgr_mat;

  if (encoding == "yuyv") {
    image_view = cv::Mat(image->height(), image->width(), CV_8UC2,
                         const_cast<char*>(image->data().data()),
                         image->step());
    const cv::Mat* src = &image_view;
    if (need_resize) {
      cv::resize(image_view, resized_mat, cv::Size(), resize_ratio,
                 resize_ratio);
      src = &resized_mat;
    }
    cv::cvtColor(*src, bgr_mat, cv::COLOR_YUV2BGR_YUYV);
  } else if (encoding == "rgb8" || encoding == "bgr8") {
    image_view = cv::Mat(image->height(), image->width(), CV_8UC3,
                         const_cast<char*>(image->data().data()),
                         image->step());
    const cv::Mat* src = &image_view;
    if (need_resize) {
      cv::resize(image_view, resized_mat, cv::Size(), resize_ratio,
                 resize_ratio);
      src = &resized_mat;
    }
    if (encoding == "rgb8") {
      cv::cvtColor(*src, bgr_mat, cv::COLOR_RGB2BGR);
    } else {
      bgr_mat = *src;
    }
  } else {
    AERROR << "Unsupported image encoding for compression: " << encoding;
    return false;
  }

  thread_local std::vector<uint8_t> compressed_buffer;
  if (compressed_buffer.capacity() < compressed_buffer_reserve_bytes_) {
    compressed_buffer.reserve(compressed_buffer_reserve_bytes_);
  }

  try {
    compressed_buffer.clear();
    if (!cv::imencode(".jpg", bgr_mat, compressed_buffer,
                      compress_params_)) {
      AERROR << "JPEG compression failed";
      return false;
    }

    auto out_msg = image_pool_->GetObject();
    if (!out_msg) {
      AERROR << "Compressed image pool is exhausted";
      return false;
    }
    out_msg->mutable_header()->CopyFrom(image->header());
    out_msg->set_format("jpeg");
    out_msg->set_data(compressed_buffer.data(), compressed_buffer.size());
    writer_->Write(out_msg);

  } catch (const cv::Exception& e) {
    AERROR << "OpenCV Exception: " << e.what();
    return false;
  }
  return true;
}

}  // namespace camera
}  // namespace drivers
}  // namespace apollo
