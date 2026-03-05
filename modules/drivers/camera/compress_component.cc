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

  // 1. 预分配内存 (关键)
  // 假设 1080P RGB 图片约为 6MB，压缩后通常 < 1MB。预留 2MB 足够，避免
  // realloc。
  compressed_buffer_.reserve(2 * 1024 * 1024);

  // 2. 设置压缩参数
  compress_params_ = {
      cv::IMWRITE_JPEG_QUALITY, 80,  // 工业实践：95太高，80肉眼难辨但体积减半
      cv::IMWRITE_JPEG_OPTIMIZE, 1   // 启用 Huffman 优化 (如果 OpenCV 版本支持)
  };

  writer_ = node_->CreateWriter<CompressedImage>(
      config_.compress_conf().output_channel());
  return true;
}

bool CompressComponent::Proc(const std::shared_ptr<Image>& image) {
  // 1. Wrap raw data
  cv::Mat raw_mat(image->height(), image->width(), CV_8UC3,
                  const_cast<char*>(image->data().data()), image->step());

  // 2. Decide resize and color conversion
  // 不要修改原始 const image 的数据
  bool need_resize = (config_.compress_conf().resize_ratio() < 1.0);
  bool need_cvt_color =
      (config_.output_type() ==
       config::OutputType::RGB);  // Assuming we need BGR for JPEG

  cv::Mat processing_mat;  // 局部变量，线程安全

  if (need_resize) {
    double scale = config_.compress_conf().resize_ratio();
    // Resize 会创建新内存，所以 processing_mat 现在是独立的
    cv::resize(raw_mat, processing_mat, cv::Size(), scale, scale);

    if (need_cvt_color) {
      // 在独立内存上做颜色转换，安全
      cv::cvtColor(processing_mat, processing_mat, cv::COLOR_RGB2BGR);
    }
  } else {
    // 没有 Resize
    if (need_cvt_color) {
      // 必须 Clone！因为不能修改 raw_mat
      cv::cvtColor(raw_mat, processing_mat, cv::COLOR_RGB2BGR);
    } else {
      // 既不 resize 也不变色，直接引用原图
      processing_mat = raw_mat;
    }
  }

  // 3. Compress
  try {
    // 使用线程局部存储 (Thread Local Storage) 或者 互斥锁保护成员变量 buffer
    std::lock_guard<std::mutex> lock(mutex_);

    compressed_buffer_.clear();
    if (!cv::imencode(".jpg", processing_mat, compressed_buffer_,
                      compress_params_)) {
      AERROR << "JPEG compression failed";
      return false;
    }

    // 4. Publish
    auto out_msg = image_pool_->GetObject();
    out_msg->mutable_header()->CopyFrom(image->header());
    out_msg->set_format("jpeg");
    out_msg->set_data(compressed_buffer_.data(), compressed_buffer_.size());
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
