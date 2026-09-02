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

#include "modules/camera_semantic_segmentation/inference/segformer.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace apollo {
namespace camera_semantic_segmentation {

namespace {

void SetError(const std::string& message, std::string* error) {
  if (error != nullptr) {
    *error = message;
  }
}

bool IsSupportedLayout(const std::string& layout) {
  return layout == "NCHW" || layout == "NHWC";
}

}  // namespace

bool SegFormerPreprocessor::Init(const ImagePreprocessOptions& options,
                                 std::string* error) {
  if (options.width == 0U || options.height == 0U) {
    SetError("target preprocess width and height must be non-zero", error);
    return false;
  }
  if (options.channel_mean.size() != kSegFormerInputChannels ||
      options.channel_std.size() != kSegFormerInputChannels) {
    SetError("channel_mean and channel_std must have exactly 3 values", error);
    return false;
  }
  for (std::size_t idx = 0; idx < kSegFormerInputChannels; ++idx) {
    if (options.channel_std[idx] <= 0.0F) {
      SetError("channel_std must be positive", error);
      return false;
    }
  }
  options_ = options;
  initialized_ = true;
  return true;
}

bool SegFormerPreprocessor::Preprocess(const ImageView& image,
                                       std::vector<float>* input,
                                       std::string* error) const {
  if (!initialized_) {
    SetError("preprocessor is not initialized", error);
    return false;
  }
  if (input == nullptr) {
    SetError("preprocessed input destination is null", error);
    return false;
  }
  if (!image.Validate(error)) {
    return false;
  }

  const uint32_t target_w = options_.width;
  const uint32_t target_h = options_.height;
  const std::size_t plane_size =
      static_cast<std::size_t>(target_w) * static_cast<std::size_t>(target_h);
  input->assign(plane_size * kSegFormerInputChannels, 0.0F);

  cv::Mat source(static_cast<int>(image.height), static_cast<int>(image.width),
                 CV_8UC3, const_cast<uint8_t*>(image.bytes));
  cv::Mat resized;
  cv::resize(source, resized,
             cv::Size(static_cast<int>(target_w), static_cast<int>(target_h)),
             0.0, 0.0, cv::INTER_LINEAR);

  std::vector<cv::Mat> source_channels;
  cv::split(resized, source_channels);

  for (std::size_t c = 0; c < kSegFormerInputChannels; ++c) {
    std::size_t src_c = c;
    if (options_.to_rgb) {
      src_c = (image.encoding == ImageEncoding::kRgb8) ? c : (2U - c);
    } else {
      src_c = (image.encoding == ImageEncoding::kBgr8) ? c : (2U - c);
    }
    cv::Mat destination(static_cast<int>(target_h), static_cast<int>(target_w),
                        CV_32FC1, input->data() + c * plane_size);
    source_channels[src_c].convertTo(
        destination, CV_32FC1,
        1.0 / static_cast<double>(options_.channel_std[c]),
        -static_cast<double>(options_.channel_mean[c]) /
            static_cast<double>(options_.channel_std[c]));
  }
  return true;
}

bool SegFormerDecoder::Init(const SegFormerModelOptions& options,
                            std::string* error) {
  if (options.num_classes == 0U) {
    SetError("num_classes must be positive", error);
    return false;
  }
  if (!IsSupportedLayout(options.output_layout)) {
    SetError("output_layout must be NCHW or NHWC", error);
    return false;
  }
  options_ = options;
  output_is_nhwc_ = (options.output_layout == "NHWC");
  initialized_ = true;
  return true;
}

std::size_t SegFormerDecoder::LogitOffset(uint32_t y, uint32_t x, uint32_t cls,
                                          uint32_t height,
                                          uint32_t width) const {
  const std::size_t h = height;
  const std::size_t w = width;
  const std::size_t c = options_.num_classes;
  if (output_is_nhwc_) {
    return ((static_cast<std::size_t>(y) * w + x) * c) + cls;
  }
  return ((static_cast<std::size_t>(cls) * h + y) * w) + x;
}

bool SegFormerDecoder::Decode(const SegFormerTensor& logits,
                              SegmentationMask* mask,
                              std::string* error) const {
  if (!initialized_) {
    SetError("decoder is not initialized", error);
    return false;
  }
  if (mask == nullptr) {
    SetError("segmentation mask output is null", error);
    return false;
  }

  uint32_t height = options_.preprocess.height;
  uint32_t width = options_.preprocess.width;

  if (logits.shape.size() == 4U) {
    if (output_is_nhwc_) {
      height = static_cast<uint32_t>(logits.shape[1]);
      width = static_cast<uint32_t>(logits.shape[2]);
    } else {
      height = static_cast<uint32_t>(logits.shape[2]);
      width = static_cast<uint32_t>(logits.shape[3]);
    }
  }

  const std::size_t pixel_count =
      static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  const std::size_t expected_count =
      pixel_count * static_cast<std::size_t>(options_.num_classes);

  if (logits.values.size() != expected_count) {
    SetError("logits size does not match expected output shape", error);
    return false;
  }

  mask->width = width;
  mask->height = height;
  mask->labels.assign(pixel_count, 0U);
  mask->confidences.assign(pixel_count, 0.0F);

  for (uint32_t y = 0; y < height; ++y) {
    for (uint32_t x = 0; x < width; ++x) {
      const std::size_t pixel_index = static_cast<std::size_t>(y) * width + x;
      uint32_t best_label = 0U;
      float best_logit = -std::numeric_limits<float>::infinity();

      for (uint32_t cls = 0U; cls < options_.num_classes; ++cls) {
        const float val = logits.values[LogitOffset(y, x, cls, height, width)];
        if (!std::isfinite(val)) {
          SetError("SegFormer output contains non-finite values", error);
          return false;
        }
        if (val > best_logit) {
          best_logit = val;
          best_label = cls;
        }
      }

      double exp_sum = 0.0;
      for (uint32_t cls = 0U; cls < options_.num_classes; ++cls) {
        exp_sum += std::exp(static_cast<double>(
            logits.values[LogitOffset(y, x, cls, height, width)] -
            best_logit));
      }

      mask->labels[pixel_index] = static_cast<uint8_t>(best_label);
      mask->confidences[pixel_index] =
          exp_sum > 0.0 ? static_cast<float>(1.0 / exp_sum) : 0.0F;
    }
  }
  return true;
}

bool SegFormerSegmenter::Init(const SegFormerModelOptions& options,
                              std::string* error) {
  if (executor_ == nullptr) {
    SetError("SegFormer executor is null", error);
    return false;
  }
  if (!preprocessor_.Init(options.preprocess, error)) {
    return false;
  }
  if (!decoder_.Init(options, error)) {
    return false;
  }
  if (!executor_->Init(options)) {
    SetError("failed to initialize SegFormer executor", error);
    return false;
  }
  options_ = options;
  initialized_ = true;
  return true;
}

bool SegFormerSegmenter::Segment(const ImageView& image,
                                 CameraSemanticSegmentationResult* result,
                                 std::string* error) const {
  if (!initialized_) {
    SetError("SegFormer segmenter is not initialized", error);
    return false;
  }
  if (result == nullptr) {
    SetError("segmentation result is null", error);
    return false;
  }

  std::vector<float> input;
  if (!preprocessor_.Preprocess(image, &input, error)) {
    return false;
  }

  SegFormerTensor logits;
  if (!executor_->Run(input, &logits)) {
    SetError("SegFormer executor inference failed", error);
    return false;
  }

  SegmentationMask mask;
  if (!decoder_.Decode(logits, &mask, error)) {
    return false;
  }

  result->Clear();
  if (image.timestamp_sec > 0.0) {
    result->mutable_header()->set_timestamp_sec(image.timestamp_sec);
  }
  if (image.camera_timestamp_ns > 0) {
    result->mutable_header()->set_camera_timestamp(image.camera_timestamp_ns);
  }
  if (image.sequence_num > 0) {
    result->mutable_header()->set_sequence_num(image.sequence_num);
  }
  if (!image.frame_id.empty()) {
    result->mutable_header()->set_frame_id(image.frame_id);
    result->set_source_frame_id(image.frame_id);
  }
  result->set_source_topic(options_.source_topic);
  result->set_camera_name(options_.camera_name);
  result->set_width(mask.width);
  result->set_height(mask.height);
  result->set_num_classes(options_.num_classes);
  result->set_mask(mask.labels.data(), mask.labels.size());

  result->mutable_pixel_labels()->Reserve(
      static_cast<int>(mask.labels.size()));
  for (const uint8_t label : mask.labels) {
    result->add_pixel_labels(static_cast<uint32_t>(label));
  }

  result->mutable_pixel_confidences()->Reserve(
      static_cast<int>(mask.confidences.size()));
  for (const float conf : mask.confidences) {
    result->add_pixel_confidences(conf);
  }

  return true;
}

}  // namespace camera_semantic_segmentation
}  // namespace apollo
