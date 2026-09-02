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

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "modules/camera_semantic_segmentation/proto/camera_semantic_segmentation.pb.h"
#include "modules/camera_semantic_segmentation/types/semantic_types.h"

namespace apollo {
namespace camera_semantic_segmentation {

struct SegFormerTensorNames {
  std::string input = "input";
  std::string output = "output";
};

struct SegFormerModelOptions {
  std::string engine_path;
  int device_id = 0;
  uint32_t num_classes = 19U;
  std::string output_layout = "NCHW";
  std::string camera_name = "front_6mm";
  std::string source_topic;
  SegFormerTensorNames tensor_names;
  ImagePreprocessOptions preprocess;
};

struct SegFormerTensor {
  std::vector<int64_t> shape;
  std::vector<float> values;
};

class SegFormerExecutor {
 public:
  virtual ~SegFormerExecutor() = default;
  virtual bool Init(const SegFormerModelOptions& options) = 0;
  virtual bool Run(const std::vector<float>& input,
                   SegFormerTensor* output) = 0;
};

class SegFormerPreprocessor {
 public:
  bool Init(const ImagePreprocessOptions& options, std::string* error);
  bool Preprocess(const ImageView& image, std::vector<float>* input,
                  std::string* error) const;

  const ImagePreprocessOptions& options() const { return options_; }

 private:
  ImagePreprocessOptions options_;
  bool initialized_ = false;
};

class SegFormerDecoder {
 public:
  bool Init(const SegFormerModelOptions& options, std::string* error);
  bool Decode(const SegFormerTensor& logits, SegmentationMask* mask,
              std::string* error) const;

 private:
  std::size_t LogitOffset(uint32_t y, uint32_t x, uint32_t cls,
                          uint32_t height, uint32_t width) const;

  SegFormerModelOptions options_;
  bool output_is_nhwc_ = false;
  bool initialized_ = false;
};

class SegFormerSegmenter {
 public:
  explicit SegFormerSegmenter(SegFormerExecutor* executor)
      : executor_(executor) {}

  bool Init(const SegFormerModelOptions& options, std::string* error);

  bool Segment(const ImageView& image,
               CameraSemanticSegmentationResult* result,
               std::string* error) const;

  const SegFormerModelOptions& options() const { return options_; }

 private:
  SegFormerExecutor* executor_ = nullptr;
  SegFormerModelOptions options_;
  SegFormerPreprocessor preprocessor_;
  SegFormerDecoder decoder_;
  bool initialized_ = false;
};

}  // namespace camera_semantic_segmentation
}  // namespace apollo
