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

//  Created Date: 2026-08-28
//  Author: daohu527

#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "modules/lane/types/image_frame.h"
#include "modules/lane/types/lane_types.h"

namespace apollo {
namespace lane {

constexpr int kUfldv2ModelWidth = 1600;
constexpr int kUfldv2ModelHeight = 320;
constexpr size_t kUfldv2InputElementCount =
    3U * kUfldv2ModelWidth * kUfldv2ModelHeight;

struct Ufldv2Tensor {
  std::vector<int64_t> shape;
  std::vector<float> values;
};

struct Ufldv2TensorOutputs {
  Ufldv2Tensor loc_row;
  Ufldv2Tensor loc_col;
  Ufldv2Tensor exist_row;
  Ufldv2Tensor exist_col;
};

struct Ufldv2TensorNames {
  std::string input = "input";
  std::string loc_row = "loc_row";
  std::string loc_col = "loc_col";
  std::string exist_row = "exist_row";
  std::string exist_col = "exist_col";
};

class Ufldv2Executor {
 public:
  virtual ~Ufldv2Executor() = default;
  virtual bool Run(const std::vector<float>& input,
                   Ufldv2TensorOutputs* outputs) = 0;
};

class Ufldv2Decoder {
 public:
  bool Decode(const ImageView& image, const Ufldv2TensorOutputs& outputs,
              LaneDetectionResult* result) const;

  static bool ValidateOutputShapes(const Ufldv2TensorOutputs& outputs);
};

class Ufldv2Detector {
 public:
  explicit Ufldv2Detector(Ufldv2Executor* executor) : executor_(executor) {}

  bool Detect(const ImageView& image, LaneDetectionResult* result) const;
  static bool Preprocess(const ImageView& image, std::vector<float>* input);

 private:
  Ufldv2Executor* executor_ = nullptr;
  Ufldv2Decoder decoder_;
};

}  // namespace lane
}  // namespace apollo
