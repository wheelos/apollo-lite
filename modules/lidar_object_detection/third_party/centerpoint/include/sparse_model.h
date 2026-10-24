// Copyright 2026 WheelOS All Rights Reserved.
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

#include <cstdint>
#include <string>
#include <vector>

struct SparseModelLayer {
  std::string op;
  std::vector<std::string> inputs;
  std::string output;
  std::vector<int> kernel, stride, padding, dilation, input_grid, output_grid;
  std::vector<int> dense_shape;
  std::vector<int64_t> weight_shape;
  std::vector<uint16_t> weight_data, bias_data;
  int bound = 0;
  bool subm = false;
  bool relu = false;
  bool xyz = false;
};

struct SparseModel {
  std::string input, output;
  std::vector<int64_t> output_shape;
  std::vector<SparseModelLayer> layers;
};

SparseModel LoadSparseModel(const std::string& path);
