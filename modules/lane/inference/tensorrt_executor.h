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

#include <memory>
#include <string>
#include <vector>

#include "modules/lane/inference/ufldv2.h"

namespace apollo {
namespace lane {

struct TensorRtEngineOptions {
  std::string engine_path;
  int device_id = 0;
  Ufldv2TensorNames tensor_names;
};

class TensorRtUfldv2Executor final : public Ufldv2Executor {
 public:
  TensorRtUfldv2Executor();
  ~TensorRtUfldv2Executor() override;

  TensorRtUfldv2Executor(const TensorRtUfldv2Executor&) = delete;
  TensorRtUfldv2Executor& operator=(const TensorRtUfldv2Executor&) = delete;

  bool Init(const TensorRtEngineOptions& options);
  bool Run(const std::vector<float>& input,
           Ufldv2TensorOutputs* outputs) override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace lane
}  // namespace apollo
