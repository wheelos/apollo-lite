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

//  Created Date: 2026-02-09
//  Author: daohu527

#pragma once

#include "NvInferRuntime.h"

namespace nvinfer1 {
#if NV_TENSORRT_MAJOR >= 10
using TRTDim_t = int64_t;
#else
using TRTDim_t = int32_t;
#endif

struct DimsNCHW : public Dims {
  DimsNCHW() { this->nbDims = 4; }
  DimsNCHW(TRTDim_t n, TRTDim_t c, TRTDim_t h, TRTDim_t w) {
    this->nbDims = 4;
    this->d[0] = n;
    this->d[1] = c;
    this->d[2] = h;
    this->d[3] = w;
  }
  TRTDim_t n() const { return this->d[0]; }
  TRTDim_t c() const { return this->d[1]; }
  TRTDim_t h() const { return this->d[2]; }
  TRTDim_t w() const { return this->d[3]; }
};

struct DimsCHW : public Dims {
  DimsCHW() { this->nbDims = 3; }
  DimsCHW(TRTDim_t c, TRTDim_t h, TRTDim_t w) {
    this->nbDims = 3;
    this->d[0] = c;
    this->d[1] = h;
    this->d[2] = w;
  }
  TRTDim_t c() const { return this->d[0]; }
  TRTDim_t h() const { return this->d[1]; }
  TRTDim_t w() const { return this->d[2]; }
};

}  // namespace nvinfer1
