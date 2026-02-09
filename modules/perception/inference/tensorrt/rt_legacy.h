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

#include <NvInfer.h>
#include <NvInferVersion.h>

#include <cstdint>

namespace nvinfer1 {

#if NV_TENSORRT_MAJOR >= 10
using TRTDim_t = int64_t;
#else
using TRTDim_t = int32_t;
#endif

struct DimsNCHW {
  Dims dims;

  DimsNCHW() {
    dims.nbDims = 4;
    for (int i = 0; i < 4; ++i) dims.d[i] = 0;
  }

  DimsNCHW(TRTDim_t n, TRTDim_t c, TRTDim_t h, TRTDim_t w) {
    dims.nbDims = 4;
    dims.d[0] = n;
    dims.d[1] = c;
    dims.d[2] = h;
    dims.d[3] = w;
  }

  operator const Dims&() const { return dims; }
  operator Dims&() { return dims; }

  TRTDim_t& n() { return dims.d[0]; }
  TRTDim_t& c() { return dims.d[1]; }
  TRTDim_t& h() { return dims.d[2]; }
  TRTDim_t& w() { return dims.d[3]; }
};

struct DimsCHW {
  Dims dims;

  DimsCHW() {
    dims.nbDims = 3;
    dims.d[0] = dims.d[1] = dims.d[2] = 0;
  }

  DimsCHW(TRTDim_t c, TRTDim_t h, TRTDim_t w) {
    dims.nbDims = 3;
    dims.d[0] = c;
    dims.d[1] = h;
    dims.d[2] = w;
  }

  operator const Dims&() const { return dims; }
};

}  // namespace nvinfer1
