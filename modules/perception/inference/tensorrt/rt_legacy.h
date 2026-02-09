
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

#pragma once

#include <NvInferVersion.h>

#include <cstdint>

namespace nvinfer1 {

#if NV_TENSORRT_MAJOR >= 10
using TRTDim_t = int64_t;
// In TRT 10+, Dims2, Dims3, Dims4 are removed. Use Dims base class.
using Dims4_Base = Dims;
using Dims3_Base = Dims;
#else
using TRTDim_t = int32_t;
using Dims4_Base = Dims4;
using Dims3_Base = Dims3;
#endif

class DimsNCHW : public Dims4_Base {
 public:
  DimsNCHW() {
    this->nbDims = 4;
    for (int i = 0; i < 4; ++i) this->d[i] = 0;
  }

  DimsNCHW(TRTDim_t batch_size, TRTDim_t channels, TRTDim_t height,
           TRTDim_t width) {
    this->nbDims = 4;
    this->d[0] = batch_size;
    this->d[1] = channels;
    this->d[2] = height;
    this->d[3] = width;
  }

  TRTDim_t& n() { return d[0]; }
  TRTDim_t n() const { return d[0]; }
  TRTDim_t& c() { return d[1]; }
  TRTDim_t c() const { return d[1]; }
  TRTDim_t& h() { return d[2]; }
  TRTDim_t h() const { return d[2]; }
  TRTDim_t& w() { return d[3]; }
  TRTDim_t w() const { return d[3]; }
};

class DimsCHW : public Dims3_Base {
 public:
  DimsCHW() {
    this->nbDims = 3;
    this->d[0] = 0;
    this->d[1] = 0;
    this->d[2] = 0;
  }

  DimsCHW(TRTDim_t channels, TRTDim_t height, TRTDim_t width) {
    this->nbDims = 3;
    this->d[0] = channels;
    this->d[1] = height;
    this->d[2] = width;
  }

  TRTDim_t& c() { return d[0]; }
  TRTDim_t c() const { return d[0]; }
  TRTDim_t& h() { return d[1]; }
  TRTDim_t h() const { return d[1]; }
  TRTDim_t& w() { return d[2]; }
  TRTDim_t w() const { return d[2]; }

  size_t totalElements() const {
    return static_cast<size_t>(d[0]) * d[1] * d[2];
  }
};

}  // namespace nvinfer1
