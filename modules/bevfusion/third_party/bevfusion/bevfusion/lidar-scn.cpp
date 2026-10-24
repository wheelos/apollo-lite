/*
 * SPDX-FileCopyrightText: Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include "lidar-scn.hpp"

#include "sparse_backbone.h"

namespace bevfusion {
namespace lidar {

class SCNImplement : public SCN {
 public:
  bool init(const SCNParameter& param) {
    this->param_ = param;
    voxelization_ = create_voxelization(param_.voxelization);
    if (voxelization_ == nullptr) return false;

    native_scn_ = LoadSparseBackbone(param_.model);
    return native_scn_ != nullptr;
  }

  virtual const nvtype::half* forward(const nvtype::half* points, unsigned int num_points, void* stream) override {
    voxelization_->forward(points, num_points, stream, param_.order);
    return static_cast<const nvtype::half*>(native_scn_->Forward(
        const_cast<void*>(voxelization_->features()),
        const_cast<void*>(voxelization_->indices()),
        voxelization_->num_voxels(), voxelization_->grid_size(),
        static_cast<cudaStream_t>(stream)));
  }

  virtual std::vector<int64_t> shape() override {
    return native_scn_->Shape();
  }

 private:
  SCNParameter param_;
  std::shared_ptr<Voxelization> voxelization_;
  std::unique_ptr<SparseBackbone> native_scn_;
};

std::shared_ptr<SCN> create_scn(const SCNParameter& param) {
  std::shared_ptr<SCNImplement> instance(new SCNImplement());
  if (!instance->init(param)) {
    instance.reset();
  }
  return instance;
}

};  // namespace lidar
};  // namespace bevfusion