// Copyright 2026 WheelOS. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "modules/lidar_semantic_segmentation/inference/cylinder_sparse.h"

namespace apollo {
namespace lidar_semantic_segmentation {
namespace cylinder {

void SpconvPairs(const SparseTensor& input, bool submanifold,
                 SparseRelation* relation);
at::Tensor SpconvApply(const at::Tensor& features, const at::Tensor& weights,
                       const SparseRelation& relation, bool inverse);

}  // namespace cylinder
}  // namespace lidar_semantic_segmentation
}  // namespace apollo
