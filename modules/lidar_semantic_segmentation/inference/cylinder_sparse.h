// Copyright 2026 WheelOS. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <memory>
#include <vector>

#include <ATen/core/Tensor.h>

#include "modules/lidar_semantic_segmentation/types/cylinder_projection.h"

namespace apollo {
namespace lidar_semantic_segmentation {
namespace cylinder {

using Coordinates = std::vector<CylinderCoordinate>;

struct DeviceCoordinates {
  std::shared_ptr<const Coordinates> coordinates;
  at::Tensor indices;
};

struct SparseTensor {
  at::Tensor features;
  std::shared_ptr<const Coordinates> coordinates;
  CylinderCoordinate shape;
  bool use_spconv = false;
  std::shared_ptr<const DeviceCoordinates> device_coordinates;
};

struct SparseRelation {
  std::shared_ptr<const Coordinates> input;
  std::shared_ptr<const Coordinates> output;
  CylinderCoordinate input_shape;
  CylinderCoordinate output_shape;
  CylinderCoordinate kernel;
  CylinderCoordinate stride;
  CylinderCoordinate padding;
  at::Tensor pairs;
  std::vector<int64_t> offsets;
  at::Tensor pair_counts;
  std::shared_ptr<const DeviceCoordinates> input_device_coordinates;
  std::shared_ptr<const DeviceCoordinates> output_device_coordinates;
};

// Coordinates are immutable; discard this cache after each scan/TTA vote.
using SparseIndiceCache = std::vector<std::shared_ptr<const SparseRelation>>;

bool SpconvAvailable();

// Weights use [k0,k1,k2,input_channels,output_channels], FP32.
SparseTensor Convolve(const SparseTensor& input, const at::Tensor& weights,
                      const at::Tensor& bias, const CylinderCoordinate& stride,
                      const CylinderCoordinate& padding, bool submanifold,
                      SparseRelation* relation = nullptr,
                      SparseIndiceCache* cache = nullptr);
SparseTensor InverseConvolve(const SparseTensor& input,
                             const at::Tensor& weights,
                             const SparseRelation& relation);

}  // namespace cylinder
}  // namespace lidar_semantic_segmentation
}  // namespace apollo
