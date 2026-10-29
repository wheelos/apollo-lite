// Copyright 2026 WheelOS. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "modules/lidar_semantic_segmentation/inference/cylinder_sparse.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include <ATen/ATen.h>
#include <c10/core/impl/VirtualGuardImpl.h>

#ifdef CYLINDER3D_WITH_SPCONV
#include "modules/lidar_semantic_segmentation/inference/cylinder_spconv.h"
#endif

namespace apollo {
namespace lidar_semantic_segmentation {
namespace cylinder {
namespace {

using Clock = std::chrono::steady_clock;

bool Profiling() {
  static const bool enabled = std::getenv("CYLINDER3D_PROFILE") != nullptr;
  return enabled;
}

void Synchronize(const at::Tensor& features) {
  if (Profiling() && features.is_cuda()) {
    c10::impl::VirtualGuardImpl(features.device().type())
        .synchronizeDevice(features.device().index());
  }
}

int64_t Key(const CylinderCoordinate& coordinate,
            const CylinderCoordinate& shape) {
  return (coordinate[0] * shape[1] + coordinate[1]) * shape[2] + coordinate[2];
}

bool InBounds(const CylinderCoordinate& coordinate,
              const CylinderCoordinate& shape) {
  for (int axis = 0; axis < 3; ++axis) {
    if (coordinate[axis] < 0 || coordinate[axis] >= shape[axis]) {
      return false;
    }
  }
  return true;
}

class CoordinateLookup {
 public:
  void Reset(const CylinderCoordinate& shape, std::size_t count) {
    // Bound the address table to 32 MiB; huge sparse grids retain hash lookup.
    constexpr int64_t kMaxCells = 8 * 1024 * 1024;
    int64_t cells = 1;
    for (auto extent : shape) {
      if (cells > kMaxCells / extent) {
        cells = 0;
        break;
      }
      cells *= extent;
    }
    dense_.assign(cells, -1);
    hash_.clear();
    if (cells == 0) {
      hash_.reserve(count);
    }
  }

  std::pair<int64_t, bool> Insert(int64_t key, int64_t value) {
    if (dense_.empty()) {
      const auto inserted = hash_.emplace(key, value);
      return {inserted.first->second, inserted.second};
    }
    auto& row = dense_[key];
    const bool inserted = row < 0;
    if (inserted) {
      row = static_cast<int32_t>(value);
    }
    return {row, inserted};
  }

  int64_t Find(int64_t key) const {
    if (!dense_.empty()) {
      return dense_[key];
    }
    const auto found = hash_.find(key);
    return found == hash_.end() ? -1 : found->second;
  }

 private:
  std::vector<int32_t> dense_;
  std::unordered_map<int64_t, int64_t> hash_;
};

void Validate(const SparseTensor& input, const at::Tensor& weights) {
  TORCH_CHECK(input.coordinates && input.features.dim() == 2 &&
                  input.features.size(0) ==
                      static_cast<int64_t>(input.coordinates->size()) &&
                  input.features.scalar_type() == at::kFloat &&
                  weights.dim() == 5 && weights.scalar_type() == at::kFloat &&
                  weights.device() == input.features.device() &&
                  weights.size(3) == input.features.size(1),
              "invalid Cylinder3D sparse tensor or convolution weights");
  int64_t volume = 1;
  for (int axis = 0; axis < 3; ++axis) {
    TORCH_CHECK(
        input.shape[axis] > 0 &&
            volume <= std::numeric_limits<int64_t>::max() / input.shape[axis] &&
            weights.size(axis) > 0 && weights.size(axis) <= 3,
        "invalid Cylinder3D sparse grid or kernel");
    volume *= input.shape[axis];
  }
}

at::Tensor ApplyPairs(const at::Tensor& features, const at::Tensor& weights,
                      int64_t count, const SparseRelation& relation,
                      bool inverse = false) {
  if (relation.pair_counts.defined()) {
#ifdef CYLINDER3D_WITH_SPCONV
    return SpconvApply(features, weights, relation, inverse);
#else
    TORCH_CHECK(false, "Cylinder3D spconv is unavailable in a CPU build");
#endif
  }
  auto result = at::zeros({count, weights.size(4)}, features.options());
  auto kernels = weights.reshape({-1, weights.size(3), weights.size(4)});
  const auto inputs = relation.pairs[inverse ? 1 : 0];
  const auto outputs = relation.pairs[inverse ? 0 : 1];
  for (std::size_t kernel = 0; kernel + 1 < relation.offsets.size(); ++kernel) {
    const auto offset = relation.offsets[kernel];
    const auto size = relation.offsets[kernel + 1] - offset;
    if (size == 0) {
      continue;
    }
    auto from = inputs.narrow(0, offset, size);
    auto to = outputs.narrow(0, offset, size);
    result.index_add_(
        0, to, at::matmul(features.index_select(0, from), kernels[kernel]));
  }
  return result;
}

}  // namespace

bool SpconvAvailable() {
#ifdef CYLINDER3D_WITH_SPCONV
  return true;
#else
  return false;
#endif
}

SparseTensor Convolve(const SparseTensor& input, const at::Tensor& weights,
                      const at::Tensor& bias, const CylinderCoordinate& stride,
                      const CylinderCoordinate& padding, bool submanifold,
                      SparseRelation* saved, SparseIndiceCache* cache) {
  Validate(input, weights);
  TORCH_CHECK(!input.use_spconv || input.features.is_cuda(),
              "Cylinder3D spconv requires CUDA features");
  TORCH_CHECK(!input.use_spconv || SpconvAvailable(),
              "Build Cylinder3D with --define=CYLINDER3D_SPCONV=true");
  Synchronize(input.features);
  const auto profile_begin = Clock::now();
  CylinderCoordinate kernel{}, shape{};
  for (int axis = 0; axis < 3; ++axis) {
    kernel[axis] = weights.size(axis);
    TORCH_CHECK(stride[axis] > 0 && padding[axis] >= 0 && padding[axis] <= 1 &&
                    (!submanifold || stride[axis] == 1),
                "invalid Cylinder3D convolution stride or padding");
    shape[axis] = submanifold
                      ? input.shape[axis]
                      : (input.shape[axis] + 2 * padding[axis] - kernel[axis]) /
                                stride[axis] +
                            1;
    TORCH_CHECK(shape[axis] > 0,
                "Cylinder3D convolution has empty spatial grid");
  }
  const auto kernel_count =
      static_cast<std::size_t>(kernel[0] * kernel[1] * kernel[2]);
  std::shared_ptr<const SparseRelation> cached;
  if (submanifold && cache != nullptr) {
    for (const auto& entry : *cache) {
      if (entry->input == input.coordinates &&
          entry->input_shape == input.shape && entry->kernel == kernel &&
          entry->stride == stride && entry->padding == padding &&
          entry->pair_counts.defined() == input.use_spconv &&
          entry->pairs.device() == input.features.device()) {
        cached = entry;
        break;
      }
    }
  }
  if (!cached) {
    auto built = std::make_shared<SparseRelation>();
    auto& relation = *built;
    relation.input = input.coordinates;
    relation.input_shape = input.shape;
    relation.output_shape = shape;
    relation.kernel = kernel;
    relation.stride = stride;
    relation.padding = padding;
    std::vector<std::vector<int64_t>> input_indices(kernel_count);
    std::vector<std::vector<int64_t>> output_indices(kernel_count);
    auto coordinates = std::make_shared<Coordinates>();
    CoordinateLookup lookup;
    auto indexed_input = input;
    bool validated = false;
    if (input.use_spconv && cache != nullptr) {
      for (const auto& entry : *cache) {
        if (entry->input == input.coordinates &&
            entry->input_shape == input.shape && entry->pair_counts.defined() &&
            entry->pairs.device() == input.features.device()) {
          // Only immutable coordinate owners validated in this forward qualify.
          validated = true;
          indexed_input.device_coordinates = entry->input_device_coordinates;
          break;
        }
      }
    }
    if (!validated) {
      lookup.Reset(input.shape, input.coordinates->size());
      for (std::size_t row = 0; row < input.coordinates->size(); ++row) {
        const auto& coordinate = (*input.coordinates)[row];
        TORCH_CHECK(
            InBounds(coordinate, input.shape) &&
                lookup.Insert(Key(coordinate, input.shape), row).second,
            "Cylinder3D sparse coordinates must be unique and in bounds");
      }
    }
    if (input.use_spconv && !input.coordinates->empty()) {
#ifdef CYLINDER3D_WITH_SPCONV
      TORCH_CHECK(
          !submanifold ||
              (kernel[0] % 2 == 1 && kernel[1] % 2 == 1 && kernel[2] % 2 == 1 &&
               padding == (CylinderCoordinate{kernel[0] / 2, kernel[1] / 2,
                                              kernel[2] / 2})),
          "Cylinder3D spconv requires centered odd submanifold kernels");
      SpconvPairs(indexed_input, submanifold, &relation);
#else
      TORCH_CHECK(false, "Cylinder3D spconv is unavailable in a CPU build");
#endif
    } else {
      if (submanifold) {
        relation.output = input.coordinates;
      } else {
        lookup.Reset(shape, input.coordinates->size());
        relation.output = coordinates;
      }
      for (int64_t k = 0; k < static_cast<int64_t>(kernel_count); ++k) {
        const CylinderCoordinate offset = {k / (kernel[1] * kernel[2]),
                                           (k / kernel[2]) % kernel[1],
                                           k % kernel[2]};
        for (std::size_t row = 0; row < input.coordinates->size(); ++row) {
          const auto& coordinate = (*input.coordinates)[row];
          CylinderCoordinate target{};
          bool divisible = true;
          for (int axis = 0; axis < 3; ++axis) {
            const auto value = coordinate[axis] + padding[axis] - offset[axis];
            if (submanifold) {
              target[axis] = value;
            } else {
              divisible = divisible && value % stride[axis] == 0;
              target[axis] = value / stride[axis];
            }
          }
          if (!divisible || !InBounds(target, shape)) {
            continue;
          }
          const auto key = Key(target, shape);
          int64_t target_row;
          if (submanifold) {
            target_row = lookup.Find(key);
            if (target_row < 0) {
              continue;
            }
          } else {
            auto inserted = lookup.Insert(key, coordinates->size());
            target_row = inserted.first;
            if (inserted.second) {
              coordinates->push_back(target);
            }
          }
          input_indices[k].push_back(row);
          output_indices[k].push_back(target_row);
        }
      }
      relation.offsets.push_back(0);
      for (const auto& indices : input_indices) {
        relation.offsets.push_back(relation.offsets.back() + indices.size());
      }
      auto pairs = at::empty({2, relation.offsets.back()},
                             at::TensorOptions().dtype(at::kLong));
      auto* from = pairs.data_ptr<int64_t>();
      auto* to = from + relation.offsets.back();
      for (std::size_t kernel_index = 0; kernel_index < kernel_count;
           ++kernel_index) {
        std::copy(input_indices[kernel_index].begin(),
                  input_indices[kernel_index].end(),
                  from + relation.offsets[kernel_index]);
        std::copy(output_indices[kernel_index].begin(),
                  output_indices[kernel_index].end(),
                  to + relation.offsets[kernel_index]);
      }
      relation.pairs = pairs.to(input.features.device());
    }
    cached = std::move(built);
    if (submanifold && cache != nullptr) {
      cache->push_back(cached);
    }
  }
  const auto& relation = *cached;
  const auto profile_pairs = Clock::now();
  auto features =
      ApplyPairs(input.features, weights, relation.output->size(), relation);
  if (bias.defined()) {
    TORCH_CHECK(bias.dim() == 1 && bias.size(0) == weights.size(4) &&
                    bias.device() == features.device() &&
                    bias.scalar_type() == at::kFloat,
                "invalid Cylinder3D convolution bias");
    features += bias;
  }
  auto result = SparseTensor{features, relation.output, shape, input.use_spconv,
                             relation.output_device_coordinates};
  if (Profiling()) {
    Synchronize(features);
    std::fprintf(
        stderr,
        "CYLINDER_PROFILE subm=%d rows=%zu channels=%ld,%ld "
        "kernel=%zu pairs_ms=%.3f apply_ms=%.3f\n",
        submanifold, input.coordinates->size(), weights.size(3),
        weights.size(4), kernel_count,
        std::chrono::duration<double, std::milli>(profile_pairs - profile_begin)
            .count(),
        std::chrono::duration<double, std::milli>(Clock::now() - profile_pairs)
            .count());
  }
  if (saved != nullptr) {
    *saved = relation;
  }
  return result;
}

SparseTensor InverseConvolve(const SparseTensor& input,
                             const at::Tensor& weights,
                             const SparseRelation& relation) {
  Validate(input, weights);
  Synchronize(input.features);
  const auto begin = Clock::now();
  TORCH_CHECK(
      input.coordinates == relation.output &&
          input.shape == relation.output_shape && relation.input &&
          weights.size(0) == relation.kernel[0] &&
          weights.size(1) == relation.kernel[1] &&
          weights.size(2) == relation.kernel[2],
      "Cylinder3D inverse convolution requires matching forward indices");
  auto result =
      SparseTensor{ApplyPairs(input.features, weights, relation.input->size(),
                              relation, true),
                   relation.input, relation.input_shape, input.use_spconv,
                   relation.input_device_coordinates};
  if (Profiling()) {
    Synchronize(result.features);
    std::fprintf(stderr, "CYLINDER_PROFILE inverse rows=%zu apply_ms=%.3f\n",
                 input.coordinates->size(),
                 std::chrono::duration<double, std::milli>(Clock::now() - begin)
                     .count());
  }
  return result;
}

}  // namespace cylinder
}  // namespace lidar_semantic_segmentation
}  // namespace apollo
