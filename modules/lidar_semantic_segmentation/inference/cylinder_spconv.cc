// Copyright 2026 WheelOS. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "modules/lidar_semantic_segmentation/inference/cylinder_spconv.h"

#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include <ATen/ATen.h>
#include <ATen/Context.h>
#include <c10/cuda/CUDAGuard.h>
#include <c10/cuda/CUDAStream.h>
#include <spconvlib/cumm/gemm/main/GemmMainUnitTest.h>
#include <spconvlib/spconv/csrc/sparse/all/SpconvOps.h>
#include <spconvlib/spconv/csrc/sparse/alloc/ExternalAllocator.h>
#include <spconvlib/spconv/csrc/sparse/convops/ExternalSpconvMatmul.h>
#include <spconvlib/spconv/csrc/sparse/convops/gemmops/GemmTunerSimple.h>
#include <spconvlib/spconv/csrc/sparse/convops/spops/ConvGemmOps.h>

namespace apollo {
namespace lidar_semantic_segmentation {
namespace cylinder {
namespace {

namespace sparse = spconvlib::spconv::csrc::sparse;
using Ops = sparse::all::SpconvOps;
using Conv = sparse::convops::spops::ConvGemmOps;
using Tuner = sparse::convops::gemmops::GemmTunerSimple;
using Gemm = spconvlib::cumm::gemm::main::GemmMainUnitTest;

tv::DType Type(at::ScalarType type) {
  switch (type) {
    case at::kFloat:
      return tv::float32;
    case at::kInt:
      return tv::int32;
    case at::kLong:
      return tv::int64;
    case at::kByte:
      return tv::uint8;
    default:
      TORCH_CHECK(false, "unsupported Cylinder3D spconv dtype");
  }
}

at::ScalarType Type(int type) {
  switch (type) {
    case tv::float32:
      return at::kFloat;
    case tv::int32:
      return at::kInt;
    case tv::int64:
      return at::kLong;
    case tv::uint8:
      return at::kByte;
    default:
      TORCH_CHECK(false, "unsupported Cylinder3D spconv allocation dtype");
  }
}

tv::Tensor View(const at::Tensor& tensor) {
  TORCH_CHECK(tensor.is_contiguous(), "spconv requires contiguous tensors");
  return tv::from_blob(tensor.data_ptr(), tensor.sizes().vec(),
                       Type(tensor.scalar_type()), tensor.is_cuda() ? 0 : -1);
}

std::uintptr_t Stream() {
  return reinterpret_cast<std::uintptr_t>(
      c10::cuda::getCurrentCUDAStream().stream());
}

// ATen owns all storage and reclaims it on its current CUDA stream. In
// particular, no fixed output bound is allowed to truncate active voxels.
class Allocator : public sparse::alloc::ExternalAllocator {
 public:
  tv::Tensor empty(std::string name, std::vector<int64_t> shape, int dtype,
                   int device, std::uintptr_t stream = 0,
                   bool temporary = false, float scale = 1.0F) override {
    auto options =
        at::TensorOptions()
            .dtype(Type(dtype))
            .device(device < 0
                        ? at::Device(at::kCPU)
                        : at::Device(at::kCUDA, c10::cuda::current_device()));
    return Keep(name, at::empty(shape, options), temporary);
  }

  tv::Tensor zeros(std::string name, std::vector<int64_t> shape, int dtype,
                   int device, std::uintptr_t stream = 0,
                   bool temporary = false, float scale = 1.0F) override {
    auto tensor = empty(name, shape, dtype, device, stream, temporary, scale);
    storage_.at(tensor.raw_data()).zero_();
    return tensor;
  }

  tv::Tensor full_int(std::string name, std::vector<int64_t> shape, int value,
                      int dtype, int device, std::uintptr_t stream = 0,
                      bool temporary = false) override {
    return Full(name, shape, value, dtype, device, stream, temporary);
  }

  tv::Tensor full_float(std::string name, std::vector<int64_t> shape,
                        float value, int dtype, int device,
                        std::uintptr_t stream = 0,
                        bool temporary = false) override {
    return Full(name, shape, value, dtype, device, stream, temporary);
  }

  tv::Tensor get_tensor_by_name(std::string name) override {
    return View(named.at(name));
  }

  void free(tv::Tensor tensor) override { storage_.erase(tensor.raw_data()); }
  void free_noexcept(tv::Tensor tensor) override {
    storage_.erase(tensor.raw_data());
  }

  tv::Tensor Keep(const std::string& name, at::Tensor tensor,
                  bool temporary = false) {
    auto view = View(tensor);
    storage_[view.raw_data()] = tensor;
    if (!temporary) {
      named[name] = std::move(tensor);
    }
    return view;
  }

  std::unordered_map<std::string, at::Tensor> named;

 private:
  tv::Tensor Full(const std::string& name, const std::vector<int64_t>& shape,
                  double value, int dtype, int device, std::uintptr_t stream,
                  bool temporary) {
    auto tensor = empty(name, shape, dtype, device, stream, temporary);
    storage_.at(tensor.raw_data()).fill_(value);
    return tensor;
  }

  std::unordered_map<void*, at::Tensor> storage_;
};

class CenterMatmul : public sparse::convops::ExternalSpconvMatmul {
 public:
  explicit CenterMatmul(Allocator* allocator) : allocator_(allocator) {}

  tv::Tensor indice_conv_init_gemm(std::string features, std::string filters,
                                   bool krsc, bool kc, int center, int channels,
                                   std::uintptr_t stream = 0) override {
    TORCH_CHECK(!krsc && !kc, "Cylinder3D expects canonical HWIO weights");
    const auto& weights = allocator_->named.at(filters);
    auto kernel = weights.reshape({-1, weights.size(-2), weights.size(-1)});
    return allocator_->Keep(
        SPCONV_ALLOC_OUT_FEATURES,
        at::matmul(allocator_->named.at(features), kernel[center]));
  }

 private:
  Allocator* allocator_;
};

std::vector<int> Dimensions(const CylinderCoordinate& values) {
  std::vector<int> result;
  for (auto value : values) {
    TORCH_CHECK(value >= 0 && value <= std::numeric_limits<int>::max(),
                "Cylinder3D spconv dimension exceeds int32");
    result.push_back(static_cast<int>(value));
  }
  return result;
}

}  // namespace

void SpconvPairs(const SparseTensor& input, bool submanifold,
                 SparseRelation* relation) {
  c10::cuda::CUDAGuard guard(input.features.device());
  TORCH_CHECK(input.coordinates->size() <=
                  static_cast<std::size_t>(std::numeric_limits<int>::max()),
              "Cylinder3D spconv point count exceeds int32");
  const auto count = static_cast<int64_t>(input.coordinates->size());
  auto resident = input.device_coordinates;
  if (!resident || resident->coordinates != input.coordinates ||
      resident->indices.device() != input.features.device()) {
    auto indices = at::zeros({count, 4}, at::TensorOptions().dtype(at::kInt));
    auto* rows = indices.data_ptr<int32_t>();
    for (int64_t row = 0; row < count; ++row) {
      for (int axis = 0; axis < 3; ++axis) {
        rows[row * 4 + axis + 1] = (*input.coordinates)[row][axis];
      }
    }
    resident = std::make_shared<DeviceCoordinates>(DeviceCoordinates{
        input.coordinates, indices.to(input.features.device())});
  }
  relation->input_device_coordinates = resident;
  Allocator allocator;
  const auto output_count = Ops::get_indice_pairs(
      allocator, View(resident->indices), 1, Dimensions(input.shape),
      static_cast<int>(tv::gemm::SparseConvAlgo::kNative),
      Dimensions(relation->kernel), Dimensions(relation->stride),
      Dimensions(relation->padding), {1, 1, 1}, {0, 0, 0}, submanifold, false,
      Stream());
  relation->pairs = allocator.named.at(SPCONV_ALLOC_PAIR_FWD);
  // The native API copies counts with tv::Tensor::cpu() on its default stream.
  // Copy on the ATen current stream here, then pass CPU counts to that API.
  relation->pair_counts =
      allocator.named.at(SPCONV_ALLOC_INDICE_NUM_PER_LOC).cpu();
  if (submanifold) {
    relation->output = input.coordinates;
    relation->output_device_coordinates = resident;
  } else {
    auto output_indices =
        allocator.named.at(SPCONV_ALLOC_OUT_INDICES).narrow(0, 0, output_count);
    auto output = output_indices.cpu();
    auto coordinates = std::make_shared<Coordinates>();
    coordinates->reserve(output_count);
    const auto* values = output.data_ptr<int32_t>();
    for (int row = 0; row < output_count; ++row) {
      coordinates->push_back(
          {values[row * 4 + 1], values[row * 4 + 2], values[row * 4 + 3]});
    }
    relation->output = coordinates;
    relation->output_device_coordinates = std::make_shared<DeviceCoordinates>(
        DeviceCoordinates{coordinates, output_indices});
  }
}

at::Tensor SpconvApply(const at::Tensor& features, const at::Tensor& weights,
                       const SparseRelation& relation, bool inverse) {
  c10::cuda::CUDAGuard guard(features.device());
  at::NoTF32Guard precision;
  static thread_local Tuner tuner(Gemm::get_all_algo_desp());
  static thread_local std::unordered_map<int, std::tuple<int, int>> devices;
  const int device = features.get_device();
  auto found = devices.find(device);
  if (found == devices.end()) {
    found = devices.emplace(device, Conv::get_compute_capability()).first;
  }
  Allocator allocator;
  auto contiguous = features.contiguous();
  auto filters = weights.contiguous();
  allocator.Keep(SPCONV_ALLOC_FEATURES, contiguous);
  allocator.Keep(SPCONV_ALLOC_FILTERS, filters);
  CenterMatmul matmul(&allocator);
  const auto count = static_cast<int>(inverse ? relation.input->size()
                                              : relation.output->size());
  Conv::indice_conv(
      allocator, matmul, tuner, false, true, View(contiguous), View(filters),
      View(relation.pairs), View(relation.pair_counts), found->second, count,
      inverse, !inverse && relation.input == relation.output,
      static_cast<int>(tv::gemm::SparseConvAlgo::kNative), Stream(),
      tv::Tensor(), 0.0F, 0.0F, tv::gemm::Activation::kNone, false);
  return allocator.named.at(SPCONV_ALLOC_OUT_FEATURES);
}

}  // namespace cylinder
}  // namespace lidar_semantic_segmentation
}  // namespace apollo
