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

#include <algorithm>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include <cuda_fp16.h>
#include <spconvlib/cumm/conv/main/ConvMainUnitTest.h>
#include <spconvlib/spconv/csrc/sparse/all/SpconvOps.h>
#include <spconvlib/spconv/csrc/sparse/alloc/StaticAllocator.h>
#include <spconvlib/spconv/csrc/sparse/convops/spops/ConvGemmOps.h>
#include <spconvlib/spconv/csrc/sparse/inference/InferenceOps.h>

#include "modules/lidar_object_detection/third_party/centerpoint/include/sparse_backbone.h"
#include "modules/lidar_object_detection/third_party/centerpoint/include/sparse_model.h"

namespace {

namespace convops = spconvlib::spconv::csrc::sparse::convops;

using Allocator = spconvlib::spconv::csrc::sparse::alloc::StaticAllocator;
using Ops = spconvlib::spconv::csrc::sparse::all::SpconvOps;
using Conv = convops::spops::ConvGemmOps;
using ConvMain = spconvlib::cumm::conv::main::ConvMainUnitTest;
using Tuner = convops::convops::ConvTunerSimple;
using Inference = spconvlib::spconv::csrc::sparse::inference::InferenceOps;

void Check(cudaError_t status) {
  if (status != cudaSuccess) {
    throw std::runtime_error(std::string("spconv CUDA: ") +
                             cudaGetErrorString(status));
  }
}

void Require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error("spconv backbone: " + message);
}

int64_t Volume(const std::vector<int64_t>& shape) {
  int64_t count = 1;
  for (auto value : shape) {
    Require(value > 0 && count <= std::numeric_limits<int64_t>::max() / value,
            "invalid or overflowing tensor shape");
    count *= value;
  }
  return count;
}

struct Value {
  tv::Tensor features;
  tv::Tensor indices;
  std::vector<int> grid;
  int count = 0;
  int channels = 0;
  bool dense = false;
};

struct Layer : SparseModelLayer {
  tv::Tensor weight, bias;
};

class BufferPool {
 public:
  tv::Tensor Get(const std::string& name, const std::vector<int64_t>& shape,
                 tv::DType dtype) {
    const int64_t count = Volume(shape);
    const size_t width = tv::bit_size(dtype) / 8;
    Require(width > 0 && static_cast<uint64_t>(count) <= kLimit / width,
            "tensor exceeds memory limit");
    auto& buffer = buffers_[name];
    const size_t old_bytes = buffer.empty() ? 0 : buffer.nbytes();
    if (buffer.empty() || buffer.size() < count || buffer.dtype() != dtype) {
      const size_t new_bytes = static_cast<size_t>(count) * width;
      Require(bytes_ - old_bytes <= kLimit - new_bytes,
              "retained workspace exceeds 1 GiB limit");
      buffer = tv::empty({count}, dtype, 0);
      bytes_ = bytes_ - old_bytes + new_bytes;
    }
    return buffer.slice_first_axis(0, count).view(tv::TensorShape(shape));
  }

 private:
  static constexpr size_t kLimit = 1024ULL * 1024 * 1024;
  size_t bytes_ = 0;
  std::unordered_map<std::string, tv::Tensor> buffers_;
};

__global__ void Add(const __half* a, const __half* b, __half* output,
                    int size) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < size) output[i] = __hadd(a[i], b[i]);
}

__global__ void Scatter(const __half* features, const int* coordinates,
                        __half* output, int count, int channels, int z_size,
                        int y_size, int x_size, bool xyz) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= count) return;
  const int x = coordinates[i * 4 + (xyz ? 1 : 3)];
  const int y = coordinates[i * 4 + 2];
  const int z = coordinates[i * 4 + (xyz ? 3 : 1)];
  for (int c = 0; c < channels; ++c) {
    const int64_t offset =
        ((static_cast<int64_t>(c) * z_size + z) * y_size + y) * x_size + x;
    output[offset] = features[static_cast<int64_t>(i) * channels + c];
  }
}

class SpconvBackbone final : public SparseBackbone {
 public:
  explicit SpconvBackbone(const std::string& path)
      : tuner_(ConvMain::get_all_conv_algo_desp()) {
    Check(cudaGetDevice(&device_));
    arch_ = Conv::get_compute_capability();
    auto model = LoadSparseModel(path);
    input_ = model.input;
    output_name_ = model.output;
    output_shape_ = std::move(model.output_shape);
    for (auto& definition : model.layers) {
      Layer layer;
      static_cast<SparseModelLayer&>(layer) = std::move(definition);
      if (layer.op == "SparseConvolution") {
        layer.weight =
            tv::empty(tv::TensorShape(layer.weight_shape), tv::float16, 0);
        layer.bias = tv::empty({static_cast<int64_t>(layer.bias_data.size())},
                               tv::float16, 0);
        Check(cudaMemcpy(layer.weight.raw_data(), layer.weight_data.data(),
                         layer.weight.nbytes(), cudaMemcpyHostToDevice));
        Check(cudaMemcpy(layer.bias.raw_data(), layer.bias_data.data(),
                         layer.bias.nbytes(), cudaMemcpyHostToDevice));
        layer.weight_data.clear();
        layer.bias_data.clear();
      }
      layers_.push_back(std::move(layer));
    }
    allocators_.resize(layers_.size());
    mask_ = tv::zeros({1}, tv::uint32, -1);
    mask_.data_ptr<uint32_t>()[0] = 0xffffffff;
  }

  void* Forward(void* features, void* indices, unsigned int count,
                const std::vector<int>& grid, cudaStream_t stream) override {
    std::lock_guard<std::mutex> lock(mutex_);
    int device = -1;
    Check(cudaGetDevice(&device));
    Require(device == device_, "instance used on a different CUDA device");
    Require(features && indices && count > 0 && count <= 160000,
            "invalid device input or active voxel count");
    Require(grid == layers_.front().input_grid,
            "input grid does not match model");
    tv::Context context;
    context.set_cuda_stream_int(reinterpret_cast<std::uintptr_t>(stream));
    std::unordered_map<std::string, Value> values;
    values.emplace(
        input_, Value{tv::from_blob(features, {static_cast<int64_t>(count), 5},
                                    tv::float16, 0),
                      tv::from_blob(indices, {static_cast<int64_t>(count), 4},
                                    tv::int32, 0),
                      grid, static_cast<int>(count), 5, false});
    std::unordered_map<std::string, Pairs> pairs;
    // Keep every input/output alive until this stream completes, including
    // errors.
    try {
      for (size_t i = 0; i < layers_.size(); ++i) {
        const auto& layer = layers_[i];
        const auto& input = values.at(layer.inputs[0]);
        Value output;
        if (layer.op == "SparseConvolution") {
          output = Convolve(i, layer, input, &pairs, stream, context);
        } else if (layer.op == "Add") {
          const auto& rhs = values.at(layer.inputs[1]);
          Require(!input.dense && !rhs.dense && input.count == rhs.count &&
                      input.channels == rhs.channels &&
                      input.grid == rhs.grid &&
                      input.indices.raw_data() == rhs.indices.raw_data(),
                  "Add requires the same sparse coordinate ordering");
          output = input;
          output.features =
              pool_.Get(std::to_string(i) + ":features",
                        {input.count, input.channels}, tv::float16);
          const int size = input.count * input.channels;
          Add<<<(size + 255) / 256, 256, 0, stream>>>(
              input.features.data_ptr<__half>(),
              rhs.features.data_ptr<__half>(),
              output.features.data_ptr<__half>(), size);
          Check(cudaGetLastError());
        } else if (layer.op == "Relu") {
          output = input;
          output.features =
              pool_.Get(std::to_string(i) + ":features",
                        {input.count, input.channels}, tv::float16);
          Check(cudaMemcpyAsync(
              output.features.raw_data(), input.features.raw_data(),
              input.features.nbytes(), cudaMemcpyDeviceToDevice, stream));
          Inference::activation_inplace(
              output.features, tv::gemm::Activation::kReLU, 0, 0,
              reinterpret_cast<std::uintptr_t>(stream));
        } else if (layer.op == "ScatterDense") {
          Require(!input.dense && input.grid == layer.input_grid &&
                      input.channels == layer.dense_shape[1],
                  "ScatterDense input layout mismatch");
          std::vector<int64_t> shape(layer.dense_shape.begin(),
                                     layer.dense_shape.end());
          output.features = pool_.Get("dense", shape, tv::float16);
          output.features.zero_(context);
          const int z_size = layer.xyz ? input.grid[2] : input.grid[0];
          const int y_size = input.grid[1];
          const int x_size = layer.xyz ? input.grid[0] : input.grid[2];
          Scatter<<<(input.count + 127) / 128, 128, 0, stream>>>(
              input.features.data_ptr<__half>(), input.indices.data_ptr<int>(),
              output.features.data_ptr<__half>(), input.count, input.channels,
              z_size, y_size, x_size, layer.xyz);
          Check(cudaGetLastError());
          output.dense = true;
        } else if (layer.op == "Reshape") {
          Require(input.dense, "only dense Reshape is supported");
          output = input;
          std::vector<int64_t> shape(layer.dense_shape.begin(),
                                     layer.dense_shape.end());
          Require(Volume(shape) == input.features.size(),
                  "Reshape volume mismatch");
          output.features = input.features.view(tv::TensorShape(shape));
        }
        values.emplace(layer.output, std::move(output));
      }
      output_ = values.at(output_name_).features;
      Check(cudaStreamSynchronize(stream));
      return output_.raw_data();
    } catch (...) {
      // Preserve the original error, but surface any failure to drain the
      // stream.
      const auto status = cudaStreamSynchronize(stream);
      if (status != cudaSuccess) {
        throw std::runtime_error(
            std::string("spconv failed and stream drain failed: ") +
            cudaGetErrorString(status));
      }
      throw;
    }
  }

  std::vector<int64_t> Shape() const override { return output_shape_; }

 private:
  struct Pairs {
    tv::Tensor owner, forward, mask, argsort, indices;
    int count = 0;
  };

  Value Convolve(size_t index, const Layer& layer, const Value& input,
                 std::unordered_map<std::string, Pairs>* cache,
                 cudaStream_t stream, tv::Context context) {
    Require(!input.dense && input.grid == layer.input_grid,
            "invalid sparse input");
    const auto stream_id = reinterpret_cast<std::uintptr_t>(stream);
    const std::string prefix = std::to_string(index) + ":";
    auto allocate = [&](const std::string& role,
                        const std::vector<int64_t>& shape, tv::DType dtype) {
      return pool_.Get(prefix + role, shape, dtype);
    };
    std::ostringstream key;
    key << static_cast<const void*>(input.indices.raw_data());
    for (const auto* dimensions :
         {&layer.input_grid, &layer.kernel, &layer.stride, &layer.padding,
          &layer.dilation}) {
      for (auto value : *dimensions) key << ':' << value;
    }
    auto found = layer.subm ? cache->find(key.str()) : cache->end();
    Pairs pairs;
    if (found != cache->end()) {
      pairs = found->second;
    } else {
      const int kv = layer.kernel[0] * layer.kernel[1] * layer.kernel[2];
      const int bound = layer.subm ? input.count : layer.bound;
      const int max_theory = Ops::get_handcrafted_max_act_out(
          input.count, layer.kernel, layer.stride, layer.padding,
          layer.dilation);
      const int64_t volume = Volume(std::vector<int64_t>(
          layer.output_grid.begin(), layer.output_grid.end()));
      const bool wide = volume >= std::numeric_limits<int>::max();
      const bool direct = !layer.subm;
      const int workspace_size = Ops::get_indice_gen_workspace_size(
          kv, input.count, bound, max_theory, layer.subm, wide, direct);
      auto workspace = allocate("workspace", {workspace_size}, tv::uint8);
      auto tensors = Ops::get_indice_gen_tensors_from_workspace(
          static_cast<uint8_t*>(workspace.raw_data()), kv, input.count, bound,
          max_theory, layer.subm, wide, direct);
      pairs.owner = input.indices;
      pairs.forward = allocate("pairs", {kv, bound}, tv::int32);
      pairs.mask = allocate("mask", {1, bound}, tv::int32);
      pairs.argsort = allocate("argsort", {1, bound}, tv::int32);
      pairs.indices = allocate("indices", {bound, 4}, tv::int32);
      auto counts = allocate("counts", {kv}, tv::int32);
      tensors[SPCONV_ALLOC_PAIR_FWD] = pairs.forward;
      tensors[SPCONV_ALLOC_PAIR_MASK] = pairs.mask;
      tensors[SPCONV_ALLOC_MASK_ARG_SORT] = pairs.argsort;
      tensors[SPCONV_ALLOC_OUT_INDICES] = pairs.indices;
      tensors[SPCONV_ALLOC_INDICE_NUM_PER_LOC] = counts;
      if (!layer.subm) {
        tensors[SPCONV_ALLOC_PAIR_BWD] =
            allocate("pairs_bwd", {kv, input.count}, tv::int32);
        tensors[SPCONV_ALLOC_PAIR_MASK_BWD] =
            allocate("mask_bwd", {1, input.count}, tv::int32);
        tensors[SPCONV_ALLOC_MASK_ARG_SORT_BWD] =
            allocate("argsort_bwd", {1, input.count}, tv::int32);
      } else {
        pairs.indices.copy_(input.indices, context);
      }
      auto& allocator = allocators_[index];
      if (!allocator)
        allocator = std::make_unique<Allocator>(tensors);
      else
        allocator->set_new_tensor_dict(tensors);
      auto result = Ops::get_indice_pairs_implicit_gemm(
          *allocator, input.indices, 1, layer.input_grid,
          static_cast<int>(tv::gemm::SparseConvAlgo::kMaskImplicitGemm),
          layer.kernel, layer.stride, layer.padding, layer.dilation, {0, 0, 0},
          layer.subm, false, false, stream_id, bound,
          tv::CUDAKernelTimer(false), direct);
      pairs.count = std::get<1>(result);
      Require(pairs.count > 0 && pairs.count <= bound &&
                  (layer.subm || pairs.count < bound),
              "empty output or active output bound reached");
      if (layer.subm) cache->emplace(key.str(), pairs);
    }
    const int kv = layer.kernel[0] * layer.kernel[1] * layer.kernel[2];
    const int channels = layer.weight.dim(0);
    auto features = allocate("features", {pairs.count, channels}, tv::float16);
    Allocator allocator({{SPCONV_ALLOC_FEATURES, input.features},
                         {SPCONV_ALLOC_FILTERS, layer.weight},
                         {SPCONV_ALLOC_OUT_FEATURES, features}});
    Conv::implicit_gemm(
        allocator, tuner_, input.features, layer.weight,
        pairs.forward.view(-1)
            .slice_first_axis(0, kv * pairs.count)
            .view(kv, pairs.count),
        {pairs.mask.view(-1).slice_first_axis(0, pairs.count)},
        {pairs.argsort.view(-1).slice_first_axis(0, pairs.count)}, pairs.count,
        mask_, arch_, false, layer.subm, stream_id, tv::CUDAKernelTimer(false),
        false, false, layer.bias, 1.0, 0.0,
        layer.relu ? tv::gemm::Activation::kReLU : tv::gemm::Activation::kNone);
    return Value{features,
                 layer.subm ? input.indices
                            : pairs.indices.slice_first_axis(0, pairs.count),
                 layer.output_grid,
                 pairs.count,
                 channels,
                 false};
  }

  int device_ = -1;
  std::tuple<int, int> arch_;
  std::string input_, output_name_;
  std::vector<int64_t> output_shape_;
  std::vector<Layer> layers_;
  Tuner tuner_;
  BufferPool pool_;
  std::vector<std::unique_ptr<Allocator>> allocators_;
  tv::Tensor mask_, output_;
  std::mutex mutex_;
};

}  // namespace

std::unique_ptr<SparseBackbone> LoadSparseBackbone(const std::string& path) {
  return std::make_unique<SpconvBackbone>(path);
}
