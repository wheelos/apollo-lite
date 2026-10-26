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

#include <sys/stat.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "sparse_model.h"
#include "spconv/sparse_backbone.h"

namespace {

void Check(cudaError_t status) {
  if (status != cudaSuccess) {
    throw std::runtime_error(cudaGetErrorString(status));
  }
}

struct FreeDevice {
  void operator()(void* pointer) const {
    const auto status = cudaFree(pointer);
    if (status != cudaSuccess) {
      std::cerr << "cudaFree: " << cudaGetErrorString(status) << '\n';
      std::terminate();
    }
  }
};

using DeviceBuffer = std::unique_ptr<void, FreeDevice>;

DeviceBuffer Allocate(size_t bytes) {
  void* pointer = nullptr;
  Check(cudaMalloc(&pointer, bytes));
  return DeviceBuffer(pointer);
}

struct Stream {
  Stream() { Check(cudaStreamCreateWithFlags(&value, cudaStreamNonBlocking)); }
  ~Stream() {
    const auto status = cudaStreamDestroy(value);
    if (status != cudaSuccess) {
      std::cerr << "cudaStreamDestroy: " << cudaGetErrorString(status) << '\n';
      std::terminate();
    }
  }
  cudaStream_t value = nullptr;
};

void Read(std::ifstream& input, void* destination, size_t size) {
  if (!input.read(static_cast<char*>(destination), size)) {
    throw std::runtime_error("truncated fixture");
  }
}

int Run(const std::string& model, const std::string& list,
        const std::string& directory, int passes) {
  std::ifstream paths(list);
  if (!paths) throw std::runtime_error("cannot read fixture list: " + list);
  std::vector<std::string> fixtures;
  for (std::string path; std::getline(paths, path);) {
    if (!path.empty()) fixtures.push_back(path);
  }
  if (fixtures.size() < 2 || passes < 2 || passes > 10) {
    throw std::runtime_error("require at least two fixtures and 2..10 passes");
  }
  if (mkdir(directory.c_str(), 0755) != 0) {
    throw std::runtime_error("cannot create new output directory: " +
                             std::string(std::strerror(errno)));
  }
  std::ofstream csv(directory + "/timings.csv");
  csv.exceptions(std::ios::failbit | std::ios::badbit);
  csv << "pass,fixture,count,forward_wall_ms,hash\n";
  auto backbone = wheelos::perception::ops::spconv::CreateSparseBackbone(
      LoadSparseModel(model));
  auto features_device = Allocate(160000 * 5 * sizeof(uint16_t));
  auto indices_device = Allocate(160000 * 4 * sizeof(int32_t));
  std::array<Stream, 2> streams;
  std::vector<uint16_t> output(256 * 180 * 180);
  std::vector<uint16_t> expected(output.size());
  // Revisit the first frame after changing input shapes to expose stale caches.
  const size_t sequence_size = fixtures.size() + 1;
  for (int pass = 0; pass < passes; ++pass) {
    for (size_t position = 0; position < sequence_size; ++position) {
      const size_t index = position % fixtures.size();
      std::ifstream input(fixtures[index], std::ios::binary);
      if (!input)
        throw std::runtime_error("cannot read fixture: " + fixtures[index]);
      std::array<uint32_t, 4> header{};
      Read(input, header.data(), sizeof(header));
      const size_t count = header[0];
      if (count == 0 || count > 160000 || header[1] != 41 ||
          header[2] != 1440 || header[3] != 1440) {
        throw std::runtime_error("invalid CenterPoint fixture header");
      }
      std::vector<uint16_t> features(count * 5);
      std::vector<int32_t> indices(count * 4);
      Read(input, features.data(), features.size() * sizeof(uint16_t));
      Read(input, indices.data(), indices.size() * sizeof(int32_t));
      if (input.peek() != std::ifstream::traits_type::eof()) {
        throw std::runtime_error("fixture has trailing bytes");
      }
      for (size_t i = 0; i < count; ++i) {
        if (indices[i * 4] != 0)
          throw std::runtime_error("invalid batch index");
        for (size_t j = 1; j < 4; ++j) {
          if (indices[i * 4 + j] < 0 ||
              static_cast<uint32_t>(indices[i * 4 + j]) >= header[j]) {
            throw std::runtime_error("invalid voxel coordinate");
          }
        }
      }
      auto stream = streams[(pass * sequence_size + position) % 2].value;
      Check(cudaMemcpyAsync(features_device.get(), features.data(),
                            features.size() * sizeof(uint16_t),
                            cudaMemcpyHostToDevice, stream));
      Check(cudaMemcpyAsync(indices_device.get(), indices.data(),
                            indices.size() * sizeof(int32_t),
                            cudaMemcpyHostToDevice, stream));
      Check(cudaStreamSynchronize(stream));
      const auto start = std::chrono::steady_clock::now();
      void* dense =
          backbone->Forward(features_device.get(), indices_device.get(), count,
                            {41, 1440, 1440}, stream);
      Check(cudaStreamSynchronize(stream));
      const double elapsed = std::chrono::duration<double, std::milli>(
                                 std::chrono::steady_clock::now() - start)
                                 .count();
      Check(cudaMemcpyAsync(output.data(), dense,
                            output.size() * sizeof(uint16_t),
                            cudaMemcpyDeviceToHost, stream));
      Check(cudaStreamSynchronize(stream));
      uint64_t hash = 14695981039346656037ULL;
      for (uint16_t value : output) {
        if ((value & 0x7c00) == 0x7c00) {
          throw std::runtime_error("nonfinite dense output");
        }
        hash = (hash ^ (value & 0xff)) * 1099511628211ULL;
        hash = (hash ^ (value >> 8)) * 1099511628211ULL;
      }
      if (pass > 0 || position == fixtures.size()) {
        std::ifstream reference(
            directory + "/" + std::to_string(index) + ".fp16",
            std::ios::binary);
        Read(reference, expected.data(), expected.size() * sizeof(uint16_t));
        if (expected != output) {
          throw std::runtime_error("repeated fixture changed output: " +
                                   std::to_string(index));
        }
      }
      if (pass == 0 && position < fixtures.size()) {
        std::ofstream binary(directory + "/" + std::to_string(index) + ".fp16",
                             std::ios::binary);
        binary.exceptions(std::ios::failbit | std::ios::badbit);
        binary.write(reinterpret_cast<const char*>(output.data()),
                     output.size() * sizeof(uint16_t));
      }
      csv << pass << ',' << index << ',' << count << ',' << elapsed << ','
          << hash << '\n';
      csv.flush();
    }
  }
  std::cout << "Validated " << passes * sequence_size
            << " complete sparse+scatter forwards on two nondefault streams.\n";
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc != 5) {
      throw std::runtime_error(
          "usage: sparse_backbone_benchmark MODEL FIXTURE_LIST NEW_OUTPUT_DIR "
          "PASSES");
    }
    const std::string passes(argv[4]);
    if (passes.size() != 1 || passes[0] < '2' || passes[0] > '9') {
      throw std::runtime_error("PASSES must be 2..9");
    }
    return Run(argv[1], argv[2], argv[3], passes[0] - '0');
  } catch (const std::exception& error) {
    std::cerr << "sparse backbone validation failed: " << error.what() << '\n';
    return 1;
  }
}
