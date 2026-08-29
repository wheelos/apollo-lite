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

//  Created Date: 2026-08-28
//  Author: daohu527

#include "modules/lane/inference/tensorrt_executor.h"

#include <NvInfer.h>
#include <NvInferPlugin.h>
#include <NvInferVersion.h>

#include <array>
#include <fstream>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <cuda_runtime_api.h>

#include "cyber/common/log.h"

namespace apollo {
namespace lane {

namespace {

class TensorRtLogger final : public nvinfer1::ILogger {
 public:
  void log(Severity severity, const char* message) noexcept override {
    if (severity <= Severity::kERROR) {
      AERROR << message;
    } else if (severity == Severity::kWARNING) {
      AWARN << message;
    }
  }
};

TensorRtLogger g_logger;

bool CudaSucceeded(cudaError_t error, const char* operation) {
  if (error == cudaSuccess) {
    return true;
  }
  AERROR << operation << " failed: " << cudaGetErrorString(error);
  return false;
}

bool HasDimensions(const nvinfer1::Dims& dimensions,
                   const std::array<int, 4>& expected) {
  if (dimensions.nbDims != static_cast<int>(expected.size())) {
    return false;
  }
  for (int index = 0; index < dimensions.nbDims; ++index) {
    if (dimensions.d[index] != expected[static_cast<size_t>(index)]) {
      return false;
    }
  }
  return true;
}

nvinfer1::Dims InputDimensions() {
  nvinfer1::Dims dimensions;
  dimensions.nbDims = 4;
  dimensions.d[0] = 1;
  dimensions.d[1] = 3;
  dimensions.d[2] = kUfldv2ModelHeight;
  dimensions.d[3] = kUfldv2ModelWidth;
  return dimensions;
}

template <typename TensorRtType>
void DestroyTensorRt(TensorRtType* object) {
  if (object == nullptr) {
    return;
  }
#if NV_TENSORRT_MAJOR >= 10
  delete object;
#else
  object->destroy();
#endif
}

}  // namespace

class TensorRtUfldv2Executor::Impl {
 public:
  struct Buffer {
    std::string name;
    std::array<int, 4> dimensions = {0, 0, 0, 0};
    void* device_memory = nullptr;
#if NV_TENSORRT_MAJOR < 10
    int binding_index = -1;
#endif

    size_t ByteCount() const {
      size_t count = 1U;
      for (const int dimension : dimensions) {
        count *= static_cast<size_t>(dimension);
      }
      return count * sizeof(float);
    }
  };

  ~Impl() {
    if (initialized_) {
      cudaSetDevice(device_id_);
    }
    FreeBuffers();
    if (stream_ != nullptr) {
      cudaStreamDestroy(stream_);
    }
    DestroyTensorRt(context_);
    DestroyTensorRt(engine_);
    DestroyTensorRt(runtime_);
  }

  bool Init(const TensorRtEngineOptions& options) {
    if (initialized_ || options.device_id < 0 || options.engine_path.empty() ||
        !NamesAreDistinct(options.tensor_names) ||
        !CudaSucceeded(cudaSetDevice(options.device_id), "cudaSetDevice")) {
      return false;
    }
    device_id_ = options.device_id;
    std::ifstream file(options.engine_path, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
      AERROR << "Unable to open TensorRT engine: " << options.engine_path;
      return false;
    }
    const std::streamsize file_size = file.tellg();
    if (file_size <= 0) {
      AERROR << "TensorRT engine is empty: " << options.engine_path;
      return false;
    }
    std::vector<char> engine_bytes(static_cast<size_t>(file_size));
    file.seekg(0, std::ios::beg);
    if (!file.read(engine_bytes.data(), file_size)) {
      AERROR << "Unable to read TensorRT engine: " << options.engine_path;
      return false;
    }
    initLibNvInferPlugins(&g_logger, "");
    runtime_ = nvinfer1::createInferRuntime(g_logger);
    if (runtime_ == nullptr) {
      AERROR << "Unable to create TensorRT runtime";
      return false;
    }
#if NV_TENSORRT_MAJOR >= 10
    engine_ = runtime_->deserializeCudaEngine(engine_bytes.data(),
                                              engine_bytes.size());
#else
    engine_ = runtime_->deserializeCudaEngine(engine_bytes.data(),
                                              engine_bytes.size(), nullptr);
#endif
    if (engine_ == nullptr || !ConfigureBuffers(options.tensor_names)) {
      AERROR << "TensorRT engine does not match the UFLDv2 I/O contract";
      return false;
    }
    context_ = engine_->createExecutionContext();
    if (context_ == nullptr ||
        !CudaSucceeded(cudaStreamCreate(&stream_), "cudaStreamCreate") ||
        !AllocateBuffers()) {
      return false;
    }
    initialized_ = true;
    return true;
  }

  bool Run(const std::vector<float>& input, Ufldv2TensorOutputs* outputs) {
    if (!initialized_ || outputs == nullptr ||
        input.size() != kUfldv2InputElementCount ||
        !CudaSucceeded(cudaSetDevice(device_id_), "cudaSetDevice") ||
        !CudaSucceeded(cudaMemcpyAsync(input_.device_memory, input.data(),
                                       input_.ByteCount(),
                                       cudaMemcpyHostToDevice, stream_),
                       "cudaMemcpyAsync input")) {
      return false;
    }
#if NV_TENSORRT_MAJOR >= 10
    if (!context_->setInputShape(input_.name.c_str(), InputDimensions()) ||
        !SetTensorAddresses() || !context_->enqueueV3(stream_)) {
      AERROR << "TensorRT enqueueV3 failed";
      return false;
    }
#else
    if (!context_->setBindingDimensions(input_binding_index_,
                                        InputDimensions()) ||
        !context_->allInputDimensionsSpecified() ||
        !context_->enqueueV2(bindings_.data(), stream_, nullptr)) {
      AERROR << "TensorRT enqueueV2 failed";
      return false;
    }
#endif
    return CopyOutputs(outputs);
  }

 private:
  bool NamesAreDistinct(const Ufldv2TensorNames& names) const {
    const std::set<std::string> unique_names = {names.input, names.loc_row,
                                                names.loc_col, names.exist_row,
                                                names.exist_col};
    return unique_names.size() == 5U &&
           unique_names.find("") == unique_names.end();
  }

  bool ConfigureBuffer(const std::string& name,
                       const std::array<int, 4>& dimensions, Buffer* buffer,
                       bool expect_input) {
    if (buffer == nullptr) {
      return false;
    }
#if NV_TENSORRT_MAJOR >= 10
    bool found = false;
    for (int index = 0; index < engine_->getNbIOTensors(); ++index) {
      const char* tensor_name = engine_->getIOTensorName(index);
      if (tensor_name != nullptr && name == tensor_name) {
        found = true;
        const nvinfer1::TensorIOMode mode =
            engine_->getTensorIOMode(name.c_str());
        if ((expect_input && mode != nvinfer1::TensorIOMode::kINPUT) ||
            (!expect_input && mode != nvinfer1::TensorIOMode::kOUTPUT) ||
            engine_->getTensorDataType(name.c_str()) !=
                nvinfer1::DataType::kFLOAT ||
            !HasDimensions(engine_->getTensorShape(name.c_str()), dimensions)) {
          return false;
        }
        break;
      }
    }
    if (!found) {
      return false;
    }
#else
    const int binding_index = engine_->getBindingIndex(name.c_str());
    if (binding_index < 0 ||
        engine_->bindingIsInput(binding_index) != expect_input ||
        engine_->getBindingDataType(binding_index) !=
            nvinfer1::DataType::kFLOAT ||
        !HasDimensions(engine_->getBindingDimensions(binding_index),
                       dimensions)) {
      return false;
    }
    if (expect_input) {
      input_binding_index_ = binding_index;
    }
    buffer->binding_index = binding_index;
#endif
    buffer->name = name;
    buffer->dimensions = dimensions;
    return true;
  }

  bool ConfigureBuffers(const Ufldv2TensorNames& names) {
#if NV_TENSORRT_MAJOR >= 10
    if (engine_->getNbIOTensors() != 5) {
      return false;
    }
#else
    if (engine_->getNbBindings() != 5) {
      return false;
    }
    bindings_.assign(5U, nullptr);
#endif
    return ConfigureBuffer(names.input, {1, 3, 320, 1600}, &input_, true) &&
           ConfigureBuffer(names.loc_row, {1, 200, 72, 4}, &loc_row_, false) &&
           ConfigureBuffer(names.loc_col, {1, 100, 81, 4}, &loc_col_, false) &&
           ConfigureBuffer(names.exist_row, {1, 2, 72, 4}, &exist_row_,
                           false) &&
           ConfigureBuffer(names.exist_col, {1, 2, 81, 4}, &exist_col_, false);
  }

  bool AllocateBuffer(Buffer* buffer) {
    if (buffer == nullptr ||
        !CudaSucceeded(cudaMalloc(&buffer->device_memory, buffer->ByteCount()),
                       "cudaMalloc")) {
      return false;
    }
#if NV_TENSORRT_MAJOR < 10
    bindings_[static_cast<size_t>(buffer->binding_index)] =
        buffer->device_memory;
#endif
    return true;
  }

  bool AllocateBuffers() {
    return AllocateBuffer(&input_) && AllocateBuffer(&loc_row_) &&
           AllocateBuffer(&loc_col_) && AllocateBuffer(&exist_row_) &&
           AllocateBuffer(&exist_col_);
  }

  void FreeBuffers() {
    for (Buffer* buffer :
         {&input_, &loc_row_, &loc_col_, &exist_row_, &exist_col_}) {
      if (buffer->device_memory != nullptr) {
        cudaFree(buffer->device_memory);
        buffer->device_memory = nullptr;
      }
    }
  }

#if NV_TENSORRT_MAJOR >= 10
  bool SetTensorAddresses() {
    for (const Buffer* buffer :
         {&input_, &loc_row_, &loc_col_, &exist_row_, &exist_col_}) {
      if (!context_->setTensorAddress(buffer->name.c_str(),
                                      buffer->device_memory)) {
        return false;
      }
    }
    return true;
  }
#endif

  bool CopyBuffer(const Buffer& buffer, Ufldv2Tensor* output) {
    if (output == nullptr) {
      return false;
    }
    output->shape.assign(buffer.dimensions.begin(), buffer.dimensions.end());
    output->values.resize(buffer.ByteCount() / sizeof(float));
    return CudaSucceeded(
        cudaMemcpyAsync(output->values.data(), buffer.device_memory,
                        buffer.ByteCount(), cudaMemcpyDeviceToHost, stream_),
        "cudaMemcpyAsync output");
  }

  bool CopyOutputs(Ufldv2TensorOutputs* outputs) {
    if (!CopyBuffer(loc_row_, &outputs->loc_row) ||
        !CopyBuffer(loc_col_, &outputs->loc_col) ||
        !CopyBuffer(exist_row_, &outputs->exist_row) ||
        !CopyBuffer(exist_col_, &outputs->exist_col)) {
      return false;
    }
    return CudaSucceeded(cudaStreamSynchronize(stream_),
                         "cudaStreamSynchronize");
  }

  nvinfer1::IRuntime* runtime_ = nullptr;
  nvinfer1::ICudaEngine* engine_ = nullptr;
  nvinfer1::IExecutionContext* context_ = nullptr;
  cudaStream_t stream_ = nullptr;
  Buffer input_;
  Buffer loc_row_;
  Buffer loc_col_;
  Buffer exist_row_;
  Buffer exist_col_;
#if NV_TENSORRT_MAJOR < 10
  int input_binding_index_ = -1;
  std::vector<void*> bindings_;
#endif
  bool initialized_ = false;
  int device_id_ = 0;
};

TensorRtUfldv2Executor::TensorRtUfldv2Executor() : impl_(new Impl) {}

TensorRtUfldv2Executor::~TensorRtUfldv2Executor() = default;

bool TensorRtUfldv2Executor::Init(const TensorRtEngineOptions& options) {
  return impl_->Init(options);
}

bool TensorRtUfldv2Executor::Run(const std::vector<float>& input,
                                 Ufldv2TensorOutputs* outputs) {
  return impl_->Run(input, outputs);
}

}  // namespace lane
}  // namespace apollo
