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

#include <NvInfer.h>
#include <NvInferPlugin.h>
#include <NvInferVersion.h>

#include <array>
#include <fstream>
#include <initializer_list>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include <cuda_runtime_api.h>

#include "cyber/common/log.h"
#include "modules/prediction/evaluator/vehicle/hivt_tensorrt_executor.h"

namespace apollo {
namespace prediction {

namespace {

constexpr int kHiVTMaxActorEdges = kHiVTMaxActors * (kHiVTMaxActors - 1);
constexpr int kHiVTMaxLaneActorEdges = kHiVTMaxActors * kHiVTMaxLaneVectors;

class HiVTRuntimeLogger final : public nvinfer1::ILogger {
 public:
  void log(Severity severity, const char* message) noexcept override {
    if (severity <= Severity::kERROR) {
      AERROR << message;
    } else if (severity == Severity::kWARNING) {
      AWARN << message;
    }
  }
};

HiVTRuntimeLogger g_logger;

template <typename TensorRtObject>
void DestroyTensorRt(TensorRtObject* object) {
  if (object == nullptr) {
    return;
  }
#if NV_TENSORRT_MAJOR >= 10
  delete object;
#else
  object->destroy();
#endif
}

bool CudaSucceeded(cudaError_t status, const char* operation) {
  if (status == cudaSuccess) {
    return true;
  }
  AERROR << operation << " failed: " << cudaGetErrorString(status);
  return false;
}

nvinfer1::Dims MakeDims(std::initializer_list<int> dimensions) {
  nvinfer1::Dims result;
  result.nbDims = static_cast<int>(dimensions.size());
  int index = 0;
  for (const int dimension : dimensions) {
    result.d[index++] = dimension;
  }
  return result;
}

std::size_t ElementCount(const nvinfer1::Dims& dimensions) {
  return std::accumulate(dimensions.d, dimensions.d + dimensions.nbDims,
                         std::size_t{1}, [](std::size_t count, int dimension) {
                           return count * static_cast<std::size_t>(dimension);
                         });
}

}  // namespace

class HiVTTensorRtExecutor::Impl {
 public:
  struct Buffer {
    const char* name;
    nvinfer1::DataType type;
    std::size_t capacity_bytes;
    void* device_memory = nullptr;
  };

  ~Impl() {
    if (initialized_) {
      cudaSetDevice(device_id_);
    }
    for (auto& buffer : buffers_) {
      if (buffer.device_memory != nullptr) {
        cudaFree(buffer.device_memory);
      }
    }
    if (stream_ != nullptr) {
      cudaStreamDestroy(stream_);
    }
    DestroyTensorRt(context_);
    DestroyTensorRt(engine_);
    DestroyTensorRt(runtime_);
  }

  bool Init(const std::string& engine_path, int device_id) {
    if (initialized_ || engine_path.empty() || device_id < 0 ||
        !CudaSucceeded(cudaSetDevice(device_id), "cudaSetDevice")) {
      return false;
    }
    device_id_ = device_id;

    std::ifstream engine_file(engine_path, std::ios::binary | std::ios::ate);
    if (!engine_file.is_open()) {
      AERROR << "Unable to open HiVT TensorRT engine: " << engine_path;
      return false;
    }
    const std::streamsize engine_size = engine_file.tellg();
    if (engine_size <= 0) {
      AERROR << "HiVT TensorRT engine is empty: " << engine_path;
      return false;
    }
    std::vector<char> engine_bytes(static_cast<std::size_t>(engine_size));
    engine_file.seekg(0, std::ios::beg);
    if (!engine_file.read(engine_bytes.data(), engine_size)) {
      AERROR << "Unable to read HiVT TensorRT engine: " << engine_path;
      return false;
    }

    if (!initLibNvInferPlugins(&g_logger, "")) {
      AERROR << "TensorRT plugin initialization failed";
      return false;
    }
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
    if (engine_ == nullptr) {
      AERROR << "Unable to deserialize HiVT TensorRT engine";
      return false;
    }
    context_ = engine_->createExecutionContext();
    if (context_ == nullptr ||
        !CudaSucceeded(cudaStreamCreate(&stream_), "cudaStreamCreate") ||
        !ConfigureBuffers()) {
      return false;
    }
    initialized_ = true;
    return true;
  }

  bool Run(const HiVTSceneInput& input, HiVTSceneOutput* output) {
    if (!initialized_ || output == nullptr || input.actor_count < 2 ||
        input.actor_count > kHiVTMaxActors || input.lane_vector_count < 1 ||
        input.lane_vector_count > kHiVTMaxLaneVectors ||
        input.actor_edge_count < 1 ||
        input.actor_edge_count > kHiVTMaxActorEdges ||
        input.lane_actor_edge_count < 1 ||
        input.lane_actor_edge_count > kHiVTMaxLaneActorEdges ||
        !CudaSucceeded(cudaSetDevice(device_id_), "cudaSetDevice") ||
        !HasValidInputSizes(input) || !SetInputShapes(input) ||
        !CopyInputs(input)) {
      AERROR << "HiVT TensorRT input validation or transfer failed";
      return false;
    }
    for (const auto& buffer : buffers_) {
      if (!context_->setTensorAddress(buffer.name, buffer.device_memory)) {
        AERROR << "TensorRT rejected tensor address for " << buffer.name;
        return false;
      }
    }
    const nvinfer1::Dims trajectory_shape =
        context_->getTensorShape("trajectories");
    const nvinfer1::Dims logit_shape = context_->getTensorShape("logits");
    if (!HasShape(trajectory_shape,
                  {kHiVTNumModes, input.actor_count, kHiVTFutureSteps, 4}) ||
        !HasShape(logit_shape, {input.actor_count, kHiVTNumModes})) {
      AERROR << "Unexpected HiVT output shapes";
      return false;
    }

    output->trajectories.resize(ElementCount(trajectory_shape));
    output->logits.resize(ElementCount(logit_shape));
    if (output->trajectories.size() * sizeof(float) >
            FindBuffer("trajectories")->capacity_bytes ||
        output->logits.size() * sizeof(float) >
            FindBuffer("logits")->capacity_bytes) {
      AERROR << "HiVT output exceeds allocated TensorRT buffer capacity";
      return false;
    }
    if (!context_->enqueueV3(stream_)) {
      AERROR << "HiVT TensorRT enqueueV3 failed";
      return false;
    }
    if (!CudaSucceeded(
            cudaMemcpyAsync(output->trajectories.data(),
                            FindBuffer("trajectories")->device_memory,
                            output->trajectories.size() * sizeof(float),
                            cudaMemcpyDeviceToHost, stream_),
            "cudaMemcpyAsync HiVT trajectories") ||
        !CudaSucceeded(cudaMemcpyAsync(output->logits.data(),
                                       FindBuffer("logits")->device_memory,
                                       output->logits.size() * sizeof(float),
                                       cudaMemcpyDeviceToHost, stream_),
                       "cudaMemcpyAsync HiVT logits") ||
        !CudaSucceeded(cudaStreamSynchronize(stream_),
                       "cudaStreamSynchronize HiVT")) {
      return false;
    }
    return true;
  }

 private:
  bool ConfigureBuffers() {
    buffers_ = {
        {"x", nvinfer1::DataType::kFLOAT,
         kHiVTMaxActors * kHiVTHistoricalSteps * 2 * sizeof(float)},
        {"positions", nvinfer1::DataType::kFLOAT,
         kHiVTMaxActors * (kHiVTHistoricalSteps + kHiVTFutureSteps) * 2 *
             sizeof(float)},
        {"edge_index", nvinfer1::DataType::kINT64,
         2 * kHiVTMaxActorEdges * sizeof(int64_t)},
        {"padding_mask", nvinfer1::DataType::kBOOL,
         kHiVTMaxActors * (kHiVTHistoricalSteps + kHiVTFutureSteps)},
        {"bos_mask", nvinfer1::DataType::kBOOL,
         kHiVTMaxActors * kHiVTHistoricalSteps},
        {"rotate_angles", nvinfer1::DataType::kFLOAT,
         kHiVTMaxActors * sizeof(float)},
        {"lane_vectors", nvinfer1::DataType::kFLOAT,
         kHiVTMaxLaneVectors * 2 * sizeof(float)},
        {"is_intersections", nvinfer1::DataType::kUINT8, kHiVTMaxLaneVectors},
        {"turn_directions", nvinfer1::DataType::kUINT8, kHiVTMaxLaneVectors},
        {"traffic_controls", nvinfer1::DataType::kUINT8, kHiVTMaxLaneVectors},
        {"lane_actor_index", nvinfer1::DataType::kINT64,
         2 * kHiVTMaxLaneActorEdges * sizeof(int64_t)},
        {"lane_actor_vectors", nvinfer1::DataType::kFLOAT,
         2 * kHiVTMaxLaneActorEdges * sizeof(float)},
        {"trajectories", nvinfer1::DataType::kFLOAT,
         kHiVTNumModes * kHiVTMaxActors * kHiVTFutureSteps * 4 * sizeof(float)},
        {"logits", nvinfer1::DataType::kFLOAT,
         kHiVTMaxActors * kHiVTNumModes * sizeof(float)},
    };
    for (auto& buffer : buffers_) {
      if (!HasTensor(buffer.name, buffer.type, IsOutput(buffer.name))) {
        AERROR << "HiVT engine tensor contract mismatch: " << buffer.name;
        return false;
      }
      if (!CudaSucceeded(
              cudaMalloc(&buffer.device_memory, buffer.capacity_bytes),
              "cudaMalloc HiVT tensor")) {
        return false;
      }
    }
    return true;
  }

  bool HasTensor(const char* name, nvinfer1::DataType expected_type,
                 bool expect_output) const {
    for (int index = 0; index < engine_->getNbIOTensors(); ++index) {
      const char* tensor_name = engine_->getIOTensorName(index);
      if (tensor_name == nullptr || std::string(tensor_name) != name) {
        continue;
      }
      const auto mode = engine_->getTensorIOMode(name);
      const bool is_output = mode == nvinfer1::TensorIOMode::kOUTPUT;
      return is_output == expect_output &&
             engine_->getTensorDataType(name) == expected_type;
    }
    return false;
  }

  bool IsOutput(const char* name) const {
    return std::string(name) == "trajectories" || std::string(name) == "logits";
  }

  bool SetInputShapes(const HiVTSceneInput& input) {
    return context_->setInputShape(
               "x", MakeDims({input.actor_count, kHiVTHistoricalSteps, 2})) &&
           context_->setInputShape(
               "positions",
               MakeDims({input.actor_count,
                         kHiVTHistoricalSteps + kHiVTFutureSteps, 2})) &&
           context_->setInputShape("edge_index",
                                   MakeDims({2, input.actor_edge_count})) &&
           context_->setInputShape(
               "padding_mask",
               MakeDims({input.actor_count,
                         kHiVTHistoricalSteps + kHiVTFutureSteps})) &&
           context_->setInputShape(
               "bos_mask",
               MakeDims({input.actor_count, kHiVTHistoricalSteps})) &&
           context_->setInputShape("rotate_angles",
                                   MakeDims({input.actor_count})) &&
           context_->setInputShape("lane_vectors",
                                   MakeDims({input.lane_vector_count, 2})) &&
           context_->setInputShape("is_intersections",
                                   MakeDims({input.lane_vector_count})) &&
           context_->setInputShape("turn_directions",
                                   MakeDims({input.lane_vector_count})) &&
           context_->setInputShape("traffic_controls",
                                   MakeDims({input.lane_vector_count})) &&
           context_->setInputShape(
               "lane_actor_index",
               MakeDims({2, input.lane_actor_edge_count})) &&
           context_->setInputShape("lane_actor_vectors",
                                   MakeDims({input.lane_actor_edge_count, 2}));
  }

  bool CopyInputs(const HiVTSceneInput& input) {
    return Copy("x", input.x) && Copy("positions", input.positions) &&
           Copy("edge_index", input.edge_index) &&
           Copy("padding_mask", input.padding_mask) &&
           Copy("bos_mask", input.bos_mask) &&
           Copy("rotate_angles", input.rotate_angles) &&
           Copy("lane_vectors", input.lane_vectors) &&
           Copy("is_intersections", input.is_intersections) &&
           Copy("turn_directions", input.turn_directions) &&
           Copy("traffic_controls", input.traffic_controls) &&
           Copy("lane_actor_index", input.lane_actor_index) &&
           Copy("lane_actor_vectors", input.lane_actor_vectors);
  }

  bool HasValidInputSizes(const HiVTSceneInput& input) const {
    const std::size_t actors = input.actor_count;
    const std::size_t lanes = input.lane_vector_count;
    const std::size_t actor_edges = input.actor_edge_count;
    const std::size_t lane_actor_edges = input.lane_actor_edge_count;
    return input.actor_ids.size() == actors &&
           input.x.size() == actors * kHiVTHistoricalSteps * 2 &&
           input.positions.size() ==
               actors * (kHiVTHistoricalSteps + kHiVTFutureSteps) * 2 &&
           input.edge_index.size() == 2 * actor_edges &&
           input.padding_mask.size() ==
               actors * (kHiVTHistoricalSteps + kHiVTFutureSteps) &&
           input.bos_mask.size() == actors * kHiVTHistoricalSteps &&
           input.rotate_angles.size() == actors &&
           input.lane_vectors.size() == lanes * 2 &&
           input.is_intersections.size() == lanes &&
           input.turn_directions.size() == lanes &&
           input.traffic_controls.size() == lanes &&
           input.lane_actor_index.size() == 2 * lane_actor_edges &&
           input.lane_actor_vectors.size() == 2 * lane_actor_edges;
  }

  template <typename Element>
  bool Copy(const char* name, const std::vector<Element>& values) {
    const Buffer* buffer = FindBuffer(name);
    const std::size_t bytes = values.size() * sizeof(Element);
    return buffer != nullptr && bytes <= buffer->capacity_bytes &&
           CudaSucceeded(
               cudaMemcpyAsync(buffer->device_memory, values.data(), bytes,
                               cudaMemcpyHostToDevice, stream_),
               "cudaMemcpyAsync HiVT input");
  }

  Buffer* FindBuffer(const char* name) {
    for (auto& buffer : buffers_) {
      if (std::string(buffer.name) == name) {
        return &buffer;
      }
    }
    return nullptr;
  }

  const Buffer* FindBuffer(const char* name) const {
    for (const auto& buffer : buffers_) {
      if (std::string(buffer.name) == name) {
        return &buffer;
      }
    }
    return nullptr;
  }

  static bool HasShape(const nvinfer1::Dims& actual,
                       std::initializer_list<int> expected) {
    if (actual.nbDims != static_cast<int>(expected.size())) {
      return false;
    }
    int index = 0;
    for (const int dimension : expected) {
      if (actual.d[index++] != dimension) {
        return false;
      }
    }
    return true;
  }

  int device_id_ = 0;
  bool initialized_ = false;
  nvinfer1::IRuntime* runtime_ = nullptr;
  nvinfer1::ICudaEngine* engine_ = nullptr;
  nvinfer1::IExecutionContext* context_ = nullptr;
  cudaStream_t stream_ = nullptr;
  std::vector<Buffer> buffers_;
};

HiVTTensorRtExecutor::HiVTTensorRtExecutor()
    : impl_(std::make_unique<Impl>()) {}

HiVTTensorRtExecutor::~HiVTTensorRtExecutor() = default;

bool HiVTTensorRtExecutor::Init(const std::string& engine_path, int device_id) {
  return impl_->Init(engine_path, device_id);
}

bool HiVTTensorRtExecutor::Run(const HiVTSceneInput& input,
                               HiVTSceneOutput* output) {
  return impl_->Run(input, output);
}

}  // namespace prediction
}  // namespace apollo
