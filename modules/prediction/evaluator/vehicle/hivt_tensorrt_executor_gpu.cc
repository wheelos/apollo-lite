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

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fstream>
#include <initializer_list>
#include <numeric>
#include <string>
#include <sys/resource.h>
#include <time.h>
#include <utility>
#include <vector>

#include <cuda_runtime_api.h>

#include "cyber/common/log.h"
#include "modules/prediction/evaluator/vehicle/hivt_tensorrt_executor.h"
#include "modules/prediction/common/prediction_system_gflags.h"

namespace apollo {
namespace prediction {

namespace {

constexpr int kHiVTMaxActorEdges = kHiVTMaxActors * (kHiVTMaxActors - 1);
constexpr int kHiVTMaxLaneActorEdges = kHiVTMaxActors * kHiVTMaxLaneVectors;
constexpr std::size_t kInputBufferAlignment = 256;

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

struct ThreadUsage {
  double cpu_time_ms = 0.0;
  long voluntary_context_switches = 0;
  long involuntary_context_switches = 0;
};

bool GetThreadUsage(ThreadUsage* usage) {
  struct timespec cpu_time;
  struct rusage resource_usage;
  if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &cpu_time) != 0) {
    AWARN << "Unable to read HiVT profiling thread CPU time: "
          << std::strerror(errno);
    return false;
  }
  if (getrusage(RUSAGE_THREAD, &resource_usage) != 0) {
    AWARN << "Unable to read HiVT profiling thread context switches: "
          << std::strerror(errno);
    return false;
  }
  usage->cpu_time_ms =
      cpu_time.tv_sec * 1000.0 + cpu_time.tv_nsec / 1000000.0;
  usage->voluntary_context_switches = resource_usage.ru_nvcsw;
  usage->involuntary_context_switches = resource_usage.ru_nivcsw;
  return true;
}

ThreadUsage Difference(const ThreadUsage& end, const ThreadUsage& start) {
  return {end.cpu_time_ms - start.cpu_time_ms,
          end.voluntary_context_switches - start.voluntary_context_switches,
          end.involuntary_context_switches -
              start.involuntary_context_switches};
}

}  // namespace

class HiVTTensorRtExecutor::Impl {
 public:
  struct Buffer {
    const char* name;
    nvinfer1::DataType type;
    std::size_t capacity_bytes;
    void* device_memory = nullptr;
    std::size_t input_offset = 0;
  };

  ~Impl() {
    if (initialized_) {
      cudaSetDevice(device_id_);
    }
    if (inference_start_event_ != nullptr) {
      cudaEventDestroy(inference_start_event_);
    }
    if (inference_end_event_ != nullptr) {
      cudaEventDestroy(inference_end_event_);
    }
    for (auto& buffer : buffers_) {
      if (buffer.device_memory != nullptr && IsOutput(buffer.name)) {
        cudaFree(buffer.device_memory);
      }
    }
    if (host_input_staging_ != nullptr) {
      cudaFreeHost(host_input_staging_);
    }
    if (device_input_memory_ != nullptr) {
      cudaFree(device_input_memory_);
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
    if (FLAGS_prediction_enable_profiling) {
      const cudaError_t start_status =
          cudaEventCreate(&inference_start_event_);
      const cudaError_t end_status =
          start_status == cudaSuccess
              ? cudaEventCreate(&inference_end_event_)
              : start_status;
      if (start_status == cudaSuccess && end_status == cudaSuccess) {
        inference_events_ready_ = true;
      } else {
        AWARN << "HiVT CUDA inference timing events are unavailable: "
              << cudaGetErrorString(start_status == cudaSuccess
                                        ? end_status
                                        : start_status);
      }
    }
    return true;
  }

  bool Run(const HiVTSceneInput& input, HiVTSceneOutput* output) {
    const auto prepare_start = std::chrono::steady_clock::now();
    if (!initialized_ || output == nullptr || input.actor_count < 2 ||
        input.actor_count > kHiVTMaxActors || input.lane_vector_count < 1 ||
        input.lane_vector_count > kHiVTMaxLaneVectors ||
        input.actor_edge_count < 1 ||
        input.actor_edge_count > kHiVTMaxActorEdges ||
        input.lane_actor_edge_count < 1 ||
        input.lane_actor_edge_count > kHiVTMaxLaneActorEdges) {
      AERROR << "HiVT TensorRT input validation or transfer failed";
      return false;
    }
    ThreadUsage input_pre_transfer_usage_start;
    const bool input_pre_transfer_usage_start_ready =
        FLAGS_prediction_enable_profiling &&
        GetThreadUsage(&input_pre_transfer_usage_start);
    const auto device_select_start = std::chrono::steady_clock::now();
    if (!CudaSucceeded(cudaSetDevice(device_id_), "cudaSetDevice")) {
      AERROR << "HiVT TensorRT input validation or transfer failed";
      return false;
    }
    const auto device_select_end = std::chrono::steady_clock::now();
    const auto input_validation_start = device_select_end;
    if (!HasValidInputSizes(input)) {
      AERROR << "HiVT TensorRT input validation or transfer failed";
      return false;
    }
    const auto input_validation_end = std::chrono::steady_clock::now();
    const auto input_shape_setup_start = input_validation_end;
    if (!SetInputShapes(input)) {
      AERROR << "HiVT TensorRT input validation or transfer failed";
      return false;
    }
    const auto input_shape_setup_end = std::chrono::steady_clock::now();
    const auto input_shapes_ready = std::chrono::steady_clock::now();
    input_copy_bytes_ = 0;
    if (!CopyInputs(input)) {
      AERROR << "HiVT TensorRT input validation or transfer failed";
      return false;
    }
    const auto input_packed = std::chrono::steady_clock::now();
    if (input_copy_bytes_ == 0 ||
        input_copy_bytes_ > input_capacity_bytes_) {
      AERROR << "HiVT TensorRT packed input size is invalid";
      return false;
    }
    ThreadUsage input_transfer_usage_start;
    ThreadUsage input_transfer_usage_end;
    const auto input_transfer_profile_probe_start =
        std::chrono::steady_clock::now();
    const bool input_transfer_usage_start_ready =
        FLAGS_prediction_enable_profiling &&
        GetThreadUsage(&input_transfer_usage_start);
    const auto input_transfer_call_start = std::chrono::steady_clock::now();
    const auto input_transfer_profile_probe_end =
        std::chrono::steady_clock::now();
    const cudaError_t input_transfer_status =
        cudaMemcpyAsync(device_input_memory_, host_input_staging_,
                        input_copy_bytes_, cudaMemcpyHostToDevice, stream_);
    const auto input_copy_submitted = std::chrono::steady_clock::now();
    const bool input_transfer_usage_end_ready =
        FLAGS_prediction_enable_profiling &&
        GetThreadUsage(&input_transfer_usage_end);
    if (!CudaSucceeded(input_transfer_status,
                       "cudaMemcpyAsync HiVT packed inputs")) {
      AERROR << "HiVT TensorRT input transfer failed";
      return false;
    }
    const auto prepare_end = std::chrono::steady_clock::now();
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
    const auto tensor_setup_end = std::chrono::steady_clock::now();
    bool inference_start_recorded = false;
    if (inference_events_ready_) {
      const cudaError_t status =
          cudaEventRecord(inference_start_event_, stream_);
      if (status == cudaSuccess) {
        inference_start_recorded = true;
      } else {
        AWARN << "Unable to record HiVT CUDA inference start event: "
              << cudaGetErrorString(status);
      }
    }
    ThreadUsage enqueue_usage_start;
    ThreadUsage enqueue_usage_end;
    const bool enqueue_usage_start_ready =
        FLAGS_prediction_enable_profiling &&
        GetThreadUsage(&enqueue_usage_start);
    const auto enqueue_start = std::chrono::steady_clock::now();
    const bool enqueue_succeeded = context_->enqueueV3(stream_);
    const auto enqueue_end = std::chrono::steady_clock::now();
    const bool enqueue_usage_end_ready =
        FLAGS_prediction_enable_profiling &&
        GetThreadUsage(&enqueue_usage_end);
    if (!enqueue_succeeded) {
      AERROR << "HiVT TensorRT enqueueV3 failed";
      return false;
    }
    bool inference_end_recorded = false;
    if (inference_start_recorded) {
      const cudaError_t status = cudaEventRecord(inference_end_event_, stream_);
      if (status == cudaSuccess) {
        inference_end_recorded = true;
      } else {
        AWARN << "Unable to record HiVT CUDA inference end event: "
              << cudaGetErrorString(status);
      }
    }
    const auto completion_start = std::chrono::steady_clock::now();
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
    const auto completion_end = std::chrono::steady_clock::now();
    if (FLAGS_prediction_enable_profiling) {
      const auto milliseconds = [](const auto& start, const auto& end) {
        return std::chrono::duration<double, std::milli>(end - start).count();
      };
      output->input_prepare_ms = milliseconds(prepare_start, prepare_end);
      output->input_pre_transfer_ms =
          milliseconds(prepare_start, input_transfer_call_start);
      output->input_device_select_ms =
          milliseconds(device_select_start, device_select_end);
      output->input_validation_ms =
          milliseconds(input_validation_start, input_validation_end);
      output->input_shape_setup_ms =
          milliseconds(input_shape_setup_start, input_shape_setup_end);
      if (input_pre_transfer_usage_start_ready &&
          input_transfer_usage_start_ready) {
        const ThreadUsage usage = Difference(input_transfer_usage_start,
                                             input_pre_transfer_usage_start);
        output->input_pre_transfer_host_cpu_ms = usage.cpu_time_ms;
        output->input_pre_transfer_voluntary_context_switches =
            usage.voluntary_context_switches;
        output->input_pre_transfer_involuntary_context_switches =
            usage.involuntary_context_switches;
      }
      output->input_pack_ms =
          milliseconds(input_shapes_ready, input_packed);
      output->input_transfer_profile_probe_ms =
          milliseconds(input_transfer_profile_probe_start,
                       input_transfer_profile_probe_end);
      output->input_transfer_submit_ms =
          milliseconds(input_transfer_call_start, input_copy_submitted);
      output->input_copy_submit_ms =
          milliseconds(input_shapes_ready, input_copy_submitted);
      if (input_transfer_usage_start_ready &&
          input_transfer_usage_end_ready) {
        const ThreadUsage usage =
            Difference(input_transfer_usage_end, input_transfer_usage_start);
        output->input_transfer_host_cpu_ms = usage.cpu_time_ms;
        output->input_transfer_voluntary_context_switches =
            usage.voluntary_context_switches;
        output->input_transfer_involuntary_context_switches =
            usage.involuntary_context_switches;
      }
      output->tensor_binding_setup_ms =
          milliseconds(input_copy_submitted, tensor_setup_end);
      output->enqueue_cpu_ms = milliseconds(enqueue_start, enqueue_end);
      if (enqueue_usage_start_ready && enqueue_usage_end_ready) {
        const ThreadUsage usage =
            Difference(enqueue_usage_end, enqueue_usage_start);
        output->enqueue_host_cpu_ms = usage.cpu_time_ms;
        output->enqueue_voluntary_context_switches =
            usage.voluntary_context_switches;
        output->enqueue_involuntary_context_switches =
            usage.involuntary_context_switches;
      }
      output->completion_wait_ms =
          milliseconds(completion_start, completion_end);
      if (inference_end_recorded) {
        float inference_ms = 0.0F;
        const cudaError_t status = cudaEventElapsedTime(
            &inference_ms, inference_start_event_, inference_end_event_);
        if (status == cudaSuccess) {
          output->gpu_inference_ms = inference_ms;
        } else {
          AWARN << "Unable to read HiVT CUDA inference timing: "
                << cudaGetErrorString(status);
        }
      }
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
    std::size_t input_capacity = 0;
    for (auto& buffer : buffers_) {
      if (!HasTensor(buffer.name, buffer.type, IsOutput(buffer.name))) {
        AERROR << "HiVT engine tensor contract mismatch: " << buffer.name;
        return false;
      }
      if (IsOutput(buffer.name)) {
        if (!CudaSucceeded(
                cudaMalloc(&buffer.device_memory, buffer.capacity_bytes),
                "cudaMalloc HiVT output tensor")) {
          return false;
        }
        continue;
      }
      input_capacity =
          (input_capacity + kInputBufferAlignment - 1) &
          ~(kInputBufferAlignment - 1);
      buffer.input_offset = input_capacity;
      input_capacity += buffer.capacity_bytes;
    }
    input_capacity_bytes_ = input_capacity;
    if (input_capacity == 0 ||
        !CudaSucceeded(cudaMalloc(&device_input_memory_, input_capacity),
                       "cudaMalloc HiVT input tensors") ||
        !CudaSucceeded(cudaHostAlloc(&host_input_staging_, input_capacity,
                                     cudaHostAllocDefault),
                       "cudaHostAlloc HiVT input staging")) {
      return false;
    }
    for (auto& buffer : buffers_) {
      if (!IsOutput(buffer.name)) {
        buffer.device_memory =
            static_cast<char*>(device_input_memory_) + buffer.input_offset;
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
    Buffer* buffer = FindBuffer(name);
    const std::size_t bytes = values.size() * sizeof(Element);
    if (buffer == nullptr || IsOutput(name) || host_input_staging_ == nullptr ||
        bytes > buffer->capacity_bytes) {
      return false;
    }
    std::memcpy(static_cast<char*>(host_input_staging_) + buffer->input_offset,
                values.data(), bytes);
    input_copy_bytes_ =
        std::max(input_copy_bytes_, buffer->input_offset + bytes);
    return true;
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
  cudaEvent_t inference_start_event_ = nullptr;
  cudaEvent_t inference_end_event_ = nullptr;
  bool inference_events_ready_ = false;
  void* device_input_memory_ = nullptr;
  void* host_input_staging_ = nullptr;
  std::size_t input_copy_bytes_ = 0;
  std::size_t input_capacity_bytes_ = 0;
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
