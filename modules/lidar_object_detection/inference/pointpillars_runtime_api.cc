// Copyright 2026 WheelOS. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "modules/lidar_object_detection/inference/pointpillars_runtime_api.h"

#include <NvInfer.h>
#include <NvOnnxParser.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <cuda_runtime.h>

#include "modules/lidar_object_detection/third_party/pointpillars/include/kernel.h"
#include "modules/lidar_object_detection/third_party/pointpillars/include/postprocess.h"

namespace {
void Check(bool ok, const std::string& message) {
  if (!ok) throw std::runtime_error(message);
}
void Cuda(cudaError_t status) {
  Check(status == cudaSuccess, cudaGetErrorString(status));
}
void Error(const std::exception& exception, char* error, std::size_t capacity) {
  if (error != nullptr && capacity != 0) {
    std::snprintf(error, capacity, "%s", exception.what());
  }
}
class Logger : public nvinfer1::ILogger {
 public:
  void log(Severity severity, const char* message) noexcept override {
    if (severity <= Severity::kWARNING) {
      std::fprintf(stderr, "PointPillars TensorRT: %s\n", message);
    }
  }
};
Logger logger;

std::vector<char> Read(const char* path) {
  Check(path != nullptr && *path != '\0', "Missing model path");
  std::ifstream input(path, std::ios::binary);
  Check(input.good(), std::string("Cannot open model: ") + path);
  std::vector<char> bytes{std::istreambuf_iterator<char>(input), {}};
  Check(!bytes.empty() && !input.bad(), "Empty or unreadable model");
  return bytes;
}

class Runtime {
 public:
  ~Runtime() {
    if (stream_ != nullptr) {
      cudaSetDevice(gpu_);
      cudaStreamSynchronize(stream_);
    }
    context_.reset();
    engine_.reset();
    runtime_.reset();
    for (void* buffer : buffers_) {
      const auto status = cudaFree(buffer);
      if (status != cudaSuccess) {
        std::fprintf(stderr, "PointPillars cleanup: %s\n",
                     cudaGetErrorString(status));
      }
    }
    if (stream_ != nullptr) cudaStreamDestroy(stream_);
  }

  template <typename T>
  T* Allocate(std::size_t count) {
    void* buffer = nullptr;
    Cuda(cudaMalloc(&buffer, count * sizeof(T)));
    buffers_.push_back(buffer);
    return static_cast<T*>(buffer);
  }

  void Init(const char* path, int gpu) {
    gpu_ = gpu;
    Cuda(cudaSetDevice(gpu_));
    Cuda(cudaStreamCreate(&stream_));
    auto bytes = Read(path);
    runtime_.reset(nvinfer1::createInferRuntime(logger));
    Check(runtime_ != nullptr, "Cannot create TensorRT runtime");
    engine_.reset(runtime_->deserializeCudaEngine(bytes.data(), bytes.size()));
    Check(engine_ != nullptr, "Cannot deserialize PointPillars engine");
    context_.reset(engine_->createExecutionContext());
    Check(context_ != nullptr, "Cannot create PointPillars context");
    const auto input_shape = engine_->getTensorShape("voxels");
    Check(input_shape.nbDims == 3 && input_shape.d[0] > 0 &&
              input_shape.d[0] <= MAX_VOXELS && input_shape.d[1] == 32 &&
              input_shape.d[2] == 10,
          "Expected fixed PointPillars voxels shape [1..40000,32,10]");
    max_pillars_ = input_shape.d[0];
    const auto cells = params_.grid_x_size * params_.grid_y_size;
    const auto anchors =
        params_.feature_x_size * params_.feature_y_size * params_.num_anchors;
    points_ = Allocate<float>(300000 * 4);
    mask_ = Allocate<unsigned int>(cells);
    voxels_ = Allocate<float>(cells * 32 * 4);
    voxel_features_ = Allocate<float>(MAX_VOXELS * 32 * 4);
    voxel_num_ = Allocate<unsigned int>(MAX_VOXELS);
    coords_ = Allocate<unsigned int>(MAX_VOXELS * 4);
    pillar_count_ = Allocate<unsigned int>(1);
    features_ = Allocate<float>(MAX_VOXELS * 32 * 10);
    cls_ = Allocate<float>(anchors * 3);
    boxes_ = Allocate<float>(anchors * 7);
    dirs_ = Allocate<float>(anchors * 2);
    decoded_ = Allocate<float>(anchors * 9 + 1);
    counter_ = Allocate<int>(1);
    anchors_ = Allocate<float>(24);
    bottoms_ = Allocate<float>(3);
    Cuda(cudaMemcpyAsync(anchors_, params_.anchors, sizeof(params_.anchors),
                         cudaMemcpyHostToDevice, stream_));
    Cuda(cudaMemcpyAsync(bottoms_, params_.anchor_bottom_heights,
                         sizeof(params_.anchor_bottom_heights),
                         cudaMemcpyHostToDevice, stream_));
    Cuda(cudaStreamSynchronize(stream_));
    Check(engine_->getNbIOTensors() == 6,
          "Expected exactly six PointPillars engine tensors");
    Bind("voxels", features_, nvinfer1::DataType::kFLOAT,
         max_pillars_ * 32 * 10);
    Bind("voxel_idxs", coords_, nvinfer1::DataType::kINT32, max_pillars_ * 4);
    Bind("voxel_num", pillar_count_, nvinfer1::DataType::kINT32, 1);
    Bind("cls_preds", cls_, nvinfer1::DataType::kFLOAT, anchors * 3);
    Bind("box_preds", boxes_, nvinfer1::DataType::kFLOAT, anchors * 7);
    Bind("dir_cls_preds", dirs_, nvinfer1::DataType::kFLOAT, anchors * 2);
  }

  void Infer(const float* points, std::uint32_t count,
             PointPillarsDetection* output, std::size_t capacity,
             std::size_t* output_count) {
    std::lock_guard<std::mutex> lock(mutex_);
    Check(points != nullptr && count > 0 && count <= 300000,
          "PointPillars requires 1..300000 XYZI points");
    Check(output != nullptr && output_count != nullptr,
          "Null detection output");
    for (std::size_t i = 0; i < static_cast<std::size_t>(count) * 4; ++i) {
      Check(std::isfinite(points[i]), "Nonfinite XYZI feature");
    }
    Cuda(cudaSetDevice(gpu_));
    const auto cells = params_.grid_x_size * params_.grid_y_size;
    Cuda(cudaMemcpyAsync(points_, points, count * 4 * sizeof(float),
                         cudaMemcpyHostToDevice, stream_));
    Cuda(cudaMemsetAsync(mask_, 0, cells * sizeof(unsigned int), stream_));
    Cuda(cudaMemsetAsync(voxels_, 0, cells * 32 * 4 * sizeof(float), stream_));
    Cuda(cudaMemsetAsync(voxel_features_, 0,
                         MAX_VOXELS * 32 * 4 * sizeof(float), stream_));
    Cuda(cudaMemsetAsync(voxel_num_, 0, MAX_VOXELS * sizeof(unsigned int),
                         stream_));
    Cuda(cudaMemsetAsync(coords_, 0, MAX_VOXELS * 4 * sizeof(unsigned int),
                         stream_));
    Cuda(cudaMemsetAsync(features_, 0, MAX_VOXELS * 32 * 10 * sizeof(float),
                         stream_));
    Cuda(cudaMemsetAsync(pillar_count_, 0, sizeof(unsigned int), stream_));
    Cuda(generateVoxels_random_launch(
        points_, count, params_.min_x_range, params_.max_x_range,
        params_.min_y_range, params_.max_y_range, params_.min_z_range,
        params_.max_z_range, params_.pillar_x_size, params_.pillar_y_size,
        params_.pillar_z_size, params_.grid_y_size, params_.grid_x_size, mask_,
        voxels_, stream_));
    Cuda(generateBaseFeatures_launch(
        mask_, voxels_, params_.grid_y_size, params_.grid_x_size, pillar_count_,
        voxel_features_, voxel_num_, coords_, stream_));
    unsigned int pillars = 0;
    Cuda(cudaMemcpyAsync(&pillars, pillar_count_, sizeof(pillars),
                         cudaMemcpyDeviceToHost, stream_));
    Cuda(cudaStreamSynchronize(stream_));
    Check(pillars > 0 && pillars <= max_pillars_,
          "No in-range points or pillar capacity exceeded");
    Cuda(generateFeatures_launch(voxel_features_, voxel_num_, coords_,
                                 pillar_count_, params_.pillar_x_size,
                                 params_.pillar_y_size, params_.pillar_z_size,
                                 params_.min_x_range, params_.min_y_range,
                                 params_.min_z_range, features_, stream_));
    Check(context_->enqueueV3(stream_), "PointPillars enqueue failed");
    Cuda(cudaMemsetAsync(counter_, 0, sizeof(int), stream_));
    Cuda(postprocess_launch(
        cls_, boxes_, dirs_, anchors_, bottoms_, decoded_, counter_,
        params_.min_x_range, params_.max_x_range, params_.min_y_range,
        params_.max_y_range, params_.feature_x_size, params_.feature_y_size,
        params_.num_anchors, params_.num_classes, params_.num_box_values,
        params_.score_thresh, params_.dir_offset, stream_));
    int objects = 0;
    Cuda(cudaMemcpyAsync(&objects, counter_, sizeof(objects),
                         cudaMemcpyDeviceToHost, stream_));
    Cuda(cudaStreamSynchronize(stream_));
    Check(objects >= 0 && objects <= params_.feature_x_size *
                                         params_.feature_y_size *
                                         params_.num_anchors,
          "Invalid decoded detection count");
    std::vector<float> decoded(static_cast<std::size_t>(objects) * 9);
    if (objects != 0) {
      Cuda(cudaMemcpyAsync(decoded.data(), decoded_ + 1,
                           decoded.size() * sizeof(float),
                           cudaMemcpyDeviceToHost, stream_));
      Cuda(cudaStreamSynchronize(stream_));
    }
    std::vector<Bndbox> candidates, selected;
    for (int i = 0; i < objects; ++i) {
      const float* p = decoded.data() + i * 9;
      for (int j = 0; j < 9; ++j) {
        Check(std::isfinite(p[j]), "Nonfinite decoded detection");
      }
      Check(p[3] > 0 && p[4] > 0 && p[5] > 0 && p[7] >= 0 && p[7] < 3,
            "Invalid decoded box dimensions or class");
      candidates.emplace_back(p[0], p[1], p[2], p[3], p[4], p[5], p[6],
                              static_cast<int>(p[7]), p[8]);
    }
    nms_cpu(candidates, params_.nms_thresh, selected);
    Check(selected.size() <= capacity, "Detection output capacity exceeded");
    for (std::size_t i = 0; i < selected.size(); ++i) {
      const auto& b = selected[i];
      output[i] = {b.x, b.y, b.z, b.w, b.l, b.h, b.rt, b.id, b.score};
    }
    *output_count = selected.size();
  }

 private:
  void Bind(const char* name, void* pointer, nvinfer1::DataType type,
            std::int64_t elements) {
    const auto shape = engine_->getTensorShape(name);
    std::int64_t size = shape.nbDims > 0 ? 1 : 0;
    for (int i = 0; i < shape.nbDims; ++i) size *= shape.d[i];
    Check(engine_->getTensorDataType(name) == type && size == elements,
          std::string("Unexpected tensor shape/type: ") + name);
    Check(context_->setTensorAddress(name, pointer),
          std::string("Cannot bind tensor: ") + name);
  }
  Params params_;
  int gpu_ = 0;
  unsigned int max_pillars_ = 0;
  cudaStream_t stream_ = nullptr;
  std::vector<void*> buffers_;
  std::unique_ptr<nvinfer1::IRuntime> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext> context_;
  std::mutex mutex_;
  float *points_, *voxels_, *voxel_features_, *features_, *cls_, *boxes_,
      *dirs_, *decoded_, *anchors_, *bottoms_;
  unsigned int *mask_, *voxel_num_, *coords_, *pillar_count_;
  int* counter_;
};
}  // namespace

extern "C" {
std::uint32_t pointpillars_runtime_abi_version() { return 1; }
std::size_t pointpillars_runtime_detection_size() {
  return sizeof(PointPillarsDetection);
}
bool pointpillars_runtime_build_engine(const char* onnx, const char* path,
                                       int gpu, char* error,
                                       std::size_t capacity) {
  try {
    Check(onnx != nullptr && path != nullptr && *path != '\0',
          "Missing ONNX or engine path");
    Cuda(cudaSetDevice(gpu));
    std::unique_ptr<nvinfer1::IBuilder> builder(
        nvinfer1::createInferBuilder(logger));
    Check(builder != nullptr, "Cannot create TensorRT builder");
    std::unique_ptr<nvinfer1::INetworkDefinition> network(
        builder->createNetworkV2(0));
    Check(network != nullptr, "Cannot create TensorRT network");
    std::unique_ptr<nvonnxparser::IParser> parser(
        nvonnxparser::createParser(*network, logger));
    Check(
        parser != nullptr &&
            parser->parseFromFile(
                onnx, static_cast<int>(nvinfer1::ILogger::Severity::kWARNING)),
        "Cannot parse PointPillars ONNX");
    std::unique_ptr<nvinfer1::IBuilderConfig> config(
        builder->createBuilderConfig());
    Check(config != nullptr, "Cannot create TensorRT builder configuration");
    config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, 1UL << 30);
    config->setFlag(nvinfer1::BuilderFlag::kFP16);
    std::unique_ptr<nvinfer1::IHostMemory> serialized(
        builder->buildSerializedNetwork(*network, *config));
    Check(serialized != nullptr, "Cannot build PointPillars engine");
    std::ofstream output(path, std::ios::binary);
    output.write(static_cast<const char*>(serialized->data()),
                 serialized->size());
    output.close();
    Check(output.good(), "Cannot write PointPillars engine");
    return true;
  } catch (const std::exception& exception) {
    Error(exception, error, capacity);
    return false;
  }
}
void* pointpillars_runtime_create(const char* path, int gpu, char* error,
                                  std::size_t capacity) {
  try {
    auto runtime = std::make_unique<Runtime>();
    runtime->Init(path, gpu);
    return runtime.release();
  } catch (const std::exception& exception) {
    Error(exception, error, capacity);
    return nullptr;
  }
}
void pointpillars_runtime_destroy(void* runtime) {
  delete static_cast<Runtime*>(runtime);
}
bool pointpillars_runtime_infer(void* runtime, const float* points,
                                std::uint32_t count,
                                PointPillarsDetection* output,
                                std::size_t capacity, std::size_t* output_count,
                                char* error, std::size_t error_capacity) {
  if (output_count != nullptr) *output_count = 0;
  try {
    Check(runtime != nullptr, "Null PointPillars runtime");
    static_cast<Runtime*>(runtime)->Infer(points, count, output, capacity,
                                          output_count);
    return true;
  } catch (const std::exception& exception) {
    Error(exception, error, error_capacity);
    return false;
  }
}
}
