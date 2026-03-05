/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifdef NV_TENSORRT_MAJOR
#if NV_TENSORRT_MAJOR >= 8
#include "modules/perception/inference/tensorrt/rt_legacy.h"
#endif
#endif

#include <sys/stat.h>

#include <algorithm>
#include <fstream>
#include <sstream>

#include "cyber/common/log.h"
#include "modules/perception/inference/onnx_multi_batch/onnx_multi_batch_infer.h"

namespace apollo {
namespace perception {
namespace inference {

// Logger for TensorRT info/warning/errors
class OnnxLogger : public nvinfer1::ILogger {
 public:
  explicit OnnxLogger(Severity severity = Severity::kWARNING)
      : reportable_severity_(severity) {}

  void log(Severity severity, const char* msg) noexcept override {
    if (severity > reportable_severity_) return;
    switch (severity) {
      case Severity::kINTERNAL_ERROR:
      case Severity::kERROR:
        AERROR << msg;
        break;
      case Severity::kWARNING:
        AWARN << msg;
        break;
      case Severity::kINFO:
      case Severity::kVERBOSE:
        ADEBUG << msg;
        break;
      default:
        break;
    }
  }

 private:
  Severity reportable_severity_;
};

static OnnxLogger g_onnx_logger;

static bool CudaOk(cudaError_t err, const char* what) {
  if (err == cudaSuccess) return true;
  AERROR << "CUDA error: " << cudaGetErrorString(err) << " @ " << what;
  return false;
}

MultiBatchInference::~MultiBatchInference() {
  if (network_) {
#if NV_TENSORRT_MAJOR >= 10
    delete network_;
#else
    network_->destroy();
#endif
    network_ = nullptr;
  }
  if (builder_) {
#if NV_TENSORRT_MAJOR >= 10
    delete builder_;
#else
    builder_->destroy();
#endif
    builder_ = nullptr;
  }
  if (context_) {
#if NV_TENSORRT_MAJOR >= 10
    delete context_;
#else
    context_->destroy();
#endif
    context_ = nullptr;
  }
  if (engine_) {
#if NV_TENSORRT_MAJOR >= 10
    delete engine_;
#else
    engine_->destroy();
#endif
    engine_ = nullptr;
  }
  if (runtime_) {
#if NV_TENSORRT_MAJOR >= 10
    delete runtime_;
#else
    runtime_->destroy();
#endif
    runtime_ = nullptr;
  }
  if (owns_stream_ && stream_ != 0) {
    cudaStreamDestroy(stream_);
    stream_ = 0;
  }
}

void MultiBatchInference::set_model_info(
    const std::string& onnx_file, const std::vector<std::string>& input_names,
    const std::vector<std::string>& output_names) {
  onnx_file_ = onnx_file;
  net_input_names_ = input_names;
  net_output_names_ = output_names;
}

void MultiBatchInference::SetStream(cudaStream_t stream) {
  if (owns_stream_ && stream_ != 0) {
    cudaStreamDestroy(stream_);
  }
  stream_ = stream;
  owns_stream_ = false;
}

std::shared_ptr<apollo::perception::base::Blob<float>>
MultiBatchInference::get_blob(const std::string& name) {
  auto iter = blobs_.find(name);
  if (iter == blobs_.end()) {
    return nullptr;
  }
  return iter->second;
}

bool MultiBatchInference::LoadCache(const std::string& path) const {
  struct stat buffer;
  if (stat(path.c_str(), &buffer) != 0) {
    AINFO << "Cannot find model cache: " << path
          << ", it will take minutes to generate...";
    return false;
  }
  return true;
}

void MultiBatchInference::FileInit() {
  model_file_ = onnx_file_;
  output_names_ = net_output_names_;
  input_names_ = net_input_names_;
}

bool MultiBatchInference::Init(
    const std::map<std::string, std::vector<int>>& shapes) {
  if (!CudaOk(cudaSetDevice(gpu_id_), "cudaSetDevice")) {
    return false;
  }
  if (stream_ == 0) {
    cudaError_t err = cudaStreamCreate(&stream_);
    if (err != cudaSuccess) {
      AERROR << "Failed to create cuda stream: " << cudaGetErrorString(err);
      return false;
    }
    owns_stream_ = true;
  }

  FileInit();
  if (!OnnxToTRTModel(model_file_)) {
    return false;
  }
  if (engine_ == nullptr) {
    AERROR << "Fail to load ONNX model: " << model_file_;
    return false;
  }

  context_ = engine_->createExecutionContext();
  if (context_ == nullptr) {
    AERROR << "Fail to create Execution Context";
    return false;
  }

  for (const auto& name : output_names_) {
    auto iter = shapes.find(name);
    if (iter != shapes.end()) {
      blobs_.emplace(name,
                     std::make_shared<apollo::perception::base::Blob<float>>(
                         iter->second));
    }
  }
  for (const auto& name : input_names_) {
    auto iter = shapes.find(name);
    if (iter != shapes.end()) {
      blobs_.emplace(name,
                     std::make_shared<apollo::perception::base::Blob<float>>(
                         iter->second));
    }
  }

  buffers_.resize(input_names_.size() + output_names_.size());
  return true;
}

void MultiBatchInference::SetOnnxOptimizationProfile(
    nvinfer1::IOptimizationProfile* profile, const BlobMap& input_shapes) {
  // For TensorRT 10 (explicit batch + dynamic shapes), choose a conservative
  // profile to cover a wide range while keeping a reasonable OPT point.
#if NV_TENSORRT_MAJOR >= 10
  const int min_batch = std::min(500, std::max(1, max_batch_size_));
  const int opt_batch = std::min(80000, std::max(1, max_batch_size_));
  const int max_batch = std::max(1, max_batch_size_);
#endif

  for (int i = 0; i < network_->getNbInputs(); ++i) {
    nvinfer1::ITensor* nv_input = network_->getInput(i);
    const char* nv_input_name = nv_input->getName();
    nvinfer1::Dims nv_input_dims = nv_input->getDimensions();
    auto it = input_shapes.find(nv_input_name);
    if (it != input_shapes.end()) {
      CHECK_EQ(nv_input_dims.nbDims, it->second->num_axes());
    }

    if (nv_input_dims.d[0] > 0) {
      continue;
    }

    for (int j = 1; j < nv_input_dims.nbDims; ++j) {
      if (it != input_shapes.end()) {
        CHECK(nv_input_dims.d[j] < 0 ||
              nv_input_dims.d[j] == it->second->shape(j));
        nv_input_dims.d[j] = it->second->shape(j);
      } else {
        CHECK(nv_input_dims.d[j] > 0);
      }
    }

#if NV_TENSORRT_MAJOR >= 10
    nv_input_dims.d[0] = min_batch;
    profile->setDimensions(nv_input_name, nvinfer1::OptProfileSelector::kMIN,
                           nv_input_dims);
    nv_input_dims.d[0] = opt_batch;
    profile->setDimensions(nv_input_name, nvinfer1::OptProfileSelector::kOPT,
                           nv_input_dims);
    nv_input_dims.d[0] = max_batch;
    profile->setDimensions(nv_input_name, nvinfer1::OptProfileSelector::kMAX,
                           nv_input_dims);
#else
    nv_input_dims.d[0] = 1;
    profile->setDimensions(nv_input_name, nvinfer1::OptProfileSelector::kMIN,
                           nv_input_dims);
    nv_input_dims.d[0] = std::max(1, static_cast<int>(max_batch_size_ / 2));
    profile->setDimensions(nv_input_name, nvinfer1::OptProfileSelector::kOPT,
                           nv_input_dims);
    nv_input_dims.d[0] = max_batch_size_;
    profile->setDimensions(nv_input_name, nvinfer1::OptProfileSelector::kMAX,
                           nv_input_dims);
#endif
  }
}

bool MultiBatchInference::OnnxToTRTModel(const std::string& model_file) {
  if (gpu_id_ < 0) {
    AERROR << "Must use GPU mode";
    return false;
  }

  auto DestroyParser = [](nvonnxparser::IParser* parser) {
    if (parser == nullptr) return;
#if NV_TENSORRT_MAJOR >= 10
    delete parser;
#else
    parser->destroy();
#endif
  };
  auto DestroyConfig = [](nvinfer1::IBuilderConfig* config) {
    if (config == nullptr) return;
#if NV_TENSORRT_MAJOR >= 10
    delete config;
#else
    config->destroy();
#endif
  };

  builder_ = nvinfer1::createInferBuilder(g_onnx_logger);
  network_ = builder_->createNetworkV2(
      1U << static_cast<int>(
          nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH));

  nvonnxparser::IParser* parser =
      nvonnxparser::createParser(*network_, g_onnx_logger);

  int verbosity = static_cast<int>(nvinfer1::ILogger::Severity::kWARNING);
  if (!parser->parseFromFile(model_file.c_str(), verbosity)) {
    AERROR << "Failure while parsing ONNX file: " << model_file;
    DestroyParser(parser);
    return false;
  }

#if NV_TENSORRT_MAJOR < 10
  builder_->setMaxBatchSize(max_batch_size_);
#endif
  nvinfer1::IBuilderConfig* config = builder_->createBuilderConfig();
#if NV_TENSORRT_MAJOR >= 10
  config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE,
                             static_cast<size_t>(1ULL << 30));
#else
  config->setMaxWorkspaceSize(1ULL << 30);
#endif

  const bool use_fp16 = enable_fp16_ && builder_->platformHasFastFp16();
  if (use_fp16) {
    config->setFlag(nvinfer1::BuilderFlag::kFP16);
  }

#if NV_TENSORRT_MAJOR >= 10
  const std::string trt_cache_path =
      model_file + (use_fp16 ? ".fp16.trt10.engine" : ".fp32.trt10.engine");
  const std::string trt_load_path = trt_cache_path;
#else
  const std::string trt_cache_path =
      model_file + (use_fp16 ? ".fp16.trt.engine" : ".fp32.trt.engine");
  std::string trt_load_path = trt_cache_path;
  if (use_fp16) {
    const std::string legacy_cache_path = model_file + ".trt.engine";
    struct stat buffer;
    if (stat(trt_cache_path.c_str(), &buffer) != 0 &&
        stat(legacy_cache_path.c_str(), &buffer) == 0) {
      trt_load_path = legacy_cache_path;
    }
  }
#endif

#if NV_TENSORRT_MAJOR >= 10
  bool need_build = !LoadCache(trt_load_path);
  if (!need_build) {
    AINFO << "Loading TensorRT engine from serialized model file: "
          << trt_load_path;
    std::ifstream plan_file(trt_load_path, std::ios::binary);
    if (!plan_file.is_open()) {
      AERROR << "Could not open serialized model: " << trt_load_path;
      need_build = true;
    } else {
      initLibNvInferPlugins(&g_onnx_logger, "");
      std::stringstream plan_buffer;
      plan_buffer << plan_file.rdbuf();
      std::string plan = plan_buffer.str();
      if (runtime_ != nullptr) {
        // Runtime must outlive all engines created from it.
        if (engine_ != nullptr) {
          delete engine_;
          engine_ = nullptr;
        }
        delete runtime_;
        runtime_ = nullptr;
      }
      runtime_ = nvinfer1::createInferRuntime(g_onnx_logger);
      engine_ = runtime_->deserializeCudaEngine(plan.data(), plan.size());
      if (engine_ == nullptr) {
        AWARN << "Failed to deserialize TensorRT engine cache (likely version "
                 "mismatch). Will rebuild engine: "
              << trt_load_path;
        need_build = true;
      }
    }
  }

  if (need_build) {
    AINFO << "Building TensorRT engine (" << (use_fp16 ? "FP16" : "FP32")
          << "), cache path: " << trt_cache_path
          << ". Set ModelParam.Preprocess.enable_fp16=false to force FP32.";
    nvinfer1::IOptimizationProfile* profile =
        builder_->createOptimizationProfile();
    SetOnnxOptimizationProfile(profile, blobs_);
    config->addOptimizationProfile(profile);

    nvinfer1::IHostMemory* plan =
        builder_->buildSerializedNetwork(*network_, *config);
    if (plan == nullptr) {
      AERROR << "Failed to build serialized network.";
      DestroyConfig(config);
      DestroyParser(parser);
      return false;
    }
    initLibNvInferPlugins(&g_onnx_logger, "");
    if (runtime_ != nullptr) {
      // Runtime must outlive all engines created from it.
      if (engine_ != nullptr) {
        delete engine_;
        engine_ = nullptr;
      }
      delete runtime_;
      runtime_ = nullptr;
    }
    runtime_ = nvinfer1::createInferRuntime(g_onnx_logger);
    engine_ = runtime_->deserializeCudaEngine(plan->data(), plan->size());
    if (engine_ == nullptr) {
      AERROR << "Failed to deserialize engine from serialized network.";
      delete plan;
      DestroyConfig(config);
      DestroyParser(parser);
      return false;
    }
    {
      std::ofstream f(trt_cache_path, std::ios::binary);
      f.write(reinterpret_cast<const char*>(plan->data()), plan->size());
    }
    delete plan;
  }
#else

  if (!LoadCache(trt_load_path)) {
    AINFO << "Building TensorRT engine (" << (use_fp16 ? "FP16" : "FP32")
          << "), cache path: " << trt_cache_path
          << ". Set ModelParam.Preprocess.enable_fp16=false to force FP32.";
    nvinfer1::IOptimizationProfile* profile =
        builder_->createOptimizationProfile();
    SetOnnxOptimizationProfile(profile, blobs_);
    config->addOptimizationProfile(profile);

    engine_ = builder_->buildEngineWithConfig(*network_, *config);
    if (engine_ == nullptr) {
      AERROR << "Failed to build engine with config.";
      DestroyConfig(config);
      DestroyParser(parser);
      return false;
    }

    nvinfer1::IHostMemory* ser_mem = engine_->serialize();
    if (ser_mem != nullptr) {
      std::ofstream f(trt_cache_path, std::ios::binary);
      f.write(reinterpret_cast<const char*>(ser_mem->data()), ser_mem->size());
      ser_mem->destroy();
    }
  } else {
    AINFO << "Loading TensorRT engine from serialized model file: "
          << trt_load_path;
    std::ifstream plan_file(trt_load_path, std::ios::binary);
    if (!plan_file.is_open()) {
      AERROR << "Could not open serialized model: " << trt_load_path;
      DestroyConfig(config);
      DestroyParser(parser);
      return false;
    }

    initLibNvInferPlugins(&g_onnx_logger, "");
    std::stringstream plan_buffer;
    plan_buffer << plan_file.rdbuf();
    std::string plan = plan_buffer.str();
    if (runtime_ != nullptr) {
      runtime_->destroy();
      runtime_ = nullptr;
    }
    runtime_ = nvinfer1::createInferRuntime(g_onnx_logger);
    engine_ =
        runtime_->deserializeCudaEngine(plan.data(), plan.size(), nullptr);
  }
#endif

  DestroyConfig(config);
  DestroyParser(parser);
  return true;
}

void MultiBatchInference::Infer() {
  if (!CudaOk(cudaSetDevice(gpu_id_), "cudaSetDevice")) {
    return;
  }
  cudaStreamSynchronize(stream_);

  CHECK(!input_names_.empty());
  auto input_shape = blobs_[input_names_[0]]->shape();
  int batch_size = input_shape[0];
  if (batch_size == 0) {
    AINFO << "Batch size is zero.";
    return;
  }

#if NV_TENSORRT_MAJOR >= 10
  for (const auto& name : input_names_) {
    auto blob = get_blob(name);
    if (blob == nullptr) continue;
    nvinfer1::Dims dims;
    dims.nbDims = blob->num_axes();
    for (int i = 0; i < dims.nbDims; ++i) {
      dims.d[i] = blob->shape(i);
    }
    if (!context_->setInputShape(name.c_str(), dims)) {
      AERROR << "TensorRT 10: Failed to set input shape for " << name;
      return;
    }
    if (!context_->setTensorAddress(name.c_str(),
                                    const_cast<float*>(blob->gpu_data()))) {
      AERROR << "TensorRT 10: Failed to set input address for " << name;
      return;
    }
  }

  for (const auto& name : output_names_) {
    auto blob = get_blob(name);
    if (blob == nullptr) continue;
    blob->mutable_gpu_data();
    if (!context_->setTensorAddress(name.c_str(), blob->mutable_gpu_data())) {
      AERROR << "TensorRT 10: Failed to set output address for " << name;
      return;
    }
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!context_->enqueueV3(stream_)) {
      AERROR << "TensorRT 10: enqueueV3 failed";
      return;
    }
  }
#else
  for (const auto& name : input_names_) {
    auto blob = get_blob(name);
    if (blob != nullptr) {
      int32_t index = engine_->getBindingIndex(name.c_str());
      batch_size = blob->shape(0);
      buffers_[index] = const_cast<float*>(blob->gpu_data());

      nvinfer1::Dims dims = network_->getInput(0)->getDimensions();
      dims.d[0] = batch_size;
      context_->setOptimizationProfile(0);
      context_->setBindingDimensions(index, dims);
    }
  }

  for (const auto& name : output_names_) {
    auto blob = get_blob(name);
    if (blob != nullptr) {
      blob->mutable_gpu_data();
      int32_t index = engine_->getBindingIndex(name.c_str());
      buffers_[index] = blob->mutable_gpu_data();
    }
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    context_->enqueueV2(&buffers_[0], stream_, nullptr);
  }
#endif

  cudaStreamSynchronize(stream_);
  for (const auto& name : output_names_) {
    auto blob = get_blob(name);
    if (blob != nullptr) {
      blob->mutable_gpu_data();
    }
  }
}

}  // namespace inference
}  // namespace perception
}  // namespace apollo
