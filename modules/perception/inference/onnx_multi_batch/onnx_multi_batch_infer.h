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

#pragma once

#include <NvInfer.h>
#include <NvInferPlugin.h>
#include <NvInferRuntime.h>
#include <NvInferRuntimeCommon.h>
#include <NvInferVersion.h>
#include <NvOnnxParser.h>

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <cuda_runtime_api.h>

#include "modules/perception/inference/inference.h"

namespace apollo {
namespace perception {
namespace inference {

class MultiBatchInference : public Inference {
 public:
  MultiBatchInference() = default;
  ~MultiBatchInference() override;

  void set_model_info(const std::string& onnx_file,
                      const std::vector<std::string>& input_names,
                      const std::vector<std::string>& output_names);

  void set_enable_fp16(bool enable_fp16) { enable_fp16_ = enable_fp16; }

  bool Init(const std::map<std::string, std::vector<int>>& shapes) override;
  void Infer() override;
  std::shared_ptr<apollo::perception::base::Blob<float>> get_blob(
      const std::string& name) override;

  void SetStream(cudaStream_t stream);

 private:
  void FileInit();
  bool LoadCache(const std::string& path) const;
  bool OnnxToTRTModel(const std::string& model_file);
  void SetOnnxOptimizationProfile(nvinfer1::IOptimizationProfile* profile,
                                  const BlobMap& input_shapes);

 private:
  std::string onnx_file_;
  std::vector<std::string> net_output_names_;
  std::vector<std::string> net_input_names_;

  std::string model_file_;
  std::vector<std::string> output_names_;
  std::vector<std::string> input_names_;

  BlobMap blobs_;

  std::mutex mutex_;
  nvinfer1::INetworkDefinition* network_ = nullptr;
  // TensorRT runtime must outlive all engines deserialized from it.
  // Keep it as a member so that ~MultiBatchInference destroys it after engine_.
  nvinfer1::IRuntime* runtime_ = nullptr;
  nvinfer1::ICudaEngine* engine_ = nullptr;
  nvinfer1::IExecutionContext* context_ = nullptr;
  nvinfer1::IBuilder* builder_ = nullptr;

  bool enable_fp16_ = true;

  std::vector<void*> buffers_;
  cudaStream_t stream_ = 0;
  bool owns_stream_ = false;
};

}  // namespace inference
}  // namespace perception
}  // namespace apollo
