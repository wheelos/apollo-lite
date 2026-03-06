/******************************************************************************
 * Copyright 2026 The Wheel.OS Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <cuda_runtime_api.h>

#include "modules/perception/pipeline/proto/stage/recognition.pb.h"

#include "modules/perception/base/image_8u.h"
#include "modules/perception/base/traffic_light.h"
#include "modules/perception/camera/common/camera_frame.h"
#include "modules/perception/camera/common/data_provider.h"
#include "modules/perception/inference/onnx_multi_batch/onnx_multi_batch_infer.h"

namespace apollo {
namespace perception {
namespace camera {

class TrafficLightEfficientNetRecognizer {
 public:
  TrafficLightEfficientNetRecognizer() = default;
  ~TrafficLightEfficientNetRecognizer();

  bool Init(const TrafficLightRecognitionConfig& config);
  bool Recognize(CameraFrame* frame);

 private:
  void Perform(const CameraFrame* frame,
               std::vector<base::TrafficLightPtr>* tl);
  void ZeroPadding(const base::RectI& roi, base::Image8U* padded,
                   int* dst_width, int* dst_height, int provider_w,
                   int provider_h);
  void Prob2Color(const float* output_data, float threshold,
                  const base::TrafficLightPtr& light);
  bool PadAndCopyImage(const base::Image8U& src, base::Image8U* dst,
                       int left_pad, int top_pad, uint8_t pad_value);

 private:
  TrafficLightRecognitionConfig config_;
  EfficientNetRecognitionConfig eff_cfg_;

  std::shared_ptr<inference::MultiBatchInference> infer_ = nullptr;
  std::shared_ptr<base::Blob<float>> input_blob_ = nullptr;
  std::shared_ptr<base::Blob<float>> outputs_cls_ = nullptr;
  std::shared_ptr<base::Blob<float>> outputs_status_ = nullptr;

  DataProvider::ImageOptions image_options_;
  std::shared_ptr<base::Image8U> image_ = nullptr;

  float unknown_threshold_ = 0.5f;
  float scale_ = 0.00392157f;
  float mean_[3] = {0.0f, 0.0f, 0.0f};  // B, G, R in BGR mode
  bool is_bgr_ = true;
  int resize_height_ = 96;
  int resize_width_ = 96;
  int max_batch_size_ = 3;
  int gpu_id_ = 0;

  cudaStream_t stream_ = 0;
  bool owns_stream_ = false;
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
