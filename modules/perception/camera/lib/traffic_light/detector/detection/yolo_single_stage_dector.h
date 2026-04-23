/******************************************************************************
 * Copyright 2026 The Wheel.OS Authors. All Rights Reserved.
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

#include <memory>
#include <string>
#include <vector>

#include <cuda_runtime_api.h>

#include "modules/perception/pipeline/proto/stage/detection.pb.h"

#include "modules/perception/base/image_8u.h"
#include "modules/perception/base/traffic_light.h"
#include "modules/perception/camera/common/camera_frame.h"
#include "modules/perception/camera/common/data_provider.h"
#include "modules/perception/common/graph/hungarian_optimizer.h"
#include "modules/perception/inference/onnx_multi_batch/onnx_multi_batch_infer.h"

namespace apollo {
namespace perception {
namespace camera {

struct YoloSingleStageDectorLetterBoxParam {
  float scale = 1.0f;
  int resized_width = 0;
  int resized_height = 0;
  int left_pad = 0;
  int right_pad = 0;
  int top_pad = 0;
  int bottom_pad = 0;
  int src_width = 0;
  int src_height = 0;
};

class TrafficLightYoloSingleStageDector {
 public:
  TrafficLightYoloSingleStageDector() = default;
  ~TrafficLightYoloSingleStageDector();

  bool Init(const TrafficLightDetectionConfig& config);
  bool Detect(CameraFrame* frame);

 private:
  bool DetectTL(CameraFrame* frame);
  bool BuildLetterBoxImage(const base::Image8U& src, base::Image8U* dst,
                           YoloSingleStageDectorLetterBoxParam* param) const;
  bool DecodeDetections(const YoloSingleStageDectorLetterBoxParam& param,
                        std::vector<base::TrafficLightPtr>* detections) const;
  void AssociateDetections(std::vector<base::TrafficLightPtr>* hdmap_lights,
                           const std::vector<base::TrafficLightPtr>& detections,
                           int image_width, int image_height);
  void ApplyNMS(std::vector<base::TrafficLightPtr>* detections) const;
  float ComputeAssociationScore(const base::TrafficLightPtr& hdmap_light,
                                const base::TrafficLightPtr& detection,
                                int image_width, int image_height) const;
  base::TLColor ClassIdToColor(int class_id) const;
  base::TLDetectionClass GuessShapeClass(const base::RectI& roi) const;

 private:
  TrafficLightDetectionConfig config_;
  YoloSingleStageDectorTrafficLightDetectionConfig
      yolo_single_stage_dector_config_;

  std::shared_ptr<inference::MultiBatchInference> infer_ = nullptr;
  std::vector<std::string> input_names_;
  std::vector<std::string> output_names_;

  DataProvider::ImageOptions image_options_;
  std::shared_ptr<base::Blob<float>> input_blob_ = nullptr;
  std::shared_ptr<base::Blob<float>> output_blob_ = nullptr;

  common::HungarianOptimizer<float> association_optimizer_;

  float conf_threshold_ = 0.25f;
  float iou_threshold_ = 0.45f;
  float scale_ = 0.00392157f;
  float distance_weight_ = 0.15f;
  float min_box_area_ = 10.0f;
  int resize_height_ = 640;
  int resize_width_ = 640;
  int num_classes_ = 3;
  int num_predictions_ = 8400;
  int pad_value_ = 114;
  bool is_bgr_ = false;
  int green_class_id_ = 0;
  int red_class_id_ = 1;
  int yellow_class_id_ = 2;
  int gpu_id_ = 0;

  cudaStream_t stream_ = 0;
  bool owns_stream_ = false;
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
