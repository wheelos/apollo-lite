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

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <cuda_runtime_api.h>

#include "modules/perception/pipeline/proto/stage/detection.pb.h"

#include "modules/perception/base/image_8u.h"
#include "modules/perception/base/traffic_light.h"
#include "modules/perception/camera/common/camera_frame.h"
#include "modules/perception/camera/common/data_provider.h"
#include "modules/perception/camera/lib/traffic_light/detector/detection/cropbox.h"
#include "modules/perception/camera/lib/traffic_light/detector/detection/select.h"
#include "modules/perception/inference/onnx_multi_batch/onnx_multi_batch_infer.h"

namespace apollo {
namespace perception {
namespace camera {

struct YoloxPadResizeParam {
  int left_pad = 0;
  int right_pad = 0;
  int top_pad = 0;
  int bottom_pad = 0;
  int roi_width = 0;
  int roi_height = 0;
  float scale = 1.f;
};

class TrafficLightYoloxDetector {
 public:
  TrafficLightYoloxDetector() = default;
  ~TrafficLightYoloxDetector();

  bool Init(const TrafficLightDetectionConfig& config);
  bool Detect(CameraFrame* frame);

 private:
  bool DetectTL(CameraFrame* frame);
  bool GetCandidateHeads(int batch_id, int valid_batch_size,
                         std::vector<base::TrafficLightPtr>* out,
                         std::vector<base::TrafficLightPtr>& lights_ref);
  void NMS(std::vector<base::TrafficLightPtr>* lights,
           const std::string& camera_name);
  void ApplyOverlapNMS(std::vector<base::TrafficLightPtr>* lights);
  void ApplyNMS(std::vector<base::TrafficLightPtr>* lights);

  bool PadAndCopyImage(const base::Image8U& src, base::Image8U* dst,
                       int left_pad, int top_pad, uint8_t pad_value);

 private:
  TrafficLightDetectionConfig config_;
  YoloxTrafficLightDetectionConfig yolox_config_;

  std::shared_ptr<inference::MultiBatchInference> infer_ = nullptr;
  std::vector<std::string> input_names_;
  std::vector<std::string> output_names_;

  std::vector<base::TrafficLightPtr> detected_bboxes_;
  std::map<std::string, std::vector<base::TrafficLightPtr>> detected_heads_;
  std::vector<std::string> detected_light_ids_;
  std::vector<int> detected_light_indices_;
  std::vector<base::RectI> detected_crop_rois_;
  std::vector<YoloxPadResizeParam> pad_resize_params_;

  DataProvider::ImageOptions image_options_;
  Select select_;
  std::shared_ptr<IGetBox> crop_;

  float cls_th_ = 0.24f;
  int resize_height_ = 384;
  int resize_width_ = 384;
  int min_head_area_ = 10;
  int max_batch_roi_ = 3000;

  base::Image8U padding_image_container_;

  int gpu_id_ = 0;
  int max_batch_size_ = 1;
  cudaStream_t stream_ = 0;
  bool owns_stream_ = false;
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
