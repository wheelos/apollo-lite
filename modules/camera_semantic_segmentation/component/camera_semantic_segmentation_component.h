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

#pragma once

#include <memory>
#include <string>

#include "wheelos_msgs/sensor_msgs/sensor_image.pb.h"

#include "cyber/component/component.h"
#include "modules/camera_semantic_segmentation/inference/segformer.h"
#include "modules/camera_semantic_segmentation/inference/tensorrt_executor.h"
#include "modules/camera_semantic_segmentation/proto/camera_semantic_segmentation.pb.h"
#include "modules/camera_semantic_segmentation/types/image_frame.h"

namespace apollo {
namespace camera_semantic_segmentation {

class CameraSemanticSegmentationComponent final
    : public cyber::Component<apollo::drivers::Image> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<apollo::drivers::Image>& image) override;

 private:
  bool BuildOptions(const CameraSemanticSegmentationComponentConfig& config,
                    SegFormerModelOptions* options) const;
  bool MakeImageView(const apollo::drivers::Image& message,
                     ImageView* image_view) const;

  CameraSemanticSegmentationComponentConfig config_;
  std::unique_ptr<TensorRtSegFormerExecutor> executor_;
  std::unique_ptr<SegFormerSegmenter> segmenter_;
  std::shared_ptr<cyber::Writer<CameraSemanticSegmentationResult>> writer_;
};

CYBER_REGISTER_COMPONENT(CameraSemanticSegmentationComponent)

}  // namespace camera_semantic_segmentation
}  // namespace apollo
