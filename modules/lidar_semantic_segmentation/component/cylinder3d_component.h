// Copyright 2026 WheelOS. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <memory>

#include "cyber/component/component.h"
#include "modules/lidar_semantic_segmentation/inference/cylinder3d.h"

namespace apollo {
namespace lidar_semantic_segmentation {

class Cylinder3DComponent final
    : public cyber::Component<apollo::drivers::PointCloud> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<apollo::drivers::PointCloud>& cloud) override;

 private:
  Cylinder3DSegmenter segmenter_;
  std::shared_ptr<cyber::Writer<LidarSemanticSegmentationResult>> writer_;
};

CYBER_REGISTER_COMPONENT(Cylinder3DComponent)

}  // namespace lidar_semantic_segmentation
}  // namespace apollo
