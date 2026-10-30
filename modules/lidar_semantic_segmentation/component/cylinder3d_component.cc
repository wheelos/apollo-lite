// Copyright 2026 WheelOS. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "modules/lidar_semantic_segmentation/component/cylinder3d_component.h"

#include <memory>
#include <string>

#include "cyber/common/log.h"

namespace apollo {
namespace lidar_semantic_segmentation {

bool Cylinder3DComponent::Init() {
  Cylinder3DComponentConfig config;
  if (!GetProtoConfig(&config) || config.model_path().empty() ||
      config.source_topic().empty() || config.output_channel().empty() ||
      config.sensor_name().empty()) {
    AERROR
        << "Cylinder3D requires model_path, source_topic, output_channel and "
           "sensor_name";
    return false;
  }
  Cylinder3DOptions options;
  options.model_path = config.model_path();
  options.device_id = config.device_id();
  options.source_topic = config.source_topic();
  options.sensor_name = config.sensor_name();
  options.max_points = config.max_points();
  options.intensity_scale = config.intensity_scale();
  options.flip_tta = config.flip_tta();
  options.use_spconv = config.use_spconv();
  std::string error;
  if (!segmenter_.Init(options, &error)) {
    AERROR << "Cylinder3D initialization failed: " << error;
    return false;
  }
  writer_ = node_->CreateWriter<LidarSemanticSegmentationResult>(
      config.output_channel());
  return writer_ != nullptr;
}

bool Cylinder3DComponent::Proc(
    const std::shared_ptr<apollo::drivers::PointCloud>& cloud) {
  if (!cloud || !writer_) {
    return false;
  }
  auto output = std::make_shared<LidarSemanticSegmentationResult>();
  std::string error;
  if (!segmenter_.Segment(*cloud, output.get(), &error)) {
    AERROR << "Cylinder3D inference failed; no result published: " << error;
    return false;
  }
  return writer_->Write(output);
}

}  // namespace lidar_semantic_segmentation
}  // namespace apollo
