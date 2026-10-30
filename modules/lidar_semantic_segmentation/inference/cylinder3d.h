// Copyright 2026 WheelOS. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "modules/lidar_semantic_segmentation/proto/lidar_semantic_segmentation.pb.h"

#include "modules/lidar_semantic_segmentation/types/cylinder_projection.h"

namespace apollo {
namespace lidar_semantic_segmentation {

struct Cylinder3DOptions {
  std::string model_path;
  int device_id = -1;  // -1: CPU reference; >=0: CUDA device.
  std::size_t max_points = 150000;
  float intensity_scale = 1.0F / 255.0F;
  bool flip_tta = false;
  bool use_spconv = false;
  std::string source_topic;
  std::string sensor_name;
};

class Cylinder3DExecutor {
 public:
  Cylinder3DExecutor();
  ~Cylinder3DExecutor();
  bool Init(const Cylinder3DOptions& options,
            CylinderProjectionOptions* projection, std::string* error);
  bool Run(const CylinderInput& input, std::vector<float>* point_logits,
           std::string* error);
  // One or four votes with identical point_indices; results follow projected
  // row order, not the original cloud's possibly non-contiguous point indices.
  bool RunLabels(const std::vector<CylinderInput>& votes,
                 std::vector<uint32_t>* labels, std::vector<float>* confidences,
                 std::string* error);

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

class Cylinder3DSegmenter {
 public:
  bool Init(const Cylinder3DOptions& options, std::string* error);
  bool Segment(const apollo::drivers::PointCloud& cloud,
               LidarSemanticSegmentationResult* result, std::string* error);

 private:
  Cylinder3DOptions options_;
  CylinderProjector projector_;
  Cylinder3DExecutor executor_;
  bool initialized_ = false;
};

}  // namespace lidar_semantic_segmentation
}  // namespace apollo
