// Copyright 2026 WheelOS. All Rights Reserved.

#include "modules/lidar_fusion/core/alignment/input_validation.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <set>
#include <vector>

#include "modules/lidar_fusion/core/alignment/sweep_identity.h"

namespace apollo {
namespace lidar_fusion {
namespace alignment_internal {

bool Probability(double value) {
  return std::isfinite(value) && value >= 0.0 && value <= 1.0;
}

std::map<uint32_t, ClassMapping> Mappings(
    const google::protobuf::RepeatedPtrField<ClassMapping>& entries) {
  std::map<uint32_t, ClassMapping> result;
  for (const auto& entry : entries) {
    result.emplace(entry.raw_id(), entry);
  }
  return result;
}

bool ValidMappings(
    const google::protobuf::RepeatedPtrField<ClassMapping>& entries,
    bool detection) {
  std::set<uint32_t> seen;
  for (const auto& entry : entries) {
    if (!entry.has_raw_id() || !entry.has_group() ||
        !seen.insert(entry.raw_id()).second ||
        (detection && entry.group() != THING && entry.group() != GENERIC_OBS) ||
        (entry.group() == THING && entry.unified_class() == CLASS_UNKNOWN)) {
      return false;
    }
  }
  return !entries.empty();
}

Reason ValidateDetection(const drivers::PointCloud& cloud,
                         const DetectionFrame* input, const FusionConfig& c) {
  if (input == nullptr) {
    return SOURCE_MISSING;
  }
  if (!input->has_header() || !SameSweep(cloud, input->header()) ||
      input->sensor_frame_id() != c.sensor_frame()) {
    return SOURCE_MISALIGNED;
  }
  if (input->backend() != c.detection_backend() ||
      input->model_revision().empty() ||
      input->detection_size() > static_cast<int>(c.max_detections())) {
    return SOURCE_INVALID;
  }
  const auto mappings = Mappings(c.detection_mapping());
  for (const auto& det : input->detection()) {
    const std::array<double, 9> values = {
        det.center_x(), det.center_y(),   det.center_z(),
        det.size_x(),   det.size_y(),     det.size_z(),
        det.yaw(),      det.velocity_x(), det.velocity_y()};
    if (!std::all_of(values.begin(), values.end(),
                     [](double v) { return std::isfinite(v); }) ||
        !det.has_center_x() || !det.has_center_y() || !det.has_center_z() ||
        !det.has_yaw() || !det.has_score() || !det.has_class_id() ||
        det.size_x() <= 0 || det.size_y() <= 0 || det.size_z() <= 0 ||
        det.size_x() > 100 || det.size_y() > 100 || det.size_z() > 100 ||
        std::abs(det.center_x()) > 10000 || std::abs(det.center_y()) > 10000 ||
        std::abs(det.center_z()) > 10000 || !Probability(det.score()) ||
        det.class_id() < 0 || mappings.count(det.class_id()) == 0 ||
        det.has_velocity_x() != det.has_velocity_y()) {
      return SOURCE_INVALID;
    }
  }
  return NO_REASON;
}

Reason ValidateSemantic(const drivers::PointCloud& cloud,
                        const SemanticFrame* input, const FusionConfig& c) {
  if (input == nullptr) {
    return SOURCE_MISSING;
  }
  if (!input->has_header() || !SameSweep(cloud, input->header()) ||
      input->source_frame_id() != c.sensor_frame() ||
      input->source_topic() != c.pointcloud_channel()) {
    return SOURCE_MISALIGNED;
  }
  if (input->point_count() != static_cast<uint32_t>(cloud.point_size()) ||
      input->point_label_size() != cloud.point_size()) {
    return SOURCE_INVALID;
  }
  const auto mappings = Mappings(c.semantic_mapping());
  std::vector<bool> seen(cloud.point_size(), false);
  for (const auto& label : input->point_label()) {
    if (!label.has_point_index() || !label.has_semantic_label() ||
        !label.has_confidence() || label.point_index() >= seen.size() ||
        seen[label.point_index()] || !Probability(label.confidence()) ||
        mappings.count(label.semantic_label()) == 0) {
      return SOURCE_INVALID;
    }
    seen[label.point_index()] = true;
  }
  return NO_REASON;
}

}  // namespace alignment_internal
}  // namespace lidar_fusion
}  // namespace apollo
