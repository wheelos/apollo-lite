// Copyright 2026 WheelOS. All Rights Reserved.

#include "modules/lidar_fusion/publication/functional_scene.h"

#include <string>

#include "modules/lidar_fusion/core/alignment/sweep_identity.h"

namespace apollo {
namespace lidar_fusion {
namespace {
bool Fail(const std::string& message, std::string* error) {
  if (error != nullptr) {
    *error = message;
  }
  return false;
}
}  // namespace

bool ProjectFunctionalOutput(const FusionConfig& config,
                             const drivers::PointCloud& cloud,
                             const FusionOutput& full, FusionOutput* output,
                             std::string* error) {
  if (output == nullptr || output == &full ||
      full.publication().detail() != FULL_EVIDENCE || !full.has_health() ||
      !full.header().has_timestamp_sec() ||
      full.header().timestamp_sec() != cloud.measurement_time() ||
      full.point_evidence_size() != cloud.point_size() ||
      full.point_decision_size() != cloud.point_size() ||
      cloud.point_size() == 0 || !SameSweep(cloud, full.header()) ||
      cloud.frame_id() != config.sensor_frame() ||
      full.header().frame_id() != config.base_frame() ||
      full.calibration_version() != config.sensor_to_base().version()) {
    return Fail(
        "functional projection needs the complete matching source frame",
        error);
  }
  FusionOutput heap_scene;
  FusionOutput& scene =
      output->GetArena() == nullptr
          ? heap_scene
          : *google::protobuf::Arena::Create<FusionOutput>(output->GetArena());
  *scene.mutable_header() = full.header();
  scene.set_calibration_version(full.calibration_version());
  if (full.has_detection_model_revision()) {
    scene.set_detection_model_revision(full.detection_model_revision());
  }
  scene.mutable_hypothesis()->CopyFrom(full.hypothesis());
  scene.mutable_object()->CopyFrom(full.object());
  scene.mutable_unknown()->CopyFrom(full.unknown());
  if (full.has_space_map()) {
    *scene.mutable_space_map() = full.space_map();
  }
  if (full.has_health()) {
    *scene.mutable_health() = full.health();
  }
  scene.mutable_risk_event()->CopyFrom(full.risk_event());
  scene.mutable_point_view()->CopyFrom(full.point_view());
  scene.mutable_track()->CopyFrom(full.track());
  scene.mutable_history_cell()->CopyFrom(full.history_cell());
  if (full.has_odom_frame()) {
    scene.set_odom_frame(full.odom_frame());
  }
  scene.mutable_shadow_partition()->CopyFrom(full.shadow_partition());
  auto* context = scene.mutable_publication();
  context->set_detail(FUNCTIONAL_SCENE);
  context->set_source_point_count(cloud.point_size());
  *context->mutable_source_header() = cloud.header();
  context->set_source_measurement_time(cloud.measurement_time());
  context->set_source_pointcloud_channel(config.pointcloud_channel());
  *context->mutable_sensor_to_base() = config.sensor_to_base();
  if (config.has_spatial()) {
    *context->mutable_spatial_options() = config.spatial();
  }
  if (!config.publication().diagnostic_channel().empty()) {
    context->set_diagnostic_channel(config.publication().diagnostic_channel());
  }
  context->set_shadow_partitions_enabled(
      config.candidates().shadow_semantic_partitions() &&
      full.health().seg_valid() &&
      !config.quality().semantic_identity_unverified());
  output->Swap(&scene);
  return true;
}

}  // namespace lidar_fusion
}  // namespace apollo
