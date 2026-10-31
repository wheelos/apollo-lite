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

#include "modules/lidar_fusion/core/runtime/single_frame_fusion.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <numeric>
#include <queue>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "Eigen/Geometry"
#include "Eigen/QR"
#include "google/protobuf/arena.h"

#include "modules/lidar_fusion/core/alignment/input_validation.h"
#include "modules/lidar_fusion/core/alignment/sweep_identity.h"
#include "modules/lidar_fusion/core/association/build_candidates.h"
#include "modules/lidar_fusion/core/association/candidates.h"
#include "modules/lidar_fusion/core/association/detection_association.h"
#include "modules/lidar_fusion/core/association/prior_association.h"
#include "modules/lidar_fusion/core/evidence/build_evidence.h"
#include "modules/lidar_fusion/core/evidence/point_geometry.h"
#include "modules/lidar_fusion/core/spatial/connected_voxels.h"
#include "modules/lidar_fusion/core/spatial/occupancy.h"
#include "modules/lidar_fusion/core/spatial/spatial.h"

namespace apollo {
namespace lidar_fusion {
using alignment_internal::Probability;
using alignment_internal::ValidateDetection;
using alignment_internal::ValidateSemantic;
using alignment_internal::ValidMappings;
using fusion_detail::Associate;
using fusion_detail::AssociatePrior;
using fusion_detail::BuildCandidates;
using fusion_detail::BuildEvidence;
using fusion_detail::BuildSpace;
using fusion_detail::ConnectedVoxels;
using fusion_detail::InRoi;
using fusion_detail::ObstacleHeight;
using fusion_detail::SpatialIndex;
namespace {

bool Fail(const std::string& message, std::string* error) {
  if (error != nullptr) {
    *error = message;
  }
  return false;
}

using Detection = lidar_object_detection::LidarObjectDetection;

// Connected voxels, not all point pairs: dense returns do not create an O(N^2)
// neighborhood search. Ground/noise points cannot bridge obstacle components.

}  // namespace

bool SingleFrameFusion::Init(const FusionConfig& c, std::string* error) {
  initialized_ = false;
  const auto& t = c.sensor_to_base();
  const std::array<double, 38> finite_values = {
      t.x(),
      t.y(),
      t.z(),
      t.roll(),
      t.pitch(),
      t.yaw(),
      c.wait_ms(),
      c.resolution(),
      c.min_x(),
      c.max_x(),
      c.min_y(),
      c.max_y(),
      c.ground_tile_size(),
      c.expected_ground_z(),
      c.ground_seed_band(),
      c.ground_max_slope(),
      c.ground_max_residual(),
      c.ground_tolerance(),
      c.obstacle_min_height(),
      c.clearance_height(),
      c.corridor_min_x(),
      c.corridor_max_x(),
      c.corridor_half_width(),
      c.semantic_min_confidence(),
      c.noise_corridor_confidence(),
      c.noise_outside_confidence(),
      c.association_margin(),
      c.connectivity_size(),
      c.support_ratio(),
      c.conflict_ratio(),
      c.evidence_score_min(),
      c.unsupported_score_min(),
      c.recall_min_size(),
      c.recall_max_size(),
      static_cast<double>(c.ground_min_points()),
      static_cast<double>(c.core_min_points()),
      static_cast<double>(c.unknown_min_points()),
      static_cast<double>(c.recall_min_points())};
  if (!std::all_of(finite_values.begin(), finite_values.end(),
                   [](double v) { return std::isfinite(v); }) ||
      c.pointcloud_channel().empty() || c.detection_channel().empty() ||
      c.segmentation_channel().empty() || c.output_channel().empty() ||
      std::set<std::string>{c.pointcloud_channel(), c.detection_channel(),
                            c.segmentation_channel(), c.output_channel()}
              .size() != 4 ||
      c.sensor_frame().empty() || c.base_frame().empty() ||
      !c.has_sensor_to_base() || t.version().empty() || !t.has_x() ||
      !t.has_y() || !t.has_z() || !t.has_roll() || !t.has_pitch() ||
      !t.has_yaw() ||
      std::max({std::abs(t.x()), std::abs(t.y()), std::abs(t.z())}) > 100 ||
      c.resolution() < 0.05 || c.connectivity_size() < 0.05 ||
      c.ground_tile_size() < c.resolution() || c.max_x() <= c.min_x() ||
      c.max_y() <= c.min_y() ||
      std::max({std::abs(c.min_x()), std::abs(c.max_x()), std::abs(c.min_y()),
                std::abs(c.max_y())}) > 1000 ||
      c.max_points() == 0 || c.max_points() > 1000000 ||
      c.max_detections() == 0 || c.max_detections() > 4096 ||
      c.queue_capacity() == 0 || c.queue_capacity() > 64 || c.wait_ms() <= 0 ||
      c.wait_ms() > 10000 || c.ground_seed_band() <= 0 ||
      c.ground_min_points() < 3 || c.ground_max_slope() < 0 ||
      !Probability(c.quality().ground_inlier_ratio()) ||
      c.quality().ground_inlier_ratio() <= 0.5 ||
      c.ground_max_residual() <= 0 || c.ground_tolerance() <= 0 ||
      c.obstacle_min_height() <= c.ground_tolerance() ||
      c.clearance_height() <= c.obstacle_min_height() ||
      c.corridor_max_x() < c.corridor_min_x() || c.corridor_half_width() < 0 ||
      c.association_margin() < 0 || c.core_min_points() == 0 ||
      c.unknown_min_points() == 0 || c.recall_min_points() == 0 ||
      c.recall_min_size() <= 0 || c.recall_max_size() < c.recall_min_size() ||
      !Probability(c.semantic_min_confidence()) ||
      !Probability(c.noise_corridor_confidence()) ||
      !Probability(c.noise_outside_confidence()) ||
      c.noise_corridor_confidence() < c.noise_outside_confidence() ||
      !Probability(c.support_ratio()) || !Probability(c.conflict_ratio()) ||
      c.support_ratio() < 0.5 || c.conflict_ratio() < 0.5 ||
      c.support_ratio() + c.conflict_ratio() <= 1 ||
      !Probability(c.evidence_score_min()) ||
      !Probability(c.unsupported_score_min()) ||
      !Probability(c.dedup_point_overlap()) || c.dedup_point_overlap() <= 0 ||
      !Probability(c.dedup_bev_iou()) || c.dedup_bev_iou() <= 0 ||
      c.unsupported_score_min() < c.evidence_score_min() ||
      c.detection_backend() ==
          lidar_object_detection::LIDAR_OBJECT_DETECTION_BACKEND_UNSPECIFIED ||
      !ValidMappings(c.detection_mapping(), true) ||
      !ValidMappings(c.semantic_mapping(), false)) {
    return Fail("invalid fusion configuration/calibration/class mappings",
                error);
  }
  const double cells = std::ceil((c.max_x() - c.min_x()) / c.resolution()) *
                       std::ceil((c.max_y() - c.min_y()) / c.resolution());
  // Match the index origin to the published grid, avoiding half-cell drift.
  const auto on_grid = [&](double value) {
    return std::abs(value / c.resolution() -
                    std::round(value / c.resolution())) < 1e-6;
  };
  if (cells > 1000000 || !on_grid(c.min_x()) || !on_grid(c.min_y())) {
    return Fail("grid exceeds one million cells or has an unaligned origin",
                error);
  }
  if (!ValidateSpatial(c, error)) {
    return false;
  }
  if (!ValidateCandidates(c)) {
    return Fail("invalid candidate options", error);
  }
  if (c.publication().transport() == CORE_POD &&
      (c.publication().max_pod_frame_bytes() == 0 ||
       c.publication().max_pod_frame_bytes() >
           static_cast<uint32_t>(std::numeric_limits<int>::max()))) {
    return Fail("CORE_POD frame byte limit must be in [1, INT_MAX]", error);
  }
  const auto& diagnostic = c.publication().diagnostic_channel();
  if (!diagnostic.empty() &&
      (c.publication().detail() != FUNCTIONAL_SCENE ||
       diagnostic == c.pointcloud_channel() ||
       diagnostic == c.detection_channel() ||
       diagnostic == c.segmentation_channel() ||
       diagnostic == c.output_channel() ||
       (c.temporal().enabled() && diagnostic == c.temporal().pose_channel()))) {
    return Fail(
        "diagnostic channel needs FUNCTIONAL_SCENE and a distinct topic",
        error);
  }
  config_ = c;
  initialized_ = true;
  return true;
}

bool SingleFrameFusion::Process(const drivers::PointCloud& cloud,
                                const DetectionFrame* detection,
                                const SemanticFrame* segmentation,
                                FusionOutput* output, std::string* error,
                                const GroundPrior* ground_prior,
                                const ObjectPrior* object_prior,
                                FusionWork* work) const {
  if (work != nullptr) {
    *work = FusionWork{};
  }
  if (!initialized_ || output == nullptr) {
    return Fail("fusion is not initialized or output is null", error);
  }
  output->Clear();
  if (cloud.point_size() == 0 ||
      cloud.point_size() > static_cast<int>(config_.max_points()) ||
      !cloud.has_measurement_time() ||
      !std::isfinite(cloud.measurement_time()) ||
      cloud.measurement_time() <= 0 ||
      cloud.frame_id() != config_.sensor_frame() ||
      (cloud.header().has_frame_id() &&
       cloud.header().frame_id() != config_.sensor_frame()) ||
      !SameSweep(cloud, cloud.header())) {
    return Fail(
        "cloud needs a valid source identity, frame and measurement time",
        error);
  }
  const auto start = std::chrono::steady_clock::now();
  auto last_stage = start;
  auto stage = [&](std::size_t id) {
    if (work != nullptr) {
      const auto now = std::chrono::steady_clock::now();
      work->stage_ms[id] =
          std::chrono::duration<double, std::milli>(now - last_stage).count();
      last_stage = now;
    }
  };
  FusionOutput heap_frame;
  FusionOutput& frame =
      output->GetArena() == nullptr
          ? heap_frame
          : *google::protobuf::Arena::Create<FusionOutput>(output->GetArena());
  *frame.mutable_header() = cloud.header();
  frame.mutable_header()->set_timestamp_sec(cloud.measurement_time());
  frame.mutable_header()->set_frame_id(config_.base_frame());
  frame.mutable_header()->set_module_name("lidar_fusion");
  frame.set_calibration_version(config_.sensor_to_base().version());
  auto* health = frame.mutable_health();
  health->set_det_reason(ValidateDetection(cloud, detection, config_));
  health->set_seg_reason(ValidateSemantic(cloud, segmentation, config_));
  health->set_det_valid(health->det_reason() == NO_REASON);
  health->set_seg_valid(health->seg_reason() == NO_REASON);
  health->set_mode(health->det_valid()
                       ? (health->seg_valid() ? NORMAL : NO_SEG)
                       : (health->seg_valid() ? NO_DET : GEO_ONLY));
  if (!health->det_valid()) {
    detection = nullptr;
    frame.add_risk_event(health->det_reason());
  } else {
    frame.set_detection_model_revision(detection->model_revision());
  }
  if (!health->seg_valid()) {
    segmentation = nullptr;
    frame.add_risk_event(health->seg_reason());
  }
  std::vector<Eigen::Vector3d> positions;
  SpatialIndex index;
  BuildEvidence(cloud, segmentation, config_, &positions, &index, &frame,
                ground_prior);
  stage(0);
  std::vector<bool> eligible(positions.size(), false);
  for (int i = 0; i < frame.point_evidence_size(); ++i) {
    const auto& p = frame.point_evidence(i);
    eligible[i] = p.finite() && InRoi(positions[i], config_) &&
                  ObstacleHeight(p, config_) && !p.is_ground_geo() &&
                  frame.point_decision(i).explanation() != EXPL_NOISE;
  }
  const ConnectedVoxels voxels(positions, eligible,
                               config_.connectivity_size());
  const auto components = voxels.Components(eligible);
  voxels.Count(work);
  stage(1);
  const auto boxes =
      Associate(detection, config_, positions, index, components, &frame);
  AssociatePrior(object_prior, config_, &frame);
  stage(2);
  CandidateResult candidates;
  candidates.objects.Swap(frame.mutable_object());
  candidates.unknowns.Swap(frame.mutable_unknown());
  BuildCandidates(positions, components, voxels, config_,
                  frame.point_evidence(), frame.mutable_point_decision(),
                  &candidates);
  stage(3);
  CandidateWork relation_work;
  RefineCandidates(config_, frame.point_evidence(), &candidates,
                   work == nullptr ? nullptr : &relation_work, &index);
  candidates.objects.Swap(frame.mutable_object());
  candidates.unknowns.Swap(frame.mutable_unknown());
  if (work != nullptr) {
    work->relation_pairs = relation_work.pairs;
    work->relation_point_transforms = relation_work.point_transforms;
  }
  stage(4);
  // Space derives from immutable evidence, never from instance retention.
  auto cells = BuildSpace(config_, frame, boxes, frame.mutable_space_map());
  stage(5);
  EnrichSpatial(config_, &frame, &cells);
  stage(6);
  CheckOutput(config_, &frame, &cells);
  cells.Publish(frame.mutable_space_map());
  if (config_.candidates().shadow_semantic_partitions() &&
      health->seg_valid() &&
      !config_.quality().semantic_identity_unverified()) {
    PartitionUnknowns(config_, frame.point_evidence(), frame.unknown(),
                      frame.mutable_shadow_partition());
  }
  stage(7);
  health->set_latency_ms(std::chrono::duration<double, std::milli>(
                             std::chrono::steady_clock::now() - start)
                             .count());
  *output = std::move(frame);
  return true;
}

}  // namespace lidar_fusion
}  // namespace apollo
