// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "modules/localization/map/particle_relocalizer.h"

namespace apollo {
namespace localization {
namespace unified {

struct HypothesisTrackPolicy {
  size_t maximum_tracks = 0;
  uint32_t minimum_consistent_frames = 0;
  double minimum_consistency_span = 0.0;
  double maximum_unobserved_gap = 0.0;
  double anchor_translation_gate = 0.0;
  double anchor_rotation_gate = 0.0;
  double association_translation_gate = 0.0;
  double association_rotation_gate = 0.0;
};

struct TrackedHypothesis {
  uint64_t id = 0;
  std::string support_region_id;
  Eigen::Isometry3d map_to_odom_anchor = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d last_map_to_odom = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d map_base = Eigen::Isometry3d::Identity();
  double search_mass = 0.0;
  double first_time = 0.0;
  double last_time = 0.0;
  uint64_t last_scan_sequence = 0;
  uint32_t consistent_frames = 0;
  bool observed_this_update = false;
  bool consistency_span_satisfied = false;
  // Stability is a diagnostic signal, never permission to publish a transform.
  bool diagnostic_only = true;
};

struct HypothesisTrackingSnapshot {
  FieldIdentity identity;
  Epoch epoch;
  Stamp stamp;
  uint64_t scan_sequence = 0;
  bool diagnostic_only = true;
  bool handover_eligible = false;
  bool modes_truncated = false;
  double unreported_mass = 0.0;
  std::vector<TrackedHypothesis> tracks;
};

// Persistent geometric consistency only; frame counts are not independent
// evidence and no conversion to GlobalObservation/TF is exposed.
class ParticleHypothesisTracker {
 public:
  explicit ParticleHypothesisTracker(const HypothesisTrackPolicy& policy)
      : policy_(policy) {}
  Result Update(const ParticleSnapshot& particles, const LocalState& local,
                HypothesisTrackingSnapshot* output);
  void Reset();

 private:
  HypothesisTrackPolicy policy_;
  FieldIdentity identity_;
  Epoch epoch_;
  std::string source_;
  double last_time_ = 0.0;
  uint64_t last_scan_sequence_ = 0;
  uint64_t next_id_ = 1;
  std::vector<TrackedHypothesis> tracks_;
};

}  // namespace unified
}  // namespace localization
}  // namespace apollo
