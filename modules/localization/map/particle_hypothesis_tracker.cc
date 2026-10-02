// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "modules/localization/map/particle_hypothesis_tracker.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <utility>

namespace apollo {
namespace localization {
namespace unified {
namespace {

double RotationDistance(const Eigen::Isometry3d& a,
                        const Eigen::Isometry3d& b) {
  return LogRotation(Eigen::Quaterniond(
      a.linear().transpose() * b.linear())).norm();
}

bool ValidPose(const Eigen::Isometry3d& pose) {
  const Eigen::Quaterniond q(pose.linear());
  return pose.matrix().allFinite() && q.coeffs().allFinite() &&
         std::abs(q.norm() - 1.0) <= 1e-8 &&
         (pose.linear().transpose() * pose.linear())
             .isApprox(Eigen::Matrix3d::Identity(), 1e-8) &&
         std::abs(pose.linear().determinant() - 1.0) <= 1e-8;
}

bool Within(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b,
            double translation_gate, double rotation_gate) {
  return (a.translation() - b.translation()).norm() <= translation_gate &&
         RotationDistance(a, b) <= rotation_gate;
}

bool Positive(double value) { return std::isfinite(value) && value > 0.0; }

}  // namespace

Result ParticleHypothesisTracker::Update(
    const ParticleSnapshot& particles, const LocalState& local,
    HypothesisTrackingSnapshot* output) {
  if (output == nullptr || !particles.diagnostic_only ||
      particles.identity.map_id.empty() ||
      particles.identity.version.empty() ||
      particles.identity.frame.empty() ||
      particles.identity.calibration.empty() ||
      particles.epoch.session.empty() || particles.epoch.generation == 0 ||
      !local.valid || local.epoch != particles.epoch ||
      local.stamp.clock_id.empty() || local.stamp.sequence == 0 ||
      !std::isfinite(local.stamp.receive_time) ||
      !local.position.allFinite() ||
      local.position.cwiseAbs().maxCoeff() > 1e6 ||
      !local.orientation.coeffs().allFinite() ||
      std::abs(local.orientation.norm() - 1.0) > 1e-9 ||
      local.stamp.time != particles.stamp.time ||
      local.stamp.sequence != particles.stamp.sequence ||
      local.stamp.clock_id != particles.stamp.clock_id ||
      particles.scan_sequence == 0 || particles.source.empty() ||
      !std::isfinite(particles.stamp.time) || particles.stamp.time <= 0.0 ||
      particles.modes.empty() || particles.modes_truncated ||
      !std::isfinite(particles.unreported_mass) ||
      particles.unreported_mass < 0.0 ||
      particles.unreported_mass > 1e-9) {
    return {Reason::INVALID_INPUT,
            "Hypothesis tracker requires complete diagnostic modes and exact local time"};
  }
  if (policy_.maximum_tracks == 0 || policy_.maximum_tracks > 256 ||
      policy_.minimum_consistent_frames == 0 ||
      !Positive(policy_.minimum_consistency_span) ||
      !Positive(policy_.maximum_unobserved_gap) ||
      !Positive(policy_.anchor_translation_gate) ||
      !Positive(policy_.anchor_rotation_gate) ||
      !Positive(policy_.association_translation_gate) ||
      !Positive(policy_.association_rotation_gate) ||
      policy_.anchor_rotation_gate > 3.14159265358979323846 ||
      policy_.association_rotation_gate > 3.14159265358979323846) {
    return {Reason::CONFIG_INVALID, "Invalid bounded hypothesis tracking policy"};
  }
  const bool initialized = !identity_.map_id.empty();
  if (initialized &&
      (!(identity_ == particles.identity) || epoch_ != particles.epoch ||
       source_ != particles.source)) {
    return {Reason::MAP_MISMATCH,
            "Map, epoch or source change requires explicit tracker reset"};
  }
  if (initialized &&
      (particles.stamp.time <= last_time_ ||
       particles.scan_sequence <= last_scan_sequence_)) {
    return {Reason::TIMESTAMP_REGRESSION,
            "Hypothesis evidence must be a distinct advancing scan"};
  }
  uint64_t next_id = next_id_;
  auto next_tracks = tracks_;
  next_tracks.erase(
      std::remove_if(next_tracks.begin(), next_tracks.end(),
          [&](const TrackedHypothesis& track) {
            return particles.stamp.time - track.last_time >
                   policy_.maximum_unobserved_gap;
          }),
      next_tracks.end());
  for (auto& track : next_tracks) {
    track.observed_this_update = false;
  }
  std::vector<size_t> mode_order(particles.modes.size());
  for (size_t i = 0; i < mode_order.size(); ++i) {
    mode_order[i] = i;
  }
  std::stable_sort(mode_order.begin(), mode_order.end(),
      [&particles](size_t a, size_t b) {
        return particles.modes[a].search_mass >
               particles.modes[b].search_mass;
      });
  std::vector<bool> used(next_tracks.size(), false);
  for (const size_t mode_index : mode_order) {
    const auto& mode = particles.modes[mode_index];
    if (mode.support_region_id.empty() || !ValidPose(mode.pose) ||
        !std::isfinite(mode.search_mass) || mode.search_mass <= 0.0 ||
        mode.search_mass > 1.0) {
      return {Reason::INVALID_INPUT, "Invalid particle mode for tracking"};
    }
  }
  const double total_mass = std::accumulate(
      particles.modes.begin(), particles.modes.end(), 0.0,
      [](double sum, const ParticleMode& mode) {
        return sum + mode.search_mass;
      });
  if (!std::isfinite(total_mass) || total_mass < 1.0 - 1e-9 ||
      total_mass > 1.0 + 1e-9) {
    return {Reason::INVALID_INPUT,
            "Complete particle modes must account for all search mass"};
  }
  for (const size_t mode_index : mode_order) {
    const auto& mode = particles.modes[mode_index];
    const Eigen::Isometry3d alignment =
        mode.pose * Pose(local).inverse();
    size_t best = next_tracks.size();
    double best_cost = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < next_tracks.size(); ++i) {
      const auto& track = next_tracks[i];
      if (used[i] || track.support_region_id != mode.support_region_id ||
          !Within(alignment, track.map_to_odom_anchor,
                  policy_.anchor_translation_gate,
                  policy_.anchor_rotation_gate) ||
          !Within(alignment, track.last_map_to_odom,
                  policy_.association_translation_gate,
                  policy_.association_rotation_gate)) {
        continue;
      }
      const double cost =
          (alignment.translation() -
           track.last_map_to_odom.translation()).squaredNorm() +
          std::pow(RotationDistance(alignment, track.last_map_to_odom), 2);
      if (cost < best_cost) {
        best = i;
        best_cost = cost;
      }
    }
    if (best == next_tracks.size()) {
      if (next_tracks.size() >= policy_.maximum_tracks ||
          next_id == std::numeric_limits<uint64_t>::max()) {
        return {Reason::INVALID_INPUT, "Hypothesis track capacity exhausted"};
      }
      TrackedHypothesis track;
      track.id = next_id++;
      track.support_region_id = mode.support_region_id;
      track.map_to_odom_anchor = alignment;
      track.last_map_to_odom = alignment;
      track.map_base = mode.pose;
      track.search_mass = mode.search_mass;
      track.first_time = particles.stamp.time;
      track.last_time = particles.stamp.time;
      track.last_scan_sequence = particles.scan_sequence;
      track.consistent_frames = 1;
      track.observed_this_update = true;
      track.consistency_span_satisfied =
          track.consistent_frames >= policy_.minimum_consistent_frames &&
          track.last_time - track.first_time >=
              policy_.minimum_consistency_span;
      next_tracks.push_back(std::move(track));
      used.push_back(true);
      continue;
    }
    used[best] = true;
    auto& track = next_tracks[best];
    if (track.consistent_frames < std::numeric_limits<uint32_t>::max()) {
      ++track.consistent_frames;
    }
    track.last_time = particles.stamp.time;
    track.last_scan_sequence = particles.scan_sequence;
    track.last_map_to_odom = alignment;
    track.map_base = mode.pose;
    track.search_mass = mode.search_mass;
    track.observed_this_update = true;
    track.consistency_span_satisfied =
        track.consistent_frames >= policy_.minimum_consistent_frames &&
        track.last_time - track.first_time >=
            policy_.minimum_consistency_span;
  }
  std::stable_sort(next_tracks.begin(), next_tracks.end(),
      [](const TrackedHypothesis& a, const TrackedHypothesis& b) {
        if (a.observed_this_update != b.observed_this_update) {
          return a.observed_this_update;
        }
        if (a.search_mass != b.search_mass) {
          return a.search_mass > b.search_mass;
        }
        return a.id < b.id;
      });
  HypothesisTrackingSnapshot snapshot;
  snapshot.identity = particles.identity;
  snapshot.epoch = particles.epoch;
  snapshot.stamp = particles.stamp;
  snapshot.scan_sequence = particles.scan_sequence;
  snapshot.modes_truncated = particles.modes_truncated;
  snapshot.unreported_mass = particles.unreported_mass;
  snapshot.tracks = std::move(next_tracks);
  tracks_ = snapshot.tracks;
  identity_ = particles.identity;
  epoch_ = particles.epoch;
  source_ = particles.source;
  next_id_ = next_id;
  last_time_ = particles.stamp.time;
  last_scan_sequence_ = particles.scan_sequence;
  *output = std::move(snapshot);
  return {};
}

void ParticleHypothesisTracker::Reset() {
  identity_ = FieldIdentity();
  epoch_ = Epoch();
  source_.clear();
  last_time_ = 0.0;
  last_scan_sequence_ = 0;
  next_id_ = 1;
  tracks_.clear();
}

}  // namespace unified
}  // namespace localization
}  // namespace apollo
