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

#include "modules/lidar_fusion/core/tracking/temporal.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/Geometry"
#include "tracking/core/multi_object_tracker.h"
#include "tracking/estimation/predictor.h"

#include "modules/lidar_fusion/core/spatial/spatial.h"
#include "modules/lidar_fusion/core/tracking/temporal_internal.h"

namespace apollo {
namespace lidar_fusion {
using temporal_detail::CellKey;
using temporal_detail::CellState;
using temporal_detail::Measurement;
using temporal_detail::Mot;
using temporal_detail::Set;
using temporal_detail::TemporalPrior;
using temporal_detail::Track;
using temporal_detail::Transform;
using temporal_detail::TransformBox;
using temporal_detail::V;
using temporal_detail::Vector;
using temporal_detail::Vote;
namespace {
namespace tracking = common::tracking;
using Vector = Eigen::Vector3d;

class Estimator final
    : public tracking::StateEstimator<tracking::GaussianState, Measurement> {
 public:
  explicit Estimator(const TemporalOptions& c)
      : predictor_(std::make_shared<tracking::ConstantVelocityModel>(
                       3, c.process_variance()),
                   std::make_shared<tracking::PositionMeasurementModel>(
                       6, 3, c.initial_velocity_variance())) {}
  absl::Status Validate(const Measurement& p) const override {
    return predictor_.Validate(p.position);
  }
  absl::StatusOr<tracking::GaussianState> Initialize(
      const Measurement& p) const override {
    return predictor_.Initialize(p.position);
  }
  absl::Status Predict(double dt, tracking::GaussianState* s) const override {
    return predictor_.Predict(dt, s);
  }
  absl::Status Update(const Measurement& p,
                      tracking::GaussianState* s) const override {
    return predictor_.Update(p.position, s);
  }

 private:
  tracking::Predictor<tracking::VectorMeasurement> predictor_;
};

std::unique_ptr<Mot> MakeTracker(const TemporalOptions& c) {
  tracking::TrackerConfig config;
  config.lifecycle.confirmation_hits = c.confirmation_hits();
  config.lifecycle.lost_timeout_seconds = c.lost_seconds();
  config.lifecycle.publish_tentative = true;
  config.lifecycle.publish_lost = true;
  config.max_tracks = c.max_tracks();
  config.max_observations_per_frame = c.max_tracks();
  config.max_candidate_pairs = c.max_tracks() * c.max_tracks();
  auto cost = std::make_shared<tracking::FunctionalAssociationCost<
      tracking::GaussianState, Measurement>>(
      [gate = c.match_distance()](
          const Track& track,
          const tracking::Observation<Measurement>& observation)
          -> absl::StatusOr<tracking::CostEvaluation> {
        const auto& p = *observation.payload;
        if (p.unknown != track.last_observation->unknown ||
            (p.prior_track != 0 && p.prior_track != track.id)) {
          return tracking::CostEvaluation::Reject(
              "different instance kind/prior");
        }
        const double distance =
            (track.state.mean.head<3>() - p.position.value).norm();
        if (distance > gate) {
          return tracking::CostEvaluation::Reject("center gate");
        }
        const auto& old = track.last_observation->box.size();
        const auto& now = p.box.size();
        double intersection = 1, a = 1, b = 1;
        for (int i = 0; i < 2; ++i) {
          const double sa = i == 0 ? old.x() : old.y();
          const double sb = i == 0 ? now.x() : now.y();
          const double delta =
              std::abs(track.state.mean[i] - p.position.value[i]);
          intersection *=
              std::max(0., std::min(sa * 0.5, delta + sb * 0.5) -
                               std::max(-sa * 0.5, delta - sb * 0.5));
          a *= sa;
          b *= sb;
        }
        const double iou = intersection / std::max(1e-9, a + b - intersection);
        return tracking::CostEvaluation::Accept(distance / gate * 0.7 +
                                                (1 - iou) * 0.3);
      });
  return std::make_unique<Mot>(config, std::make_shared<Estimator>(c), cost,
                               std::make_shared<tracking::HungarianSolver>());
}

bool OptionsValid(const TemporalOptions& c) {
  for (double value :
       {c.pose_tolerance(), c.max_dt(), c.match_distance(),
        c.measurement_variance(), c.initial_velocity_variance(),
        c.process_variance(), c.lost_seconds(), c.history_seconds(),
        c.occupied_log_odds(), c.max_log_odds(), c.dynamic_speed(),
        c.max_pose_translation_step(), c.max_pose_rotation_step()}) {
    if (!std::isfinite(value) || value <= 0) {
      return false;
    }
  }
  return !c.odom_frame().empty() && std::isfinite(c.free_log_odds()) &&
         c.free_log_odds() < 0 && c.confirmation_hits() > 0 &&
         c.class_window() > 0 && c.class_window() <= 100 &&
         c.conflict_frames() > 0 && c.max_tracks() > 0 &&
         c.max_tracks() <= 2048 && c.max_history_cells() > 0 &&
         c.max_history_cells() <= 1000000;
}

bool PoseValid(const FusionConfig& c, const LocalPose* pose, double stamp) {
  if (pose == nullptr || !pose->valid() || !pose->has_epoch() ||
      !pose->has_stamp() || !pose->has_translation() ||
      pose->odom_frame() != c.temporal().odom_frame() ||
      pose->base_frame() != c.base_frame() || !std::isfinite(pose->stamp()) ||
      std::abs(pose->stamp() - stamp) > c.temporal().pose_tolerance()) {
    return false;
  }
  const Eigen::Quaterniond q(pose->qw(), pose->qx(), pose->qy(), pose->qz());
  return V(pose->translation()).allFinite() &&
         V(pose->translation()).cwiseAbs().maxCoeff() < 1000000 &&
         q.coeffs().allFinite() && std::abs(q.norm() - 1) < 1e-5;
}

Box UnknownBox(const UnknownCandidate& unknown) {
  Vector low = Vector::Constant(std::numeric_limits<double>::infinity());
  Vector high = -low;
  for (const auto& p : unknown.polygon()) {
    low = low.cwiseMin(V(p));
    high = high.cwiseMax(V(p));
  }
  low.z() = unknown.z_min();
  high.z() = unknown.z_max();
  Box box;
  Set((low + high) * 0.5, box.mutable_center());
  Set((high - low).cwiseMax(Vector::Constant(0.01)), box.mutable_size());
  return box;
}

}  // namespace

StatefulFusion::StatefulFusion() : impl_(std::make_unique<Impl>()) {}
StatefulFusion::~StatefulFusion() = default;
void StatefulFusion::Reset() { impl_->Reset(); }

bool StatefulFusion::Init(const FusionConfig& c, std::string* error) {
  impl_->Reset();
  impl_->initialized = false;
  if (!impl_->single.Init(c, error)) {
    return false;
  }
  if (c.temporal().enabled() && !OptionsValid(c.temporal())) {
    if (error) {
      *error = "invalid temporal options or missing local ODOM frame";
    }
    return false;
  }
  impl_->config = c;
  impl_->tracker = MakeTracker(c.temporal());
  impl_->initialized = true;
  return true;
}

bool StatefulFusion::Process(const drivers::PointCloud& cloud,
                             const DetectionFrame* detection,
                             const SemanticFrame* segmentation,
                             const LocalPose* pose, FusionOutput* output,
                             std::string* error) {
  auto& state = *impl_;
  if (!state.initialized || output == nullptr) {
    if (error) {
      *error = "stateful fusion not initialized or null output";
    }
    return false;
  }
  const auto& c = state.config.temporal();
  if (!c.enabled()) {
    return state.single.Process(cloud, detection, segmentation, output, error);
  }
  const double now = cloud.measurement_time();
  const bool valid = PoseValid(state.config, pose, now);
  const Eigen::Isometry3d t =
      valid ? Transform(*pose) : Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d delta = state.previous_pose.inverse() * t;
  const bool discontinuity =
      valid && state.active &&
      (delta.translation().norm() > c.max_pose_translation_step() ||
       Eigen::AngleAxisd(delta.rotation()).angle() >
           c.max_pose_rotation_step());
  bool reset = false;
  if (!valid ||
      (state.active &&
       (pose->epoch() != state.epoch || now <= state.stamp ||
        now - state.stamp > c.max_dt() ||
        pose->diagnostic_only() != state.diagnostic || discontinuity))) {
    state.Reset();
    reset = true;
  }
  const TemporalPrior prior = valid ? state.Prior(now, t) : TemporalPrior{};
  if (!state.single.Process(cloud, detection, segmentation, output, error,
                            &prior.ground, &prior.objects)) {
    state.Reset();
    return false;
  }
  auto* health = output->mutable_health();
  health->set_pose_valid(valid);
  if (reset) {
    output->add_risk_event(TEMPORAL_RESET);
  }
  if (!valid) {
    output->add_risk_event(POSE_UNAVAILABLE);
    return true;
  }
  tracking::TrackingFrame<Measurement> frame;
  frame.timestamp = now;
  frame.coordinate_frame = c.odom_frame();
  auto add = [&](Box box, bool unknown, uint32_t id, UnifiedClass cls,
                 bool conflict, ClassificationUse classification_use,
                 Source source, uint64_t prior_track,
                 bool point_supported_footprint) {
    auto p = std::make_shared<Measurement>();
    p->box = TransformBox(box, t);
    p->position.value = V(p->box.center());
    p->position.covariance =
        Eigen::Matrix3d::Identity() * c.measurement_variance();
    p->unknown = unknown;
    p->candidate = id;
    p->cls = cls;
    p->conflict = conflict;
    p->classification_use = classification_use;
    p->point_supported_footprint = point_supported_footprint;
    p->source = source;
    p->prior_track = prior_track;
    frame.observations.push_back({frame.observations.size() + 1, p});
  };
  for (const auto& object : output->object()) {
    add(object.box(), false, object.id(), object.unified_class(),
        object.class_conflict(), object.classification_use(), object.source(),
        object.source() == TRACK_PRIOR ? object.track_id() : 0, false);
  }
  for (const auto& unknown : output->unknown()) {
    if (!unknown.polygon().empty()) {
      add(UnknownBox(unknown), true, unknown.id(), CLASS_UNKNOWN, false,
          IDENTITY_UNCERTAIN, GEOMETRY, 0, unknown.point_supported_footprint());
    }
  }
  const auto result = state.tracker->Update(frame);
  if (!result.ok()) {
    // No partial tracking/maps survive resource or numerical failure. Current
    // immutable single-frame output is still usable in explicit degraded mode.
    state.Reset();
    for (auto& object : *output->mutable_object()) {
      object.clear_track_id();
      object.clear_track_confirmed();
      object.clear_temporal_class();
    }
    for (auto& unknown : *output->mutable_unknown()) {
      unknown.clear_track_id();
      unknown.set_confirmed(false);
    }
    output->add_risk_event(TEMPORAL_RESET);
    return true;
  }
  std::set<uint64_t> live;
  for (const auto& track : result->tracks) {
    live.insert(track.id);
    const auto& p = *track.last_observation;
    auto& vote = state.votes[track.id];
    if (track.observed && p.source != TRACK_PRIOR) {
      vote.conflicts = p.conflict ? vote.conflicts + 1 : 0;
      if (!p.conflict && p.classification_use != IDENTITY_UNCERTAIN) {
        vote.classes.push_back(p.cls);
        while (vote.classes.size() > c.class_window()) {
          vote.classes.pop_front();
        }
        std::map<UnifiedClass, unsigned> counts;
        for (auto cls : vote.classes) {
          ++counts[cls];
        }
        const auto winner = std::max_element(
            counts.begin(), counts.end(),
            [](const auto& a, const auto& b) { return a.second < b.second; });
        if (winner != counts.end() &&
            winner->second * 2 > vote.classes.size()) {
          vote.current = winner->first;
        }
      }
      if (vote.conflicts >= c.conflict_frames()) {
        vote.current = CLASS_UNKNOWN;
      }
    }
    auto* out = output->add_track();
    out->set_id(track.id);
    Box box = p.box;
    Set(track.state.mean.head<3>(), box.mutable_center());
    *out->mutable_box() = TransformBox(box, t.inverse());
    Set(t.rotation().transpose() * track.state.mean.tail<3>(),
        out->mutable_velocity());
    out->set_unified_class(vote.current);
    out->set_confirmed(track.hits >= c.confirmation_hits());
    out->set_observed(track.observed);
    out->set_unknown(p.unknown);
    out->set_last_observed_stamp(track.last_observed_time);
    out->set_source(p.unknown && out->confirmed() ? PROMOTED_UNKNOWN
                                                  : p.source);
    out->set_class_conflict(p.conflict);
    if (!track.observed) {
      continue;
    }
    if (p.unknown) {
      auto* u = output->mutable_unknown(p.candidate - 1);
      u->set_track_id(track.id);
      u->set_confirmed(out->confirmed());
    } else {
      auto* object = output->mutable_object(p.candidate - 1);
      object->set_track_id(track.id);
      object->set_track_confirmed(out->confirmed());
      object->set_temporal_class(vote.current);
      // Raw source velocity remains intact; filtered physical velocity is on
      // track.
    }
  }
  for (auto it = state.votes.begin(); it != state.votes.end();) {
    it = live.count(it->first) ? std::next(it) : state.votes.erase(it);
  }
  state.Map(t, output);
  state.active = true;
  state.stamp = now;
  state.epoch = pose->epoch();
  state.diagnostic = pose->diagnostic_only();
  state.previous_pose = t;
  health->set_temporal_enabled(true);
  health->set_temporal_epoch(state.epoch);
  health->set_diagnostic_pose(state.diagnostic);
  output->set_odom_frame(c.odom_frame());
  CheckOutput(state.config, output);
  return true;
}

}  // namespace lidar_fusion
}  // namespace apollo
