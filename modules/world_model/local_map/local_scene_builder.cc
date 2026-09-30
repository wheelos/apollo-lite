// Copyright 2026 WheelOS All Rights Reserved.
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

#include "modules/world_model/local_map/local_scene_builder.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "modules/world_model/local_map/drivable_environment.h"
#include "modules/world_model/local_map/lane_synthesis.h"
#include "modules/world_model/local_map/scene_assembly.h"

namespace apollo {
namespace world_model {
namespace {
common::Status Error(const std::string& reason) {
  return common::Status(common::ErrorCode::PLANNING_ERROR,
                        "local scene: " + reason);
}
}  // namespace

LocalSceneBuilder::LocalSceneBuilder(const TemporalLanePolicy& policy,
                                     std::string odom_frame,
                                     std::string body_frame,
                                     std::string clock_id)
    : policy_(policy),
      map_(policy),
      odom_frame_(std::move(odom_frame)),
      body_frame_(std::move(body_frame)),
      clock_id_(std::move(clock_id)) {}

common::Status LocalSceneBuilder::CheckSource(const SceneSource& s, double now,
                                              const std::string& frame,
                                              double max_age) const {
  if (frame.empty() || clock_id_.empty() || s.frame_id != frame ||
      s.clock_id != clock_id_ || s.session.empty() || s.generation == 0 ||
      s.sequence == 0 || s.health != SceneHealth::HEALTHY ||
      !std::isfinite(now) || now < 0.0 || !std::isfinite(s.measurement_time) ||
      s.measurement_time < 0.0 || !std::isfinite(s.publication_time) ||
      !std::isfinite(s.valid_until) ||
      s.measurement_time > s.publication_time || s.publication_time > now ||
      s.publication_time >= s.valid_until || now >= s.valid_until ||
      now - s.measurement_time > max_age) {
    return Error("invalid source frame, clock, identity, health or time");
  }
  return common::Status::OK();
}

bool LocalSceneBuilder::SameEpoch(const SceneSource& s) const {
  return s.session == odometry_.session && s.generation == odometry_.generation;
}

void LocalSceneBuilder::Invalidate(const std::string& reason, bool odometry) {
  invalid_reason_ = reason;
  observed_ = false;
  lane_reason_ = reason;
  map_.ClearLaneHistory();
  if (odometry) {
    armed_ = false;
    environment_observed_ = false;
    environment_frontier_observed_ = false;
    environment_frontier_ = {};
    environment_ = {};
    map_.Invalidate();
  }
}

common::Status LocalSceneBuilder::BeginEpoch(const SceneSource& source,
                                             const RelativePose& initial,
                                             double now, bool stationary) {
  Invalidate("epoch transition", true);
  auto status = CheckSource(source, now, odom_frame_, policy_.max_pose_gap);
  if (!status.ok()) return status;
  if (body_frame_.empty() || body_frame_ == odom_frame_ ||
      initial.time != source.measurement_time) {
    return Error("invalid body frame or initial pose timestamp");
  }
  status =
      map_.BeginEpoch(source.session, source.generation, initial, stationary);
  if (!status.ok()) return status;
  odometry_ = source;
  observation_ = {};
  environment_ = {};
  environment_observed_ = false;
  environment_frontier_ = {};
  environment_frontier_observed_ = false;
  lane_id_ = 0;
  next_boundary_id_ = 1;
  scene_sequence_ = 0;
  armed_ = true;
  lane_reason_ = "no lane observation";
  invalid_reason_ = "waiting for fresh drivable-region evidence";
  return common::Status::OK();
}

common::Status LocalSceneBuilder::AddOdometry(const SceneSource& source,
                                              const RelativePose& pose,
                                              double now) {
  if (!armed_ || !SameEpoch(source))
    return Error("unarmed or cross-epoch ODOM");
  auto status = CheckSource(source, now, odom_frame_, policy_.max_pose_gap);
  if (status.ok() && (source.sequence <= odometry_.sequence ||
                      source.publication_time < odometry_.publication_time ||
                      pose.time != source.measurement_time)) {
    status = Error("regressed ODOM sequence or inconsistent pose time");
  }
  if (status.ok()) {
    status = map_.AddPose(source.session, source.generation, pose);
  }
  if (!status.ok()) {
    Invalidate(status.error_message(), true);
    return status;
  }
  odometry_ = source;
  return common::Status::OK();
}

common::Status LocalSceneBuilder::Observe(const SceneSource& source,
                                          const LaneObservation& observation,
                                          double now) {
  if (!armed_ || !SameEpoch(source))
    return Error("unarmed or cross-epoch lane");
  auto status =
      CheckSource(source, now, body_frame_, policy_.max_observation_age);
  if (status.ok() &&
      (source.session != observation.session ||
       source.generation != observation.generation ||
       source.sequence != observation.sequence ||
       source.measurement_time != observation.measurement_time ||
       source.publication_time < observation_.publication_time)) {
    status = Error("inconsistent lane source provenance");
  }
  if (status.ok()) status = map_.Observe(observation, now);
  if (!status.ok()) {
    Invalidate(status.error_message(), false);
    return status;
  }
  observation_ = source;
  observed_ = true;
  lane_reason_.clear();
  invalid_reason_.clear();
  return common::Status::OK();
}

common::Status LocalSceneBuilder::ObserveLaneAbsence(const SceneSource& source,
                                                     double now) {
  if (!armed_ || !SameEpoch(source))
    return Error("unarmed or cross-epoch lane absence");
  auto status =
      CheckSource(source, now, body_frame_, policy_.max_observation_age);
  if (status.ok() &&
      (source.sequence <= observation_.sequence ||
       source.publication_time < observation_.publication_time)) {
    status = Error("regressed lane-absence sequence or publication time");
  }
  if (!status.ok()) {
    observed_ = false;
    map_.ClearLaneHistory();
    lane_reason_ = status.error_message();
    return status;
  }
  observation_ = source;
  observed_ = false;
  map_.ClearLaneHistory();
  lane_reason_ = "fresh observation reports no usable lane";
  return common::Status::OK();
}

common::Status LocalSceneBuilder::ObserveEnvironment(
    const SceneSource& source, const EnvironmentObservation& observation,
    double now) {
  if (!armed_ || !SameEpoch(source))
    return Error("unarmed or cross-epoch environment");
  auto status =
      CheckSource(source, now, body_frame_, policy_.max_observation_age);
  if (status.ok() && environment_frontier_observed_ &&
      (source.sequence <= environment_frontier_.sequence ||
       source.measurement_time < environment_frontier_.measurement_time ||
       source.publication_time < environment_frontier_.publication_time)) {
    status = Error("regressed environment sequence or source time");
  }
  RelativePose pose;
  if (status.ok()) status = map_.PoseAt(source.measurement_time, &pose);
  DrivableEnvironment environment;
  if (status.ok()) {
    status = BuildDrivableEnvironment(source, observation, pose, odom_frame_,
                                      policy_.max_position_error, &environment);
  }
  if (!status.ok()) {
    environment_observed_ = false;
    environment_ = {};
    invalid_reason_ = status.error_message();
    return status;
  }
  environment_ = std::move(environment);
  environment_observed_ = true;
  environment_frontier_ = source;
  environment_frontier_observed_ = true;
  invalid_reason_.clear();
  return common::Status::OK();
}

common::Status LocalSceneBuilder::Build(double now, LocalScene* scene,
                                        const NavigationPrior* navigation) {
  if (scene == nullptr) return Error("scene output required");
  *scene = {};
  scene->mode = SceneMode::INVALID;
  scene->source.health = SceneHealth::INVALID;
  scene->invalid_reason = invalid_reason_;
  auto status = CheckSource(odometry_, now, odom_frame_, policy_.max_pose_gap);
  if (armed_ && !status.ok()) Invalidate(status.error_message(), true);
  if (!armed_ || !environment_observed_) {
    if (armed_ && !environment_observed_)
      invalid_reason_ = "waiting for fresh drivable-region evidence";
    scene->invalid_reason = invalid_reason_;
    return Error(scene->invalid_reason);
  }
  status = CheckSource(environment_.source, now, odom_frame_,
                       policy_.max_observation_age);
  if (!status.ok()) {
    environment_observed_ = false;
    environment_ = {};
    invalid_reason_ = status.error_message();
    scene->invalid_reason = invalid_reason_;
    return status;
  }

  TemporalLaneSnapshot snapshot;
  bool lane_valid = observed_;
  if (lane_valid) {
    status = CheckSource(observation_, now, body_frame_,
                         policy_.max_observation_age);
    if (status.ok()) status = map_.Snapshot(now, &snapshot);
    if (!status.ok()) {
      observed_ = false;
      map_.ClearLaneHistory();
      lane_reason_ = status.error_message();
      lane_valid = false;
    }
  }

  const SceneSource scene_source = {
      odom_frame_,
      clock_id_,
      odometry_.session,
      odometry_.generation,
      ++scene_sequence_,
      lane_valid ? std::max(snapshot.measurement_time,
                            environment_.source.measurement_time)
                 : environment_.source.measurement_time,
      now,
      lane_valid
          ? std::min({snapshot.valid_until, odometry_.valid_until,
                      observation_.valid_until,
                      environment_.source.valid_until})
          : std::min(odometry_.valid_until, environment_.source.valid_until),
      SceneHealth::HEALTHY};
  LaneLayer lane_layer;
  const LaneLayer* lane_layer_ptr = nullptr;
  if (lane_valid) {
    if (lane_id_ != snapshot.lane_id) {
      lane_id_ = snapshot.lane_id;
      left_id_ = next_boundary_id_++;
      right_id_ = next_boundary_id_++;
    }
    SceneSource lane_source = scene_source;
    lane_source.sequence = snapshot.version;
    lane_source.measurement_time = snapshot.measurement_time;
    status = SynthesizeObservedLane(lane_source, snapshot, policy_, left_id_,
                                    right_id_, &lane_layer);
    if (!status.ok()) {
      *scene = {};
      scene->mode = SceneMode::INVALID;
      scene->source.health = SceneHealth::INVALID;
      scene->invalid_reason = status.error_message();
      return status;
    }
    lane_layer_ptr = &lane_layer;
  }
  status = AssembleLocalScene(scene_source, environment_, lane_layer_ptr,
                              lane_reason_, navigation, now, scene);
  if (!status.ok()) {
    *scene = {};
    scene->mode = SceneMode::INVALID;
    scene->source.health = SceneHealth::INVALID;
    scene->invalid_reason = status.error_message();
    return status;
  }
  return common::Status::OK();
}

}  // namespace world_model
}  // namespace apollo
