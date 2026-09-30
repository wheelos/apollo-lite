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

#include "modules/world_model/local_map/temporal_lane_map.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "modules/world_model/local_map/lane_association.h"

namespace apollo {
namespace world_model {
namespace {
common::Status Error(const std::string& reason) {
  return common::Status(common::ErrorCode::PLANNING_ERROR,
                        "temporal lane map: " + reason);
}
bool Finite(const LanePoint& p) {
  return std::isfinite(p.x) && std::isfinite(p.y) && std::abs(p.x) < 1e5 &&
         std::abs(p.y) < 1e5;
}
double Distance(const LanePoint& a, const LanePoint& b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}
double Angle(double a) { return std::atan2(std::sin(a), std::cos(a)); }
LanePoint Mix(const LanePoint& a, const LanePoint& b, double w) {
  return {a.x * (1.0 - w) + b.x * w, a.y * (1.0 - w) + b.y * w};
}
bool ValidPose(const RelativePose& p, double bound) {
  return Finite({p.x, p.y}) && std::isfinite(p.time) && p.time >= 0.0 &&
         std::isfinite(p.heading) && std::isfinite(p.position_error) &&
         p.position_error >= 0.0 && p.position_error <= bound;
}
LanePoint Transform(const LanePoint& p, const RelativePose& pose) {
  return {pose.x + std::cos(pose.heading) * p.x - std::sin(pose.heading) * p.y,
          pose.y + std::sin(pose.heading) * p.x + std::cos(pose.heading) * p.y};
}
}  // namespace

TemporalLaneMap::TemporalLaneMap(const TemporalLanePolicy& policy)
    : policy_(policy) {}

void TemporalLaneMap::Invalidate() {
  armed_ = false;
  poses_.clear();
  ClearLaneHistory();
}

void TemporalLaneMap::ClearLaneHistory() {
  samples_.clear();
  observed_end_ = {};
}

common::Status TemporalLaneMap::BeginEpoch(const std::string& session,
                                           uint64_t generation,
                                           const RelativePose& initial,
                                           bool stationary) {
  Invalidate();
  for (double value :
       {policy_.grid_spacing, policy_.history_seconds, policy_.max_pose_gap,
        policy_.max_observation_age, policy_.sample_lifetime,
        policy_.publication_lifetime, policy_.association_distance,
        policy_.min_overlap, policy_.min_width, policy_.max_width,
        policy_.max_position_error, policy_.fresh_weight}) {
    if (!std::isfinite(value) || value <= 0.0) {
      return Error("invalid policy");
    }
  }
  if (policy_.fresh_weight > 1.0 || policy_.grid_spacing < 0.05 ||
      policy_.sample_lifetime <= policy_.publication_lifetime ||
      policy_.min_width >= policy_.max_width || !stationary ||
      session.empty() || generation == 0 || generation <= generation_ ||
      !ValidPose(initial, policy_.max_position_error) ||
      initial.time < last_activity_) {
    return Error("invalid policy or nonstationary/reused epoch");
  }
  session_ = session;
  generation_ = generation;
  last_sequence_ = 0;
  version_ = 0;
  lane_id_ = 0;
  last_observation_ = -1.0;
  last_snapshot_ = -1.0;
  poses_.push_back(initial);
  last_activity_ = initial.time;
  armed_ = true;
  return common::Status::OK();
}

common::Status TemporalLaneMap::AddPose(const std::string& session,
                                        uint64_t generation,
                                        const RelativePose& pose) {
  if (!armed_ || session != session_ || generation != generation_ ||
      !ValidPose(pose, policy_.max_position_error) ||
      pose.time <= poses_.back().time) {
    return Error("invalid, cross-epoch or regressed ODOM");
  }
  poses_.push_back(pose);
  last_activity_ = std::max(last_activity_, pose.time);
  while (poses_.size() > 2 &&
         (pose.time - poses_[1].time > policy_.history_seconds ||
          poses_.size() > 10000)) {
    poses_.pop_front();
  }
  return common::Status::OK();
}

common::Status TemporalLaneMap::PoseAt(double time, RelativePose* pose) const {
  if (pose == nullptr) return Error("pose output required");
  for (size_t i = 0; i < poses_.size(); ++i) {
    if (poses_[i].time == time) {
      *pose = poses_[i];
      return common::Status::OK();
    }
    if (i > 0 && poses_[i - 1].time < time && time < poses_[i].time) {
      const auto& a = poses_[i - 1];
      const auto& b = poses_[i];
      if (b.time - a.time > policy_.max_pose_gap) {
        return Error("ODOM interpolation gap too large");
      }
      const double w = (time - a.time) / (b.time - a.time);
      *pose = {time, a.x + w * (b.x - a.x), a.y + w * (b.y - a.y),
               a.heading + w * Angle(b.heading - a.heading),
               std::max(a.position_error, b.position_error)};
      return common::Status::OK();
    }
  }
  return Error("no measurement-time ODOM; extrapolation forbidden");
}

common::Status TemporalLaneMap::Observe(const LaneObservation& o, double now) {
  if (!armed_ || o.session != session_ || o.generation != generation_ ||
      !std::isfinite(now) || !std::isfinite(o.measurement_time) ||
      o.measurement_time > now ||
      now - o.measurement_time > policy_.max_observation_age ||
      o.sequence <= last_sequence_ || o.measurement_time <= last_observation_ ||
      !std::isfinite(o.position_error) || o.position_error < 0.0 ||
      o.position_error > policy_.max_position_error ||
      o.left.size() != o.right.size() || o.left.size() < 3 ||
      o.left.size() > 2000 || !o.forward_confirmed) {
    return Error("invalid, stale, unordered or unconfirmed observation");
  }
  RelativePose pose;
  auto status = PoseAt(o.measurement_time, &pose);
  if (!status.ok()) return status;
  const double error = o.position_error + pose.position_error;
  if (error > policy_.max_position_error) return Error("uncertainty too large");
  std::vector<LanePoint> left, right, center;
  for (size_t i = 0; i < o.left.size(); ++i) {
    if (!Finite(o.left[i]) || !Finite(o.right[i]))
      return Error("nonfinite geometry");
    left.push_back(Transform(o.left[i], pose));
    right.push_back(Transform(o.right[i], pose));
    center.push_back(Mix(left.back(), right.back(), 0.5));
    const double width = Distance(left.back(), right.back());
    if (width < policy_.min_width || width > policy_.max_width) {
      samples_.clear();
      return Error("restrictive or invalid lane width; history invalidated");
    }
  }
  LanePoint anchor = anchor_;
  double heading = anchor_heading_;
  const bool first = samples_.empty();
  if (first) {
    anchor = center.front();
    heading =
        std::atan2(center.back().y - anchor.y, center.back().x - anchor.x);
  }
  std::vector<double> u;
  for (size_t i = 0; i < center.size(); ++i) {
    u.push_back((center[i].x - anchor.x) * std::cos(heading) +
                (center[i].y - anchor.y) * std::sin(heading));
    if (i > 0) {
      const double dx = center[i].x - center[i - 1].x;
      const double dy = center[i].y - center[i - 1].y;
      const double length = std::hypot(dx, dy);
      const double cross =
          dx * (left[i].y - right[i].y) - dy * (left[i].x - right[i].x);
      if (u[i] - u[i - 1] < 0.05 || length > 2.0 || cross <= 0.0 ||
          std::abs(Angle(std::atan2(dy, dx) - heading)) > 1.2) {
        return Error("reversed, crossing or unsupported boundary pairing");
      }
    }
  }
  if (u.back() - u.front() > 300.0 || std::abs(u.front()) > 1000.0) {
    return Error("local map extent exceeded");
  }
  std::map<int, TrackedLaneSample> fresh;
  size_t segment = 1;
  const int start =
      static_cast<int>(std::ceil(u.front() / policy_.grid_spacing));
  const int end = static_cast<int>(std::floor(u.back() / policy_.grid_spacing));
  for (int key = start; key <= end; ++key) {
    const double target = key * policy_.grid_spacing;
    while (segment + 1 < u.size() && u[segment] < target) ++segment;
    const double w = (target - u[segment - 1]) / (u[segment] - u[segment - 1]);
    fresh[key] = {Mix(left[segment - 1], left[segment], w),
                  Mix(right[segment - 1], right[segment], w),
                  o.measurement_time, error};
  }
  if (fresh.size() < 3) return Error("insufficient resampled geometry");
  LaneAssociationResult association;
  status = AssociateLaneSamples(samples_, fresh, policy_, now, &association);
  if (!status.ok()) {
    samples_.clear();
    return Error(status.error_message());
  }
  const auto& terminal = association.samples.at(end);
  const auto& raw_terminal = fresh.at(end);
  const TrackedLaneSample observed_end{
      {left.back().x + terminal.left.x - raw_terminal.left.x,
       left.back().y + terminal.left.y - raw_terminal.left.y},
      {right.back().x + terminal.right.x - raw_terminal.right.x,
       right.back().y + terminal.right.y - raw_terminal.right.y},
      o.measurement_time,
      terminal.position_error};
  anchor_ = anchor;
  anchor_heading_ = heading;
  if (first) ++lane_id_;
  for (const auto& entry : association.samples)
    samples_[entry.first] = entry.second;
  for (auto it = samples_.begin(); it != samples_.end();) {
    if (now - it->second.last_observed >= policy_.sample_lifetime ||
        it->first < start - 40 || it->first > end + 40) {
      it = samples_.erase(it);
    } else {
      ++it;
    }
  }
  last_sequence_ = o.sequence;
  last_observation_ = o.measurement_time;
  last_activity_ = std::max(last_activity_, now);
  observed_end_ = observed_end;
  return common::Status::OK();
}

common::Status TemporalLaneMap::Snapshot(double now,
                                         TemporalLaneSnapshot* out) {
  if (out == nullptr) return Error("snapshot output required");
  *out = {};
  if (!armed_ || !std::isfinite(now) || now < last_snapshot_ ||
      last_observation_ < 0.0 || now < last_observation_ ||
      now - last_observation_ > policy_.max_observation_age) {
    return Error("unavailable or expired temporal map");
  }
  TemporalLaneSnapshot snapshot;
  snapshot.session = session_;
  snapshot.generation = generation_;
  snapshot.version = ++version_;
  snapshot.lane_id = lane_id_;
  snapshot.measurement_time = last_observation_;
  snapshot.valid_until =
      std::min(now + policy_.publication_lifetime,
               last_observation_ + policy_.max_observation_age);
  int previous = 0;
  for (const auto& entry : samples_) {
    if (entry.second.last_observed + policy_.sample_lifetime <=
        now + policy_.publication_lifetime)
      continue;
    if (!snapshot.samples.empty() && entry.first != previous + 1) {
      return Error("expired geometry creates unsupported map gap");
    }
    snapshot.samples.push_back(entry.second);
    previous = entry.first;
  }
  if (snapshot.samples.size() < 3 || snapshot.valid_until <= now) {
    return Error("insufficient fresh map coverage");
  }
  // Grid association must not quantize the observed stopping boundary. Keep
  // the measured terminal segment, with the adjacent track's fusion offset.
  if (observed_end_.last_observed + policy_.sample_lifetime >
      now + policy_.publication_lifetime) {
    const auto endpoint = Mix(observed_end_.left, observed_end_.right, 0.5);
    const auto& last = snapshot.samples.back();
    const auto center = Mix(last.left, last.right, 0.5);
    const double forward = (endpoint.x - center.x) * std::cos(anchor_heading_) +
                           (endpoint.y - center.y) * std::sin(anchor_heading_);
    if (forward > 0.0) {
      if (Distance(endpoint, center) < policy_.grid_spacing)
        snapshot.samples.pop_back();
      snapshot.samples.push_back(observed_end_);
    }
  }
  last_snapshot_ = now;
  last_activity_ = std::max(last_activity_, now);
  *out = std::move(snapshot);
  return common::Status::OK();
}
}  // namespace world_model
}  // namespace apollo
