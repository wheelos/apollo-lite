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

#include "modules/local_planning/planning_context/input_gate.h"

#include <cmath>
#include <string>

namespace apollo {
namespace local_planning {
namespace {

common::Status Reject(const std::string& reason) {
  return common::Status(common::ErrorCode::PLANNING_ERROR, reason);
}

common::Status CheckFrontier(const SourceStamp& current,
                             const SourceStamp& previous,
                             bool strict_measurement_time) {
  if (current.sequence < previous.sequence) {
    return Reject("source sequence regressed");
  }
  if (current.sequence == previous.sequence) {
    if (!(current == previous)) {
      return Reject("accepted source version mutated");
    }
    return common::Status::OK();
  }
  if (current.measurement_time < previous.measurement_time ||
      (strict_measurement_time &&
       current.measurement_time == previous.measurement_time) ||
      current.publication_time < previous.publication_time) {
    return Reject("source time did not advance with sequence");
  }
  return common::Status::OK();
}

}  // namespace

InputGate::InputGate(const InputPolicy& policy) : policy_(policy) {}

void InputGate::Disarm() { armed_ = false; }

common::Status InputGate::ValidatePolicy() const {
  if (policy_.odom_frame.empty() || policy_.clock_id.empty() ||
      !std::isfinite(policy_.max_odom_age) || policy_.max_odom_age <= 0.0 ||
      !std::isfinite(policy_.max_graph_age) || policy_.max_graph_age <= 0.0 ||
      !std::isfinite(policy_.max_prediction_age) ||
      policy_.max_prediction_age <= 0.0 ||
      !std::isfinite(policy_.stationary_speed_limit) ||
      policy_.stationary_speed_limit < 0.0) {
    return Reject("invalid input admission policy");
  }
  return common::Status::OK();
}

common::Status InputGate::ValidateStamp(const SourceStamp& stamp, double now,
                                        double max_age) const {
  if (stamp.frame_id != policy_.odom_frame ||
      stamp.clock_id != policy_.clock_id) {
    return Reject("frame or clock domain mismatch");
  }
  if (stamp.epoch.producer_session.empty() || stamp.epoch.generation == 0 ||
      stamp.sequence == 0 || stamp.health != InputHealth::HEALTHY) {
    return Reject("missing source identity or unhealthy input");
  }
  if (!std::isfinite(now) || now < 0.0 ||
      !std::isfinite(stamp.measurement_time) || stamp.measurement_time < 0.0 ||
      !std::isfinite(stamp.publication_time) ||
      !std::isfinite(stamp.valid_until) ||
      stamp.measurement_time > stamp.publication_time ||
      stamp.publication_time > now ||
      stamp.publication_time >= stamp.valid_until || now >= stamp.valid_until ||
      now - stamp.measurement_time > max_age) {
    return Reject("invalid, future, expired or stale source time");
  }
  return common::Status::OK();
}

common::Status InputGate::ValidateOdometry(const OdometryInput& input,
                                           double now) const {
  auto status = ValidateStamp(input.stamp, now, policy_.max_odom_age);
  if (!status.ok()) {
    return status;
  }
  if (!std::isfinite(input.x) || !std::isfinite(input.y) ||
      !std::isfinite(input.heading) || !std::isfinite(input.speed_mps)) {
    return Reject("nonfinite odometry state");
  }
  return common::Status::OK();
}

common::Status InputGate::BeginEpoch(const OdometryInput& odometry,
                                     double now) {
  armed_ = false;
  auto status = ValidatePolicy();
  if (!status.ok()) {
    return status;
  }
  status = ValidateOdometry(odometry, now);
  if (!status.ok()) {
    return Reject("epoch start: " + status.error_message());
  }
  if (std::abs(odometry.speed_mps) > policy_.stationary_speed_limit) {
    return Reject("epoch transition requires stationary odometry");
  }
  if (has_epoch_ && (odometry.stamp.epoch.generation <= epoch_.generation ||
                     now < epoch_start_time_ ||
                     (has_cycle_ && now < last_input_.planning_time))) {
    return Reject("epoch generation or transition time regressed");
  }
  epoch_ = odometry.stamp.epoch;
  epoch_start_time_ = now;
  initial_odometry_ = odometry;
  has_epoch_ = true;
  has_cycle_ = false;
  last_input_ = {};
  armed_ = true;
  return common::Status::OK();
}

common::Status InputGate::Admit(const CycleInput& input) {
  if (!armed_) {
    return Reject("ODOM epoch has not been armed");
  }
  if (!std::isfinite(input.planning_time) ||
      input.planning_time < epoch_start_time_ ||
      (has_cycle_ && input.planning_time <= last_input_.planning_time)) {
    return Reject("planning reference time must advance");
  }
  auto status = ValidateOdometry(input.odometry, input.planning_time);
  if (!status.ok()) {
    return Reject("odometry: " + status.error_message());
  }
  status =
      ValidateStamp(input.graph, input.planning_time, policy_.max_graph_age);
  if (!status.ok()) {
    return Reject("graph: " + status.error_message());
  }
  if (has_cycle_ && input.graph.sequence == last_input_.graph.sequence &&
      (!(input.corridor == last_input_.corridor) ||
       !(input.environment == last_input_.environment))) {
    return Reject("accepted graph payload mutated");
  }
  status = ValidateStamp(input.prediction.stamp, input.planning_time,
                         policy_.max_prediction_age);
  if (!status.ok()) {
    return Reject("prediction: " + status.error_message());
  }
  if (has_cycle_ &&
      input.prediction.stamp.sequence ==
          last_input_.prediction.stamp.sequence &&
      !(input.occupancy == last_input_.occupancy)) {
    return Reject("accepted prediction payload mutated");
  }
  if (!(input.odometry.stamp.epoch == epoch_) ||
      !(input.graph.epoch == epoch_) ||
      !(input.prediction.stamp.epoch == epoch_)) {
    return Reject("cross-epoch input");
  }
  if (input.prediction.graph_sequence != input.graph.sequence) {
    return Reject("prediction references a different graph version");
  }
  const auto& previous_odometry =
      has_cycle_ ? last_input_.odometry : initial_odometry_;
  status = CheckFrontier(input.odometry.stamp, previous_odometry.stamp, true);
  if (!status.ok()) {
    return Reject("odometry: " + status.error_message());
  }
  if (input.odometry.stamp.sequence == previous_odometry.stamp.sequence &&
      !(input.odometry == previous_odometry)) {
    return Reject("accepted odometry state mutated");
  }
  if (has_cycle_) {
    status = CheckFrontier(input.graph, last_input_.graph, false);
    if (!status.ok()) {
      return Reject("graph: " + status.error_message());
    }
    status = CheckFrontier(input.prediction.stamp, last_input_.prediction.stamp,
                           false);
    if (!status.ok()) {
      return Reject("prediction: " + status.error_message());
    }
    if (input.prediction.stamp.sequence ==
            last_input_.prediction.stamp.sequence &&
        input.prediction.graph_sequence !=
            last_input_.prediction.graph_sequence) {
      return Reject("accepted prediction graph reference mutated");
    }
  }
  last_input_ = input;
  has_cycle_ = true;
  return common::Status::OK();
}

}  // namespace local_planning
}  // namespace apollo
