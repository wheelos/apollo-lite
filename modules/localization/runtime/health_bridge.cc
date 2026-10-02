// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "modules/localization/runtime/health_bridge.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <utility>

#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/localization/common/rigid_transform_helper.h"

namespace apollo {
namespace localization {
namespace unified {
namespace {

bool Positive(double value) { return std::isfinite(value) && value > 0.0; }

bool Fresh(double stamp, double now, double age, double future) {
  return Positive(stamp) && std::isfinite(now) &&
         now - stamp <= age && stamp - now <= future;
}

}  // namespace

LocalizationHealthBridge::~LocalizationHealthBridge() {
  if (timer_) {
    timer_->Stop();
  }
}

bool LocalizationHealthBridge::Init() {
  if (!GetProtoConfig(&config_) || !config_.IsInitialized() ||
      !Positive(config_.local_timeout()) || !Positive(config_.global_timeout()) ||
      !Positive(config_.health_period_ms()) || !Positive(config_.history_duration()) ||
      !Positive(config_.interpolation_gap()) ||
      !Positive(config_.max_local_position_std()) ||
      !Positive(config_.max_local_attitude_std()) ||
      !Positive(config_.max_local_velocity_std()) ||
      !std::isfinite(config_.future_tolerance()) || config_.future_tolerance() < 0.0 ||
      config_.odom_frame().empty() || config_.base_frame().empty() ||
      config_.odom_frame() == config_.base_frame() || config_.clock_id().empty()) {
    AERROR << "CONFIG_INVALID: health bridge timing/identity";
    return false;
  }
  if (config_.enable_lane_evidence()) {
    if (!config_.has_lane_calibration_id() || config_.lane_calibration_id().empty() ||
        !config_.has_boundary_std() || !Positive(config_.boundary_std()) ||
        !config_.has_heading_std() || !Positive(config_.heading_std()) ||
        !config_.has_vehicle_front() || !Positive(config_.vehicle_front()) ||
        !config_.has_vehicle_rear() || !Positive(config_.vehicle_rear()) ||
        !config_.has_vehicle_half_width() || !Positive(config_.vehicle_half_width()) ||
        !config_.has_clearance() || !std::isfinite(config_.clearance()) ||
        config_.clearance() < 0.0 || !Positive(config_.lane_timeout()) ||
        !Positive(config_.uncertainty_multiplier()) ||
        !Positive(config_.minimum_lane_width()) ||
        !Positive(config_.maximum_lane_width()) ||
        config_.maximum_lane_width() < config_.minimum_lane_width() ||
        !Positive(config_.minimum_confidence()) ||
        config_.minimum_confidence() > 1.0 ||
        !config_.has_relative_position_std_per_second() ||
        !Positive(config_.relative_position_std_per_second()) ||
        !config_.has_relative_heading_std_per_second() ||
        !Positive(config_.relative_heading_std_per_second()) ||
        !Positive(config_.max_lane_lateral_std()) ||
        !Positive(config_.max_lane_heading_std()) ||
        !std::isfinite(config_.lane_reference_distance()) ||
        config_.lane_reference_distance() < 0.0) {
      AERROR << "CONFIG_INVALID: lane evidence needs measured calibration/error budgets";
      return false;
    }
    lane_policy_.front = config_.vehicle_front();
    lane_policy_.rear = config_.vehicle_rear();
    lane_policy_.half_width = config_.vehicle_half_width();
    lane_policy_.clearance = config_.clearance();
    lane_policy_.boundary_std = config_.boundary_std();
    lane_policy_.heading_std = config_.heading_std();
    lane_policy_.sigma_multiplier = config_.uncertainty_multiplier();
    lane_policy_.minimum_width = config_.minimum_lane_width();
    lane_policy_.maximum_width = config_.maximum_lane_width();
    lane_policy_.reference_x = config_.lane_reference_distance();
  }
  decoding_config_.set_odom_frame(config_.odom_frame());
  decoding_config_.set_base_frame(config_.base_frame());
  decoding_config_.set_clock_id(config_.clock_id());
  decoding_config_.set_local_timeout(config_.local_timeout());
  decoding_config_.set_future_tolerance(config_.future_tolerance());
  pose_writer_ = node_->CreateWriter<LocalizationEstimate>(config_.health_pose_topic());
  assessment_writer_ =
      node_->CreateWriter<apollo::localization::LocalizationAssessment>(
          config_.health_assessment_topic());
  lane_writer_ = node_->CreateWriter<LaneRelativeState>(config_.lane_relative_topic());
  global_reader_ = node_->CreateReader<GlobalLocalization>(
      config_.global_topic(), [this](const auto& msg) { OnGlobal(msg); });
  local_assessment_reader_ = node_->CreateReader<LocalizationAssessment>(
      config_.local_assessment_topic(), [this](const auto& msg) { OnLocalAssessment(msg); });
  global_assessment_reader_ = node_->CreateReader<LocalizationAssessment>(
      config_.global_assessment_topic(), [this](const auto& msg) { OnGlobalAssessment(msg); });
  if (!pose_writer_ || !assessment_writer_ || !lane_writer_ || !global_reader_ ||
      !local_assessment_reader_ || !global_assessment_reader_) {
    AERROR << "FAULT: health bridge IO";
    return false;
  }
  if (config_.enable_lane_evidence()) {
    lanes_reader_ = node_->CreateReader<perception::PerceptionLanes>(
        config_.lane_topic(), [this](const auto& msg) { OnLanes(msg); });
    if (!lanes_reader_) {
      AERROR << "FAULT: lane evidence reader";
      return false;
    }
  }
  timer_ = std::make_unique<cyber::Timer>(
      config_.health_period_ms(), [this]() { Tick(); }, false);
  timer_->Start();
  return true;
}

bool LocalizationHealthBridge::Proc(const std::shared_ptr<LocalOdometry>& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  const double now = cyber::Clock::NowInSeconds();
  LocalState local;
  const Result decoded =
      message ? DecodeLocal(*message, decoding_config_, now, &local)
              : Result{Reason::INVALID_INPUT, "null ODOM"};
  if (!decoded.ok()) {
    AERROR << ReasonName(decoded.reason) << ": " << decoded.message;
    latest_.set_valid(false);
    return false;
  }
  if (retired_sessions_.count(local.epoch.session) != 0) {
    AERROR << "EPOCH_MISMATCH: retired health ODOM producer";
    return false;
  }
  if (!history_.empty()) {
    const auto& previous = history_.back();
    if (local.stamp.time <= previous.stamp.time ||
        (local.epoch == previous.epoch &&
         local.stamp.sequence <= previous.stamp.sequence) ||
        (local.epoch.session == previous.epoch.session &&
         local.epoch.generation < previous.epoch.generation)) {
      AERROR << "TIMESTAMP_REGRESSION: health ODOM frontier";
      return false;
    }
    if (local.epoch != previous.epoch) {
      if (local.epoch.session != previous.epoch.session) {
        if (retired_sessions_.size() >= 128) {
          AERROR << "EPOCH_MISMATCH: health retired-session capacity exhausted";
          return false;
        }
        retired_sessions_.insert(previous.epoch.session);
      }
      history_.clear();
      global_.Clear();
      local_assessment_.Clear();
      global_assessment_.Clear();
      lane_time_ = 0.0;
      lane_frontier_ = 0.0;
      lane_result_ = {Reason::EPOCH_MISMATCH, "new ODOM epoch needs new lanes"};
    }
  }
  history_.push_back(local);
  while (history_.size() > 1 &&
         (history_.size() > 4000 ||
          local.stamp.time - history_.front().stamp.time > config_.history_duration())) {
    history_.pop_front();
  }
  latest_ = *message;
  LocalizationEstimate pose = message->localization();
  // This stream contains actual new ODOM only; the timer never republishes poses.
  pose.mutable_header()->set_timestamp_sec(local.stamp.time);
  pose.mutable_header()->set_sequence_num(static_cast<uint32_t>(local.stamp.sequence));
  if (!pose_writer_->Write(pose)) {
    AERROR << "FAULT: independent-checker ODOM publication";
    return false;
  }
  return true;
}

void LocalizationHealthBridge::OnGlobal(
    const std::shared_ptr<GlobalLocalization>& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!message || !message->IsInitialized()) {
    AERROR << "INVALID_INPUT: incomplete global health evidence";
    global_.Clear();
    return;
  }
  if (global_.IsInitialized() &&
      message->localization().measurement_time() <=
          global_.localization().measurement_time()) {
    AERROR << "TIMESTAMP_REGRESSION: global health evidence";
    return;
  }
  global_ = *message;
}

void LocalizationHealthBridge::OnLocalAssessment(
    const std::shared_ptr<LocalizationAssessment>& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!message || !message->IsInitialized() || message->owner() != "local") {
    AERROR << "INVALID_INPUT: local self-assessment";
    local_assessment_.Clear();
    return;
  }
  if (local_assessment_.IsInitialized() &&
      message->session_id() == local_assessment_.session_id() &&
      message->sequence() <= local_assessment_.sequence()) {
    AERROR << "TIMESTAMP_REGRESSION: local self-assessment";
    return;
  }
  local_assessment_ = *message;
}

void LocalizationHealthBridge::OnGlobalAssessment(
    const std::shared_ptr<LocalizationAssessment>& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!message || !message->IsInitialized() || message->owner() != "global") {
    AERROR << "INVALID_INPUT: global self-assessment";
    global_assessment_.Clear();
    return;
  }
  if (global_assessment_.IsInitialized() &&
      message->session_id() == global_assessment_.session_id() &&
      message->sequence() <= global_assessment_.sequence()) {
    AERROR << "TIMESTAMP_REGRESSION: global self-assessment";
    return;
  }
  global_assessment_ = *message;
}

Result LocalizationHealthBridge::Lookup(double time, LocalState* state) const {
  if (state == nullptr || !std::isfinite(time) || history_.empty() ||
      time < history_.front().stamp.time || time > history_.back().stamp.time) {
    return {Reason::HISTORY_UNAVAILABLE, "lane time has no local bracket"};
  }
  auto after = std::lower_bound(
      history_.begin(), history_.end(), time,
      [](const LocalState& item, double t) { return item.stamp.time < t; });
  if (after->stamp.time == time) {
    *state = *after;
    return {};
  }
  const auto& before = *std::prev(after);
  const double span = after->stamp.time - before.stamp.time;
  if (span > config_.interpolation_gap()) {
    return {Reason::HISTORY_UNAVAILABLE, "lane local bracket too wide"};
  }
  const double fraction = (time - before.stamp.time) / span;
  *state = before;
  state->covariance_model_valid =
      before.covariance_model_valid && after->covariance_model_valid;
  state->stamp.time = time;
  state->position = (1.0 - fraction) * before.position + fraction * after->position;
  state->orientation = before.orientation.slerp(fraction, after->orientation);
  state->covariance = 2.0 * ((1.0 - fraction) * before.covariance +
                             fraction * after->covariance);
  return {};
}

void LocalizationHealthBridge::OnLanes(
    const std::shared_ptr<perception::PerceptionLanes>& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  const double now = cyber::Clock::NowInSeconds();
  const double previous_lane_time = lane_time_;
  lane_result_ = {Reason::INVALID_INPUT, "invalid or ambiguous measured lanes"};
  lane_time_ = 0.0;
  if (!message || !message->has_header() ||
      !message->header().has_timestamp_sec() ||
      !Fresh(message->header().timestamp_sec(), now,
             config_.lane_timeout(), config_.future_tolerance()) ||
      message->header().timestamp_sec() <= lane_frontier_ ||
      message->error_code() != perception::camera::ERROR_NONE ||
      !message->has_source_topic() || message->source_topic().empty() ||
      message->header().frame_id().empty() || message->camera_laneline_size() > 16) {
    AERROR << lane_result_.message;
    return;
  }
  lane_frontier_ = message->header().timestamp_sec();
  LocalState observation_local;
  const auto lookup = Lookup(lane_frontier_, &observation_local);
  if (!lookup.ok()) {
    lane_result_ = lookup;
    AERROR << lookup.message;
    return;
  }
  Eigen::Affine3d extrinsic;
  if (!apollo::localization::common::LookupStaticTransform(
          config_.base_frame(), message->header().frame_id(), &extrinsic, 0.0f) ||
      !extrinsic.matrix().allFinite()) {
    AERROR << "INVALID_INPUT: lane camera static extrinsic unavailable";
    return;
  }
  LaneGeometry geometry;
  for (const auto& line : message->camera_laneline()) {
    if (!line.has_pos_type() ||
        (line.pos_type() != perception::camera::EGO_LEFT &&
         line.pos_type() != perception::camera::EGO_RIGHT)) {
      continue;
    }
    auto& points = line.pos_type() == perception::camera::EGO_LEFT
                       ? geometry.left : geometry.right;
    if (!points.empty() || !line.has_confidence() ||
        !std::isfinite(line.confidence()) ||
        line.confidence() < config_.minimum_confidence() ||
        line.confidence() > 1.0 || !line.has_use_type() ||
        line.use_type() != perception::camera::REAL ||
        line.curve_camera_point_set_size() < 2 ||
        line.curve_camera_point_set_size() > 2000) {
      AERROR << "INVALID_INPUT: ambiguous/virtual/low-confidence lane boundary";
      return;
    }
    for (const auto& point : line.curve_camera_point_set()) {
      if (!point.has_x() || !point.has_y() || !point.has_z() ||
          !std::isfinite(point.x()) || !std::isfinite(point.y()) ||
          !std::isfinite(point.z())) {
        AERROR << "INVALID_INPUT: nonfinite lane point";
        return;
      }
      points.push_back(extrinsic * Eigen::Vector3d(point.x(), point.y(), point.z()));
    }
    std::sort(points.begin(), points.end(),
              [](const auto& a, const auto& b) { return a.x() < b.x(); });
  }
  if (geometry.left.empty() || geometry.right.empty()) {
    AERROR << "INVALID_INPUT: both measured ego boundaries are required";
    return;
  }
  LaneRelation relation;
  lane_result_ = EvaluateLane(geometry, lane_policy_, 0.0, 0.0, &relation);
  if (!lane_result_.ok()) {
    AERROR << ReasonName(lane_result_.reason) << ": " << lane_result_.message;
    return;
  }
  bool same_reference = previous_lane_time > 0.0 &&
      lane_frontier_ - previous_lane_time <= config_.lane_timeout() &&
      lane_source_ == message->source_topic() &&
      lane_local_.epoch == observation_local.epoch;
  if (same_reference) {
    const Eigen::Isometry3d delta =
        Pose(observation_local).inverse() * Pose(lane_local_);
    LaneGeometry previous;
    for (const auto& point : lane_geometry_.left) {
      previous.left.push_back(delta * point);
    }
    for (const auto& point : lane_geometry_.right) {
      previous.right.push_back(delta * point);
    }
    LaneRelation predicted;
    const double age = lane_frontier_ - previous_lane_time;
    const auto checked = EvaluateLane(
        previous, lane_policy_, age * config_.relative_position_std_per_second(),
        age * config_.relative_heading_std_per_second(), &predicted);
    same_reference = checked.ok() &&
        std::abs(predicted.lateral - relation.lateral) <=
            config_.uncertainty_multiplier() *
                (predicted.lateral_std + relation.lateral_std) &&
        std::abs(predicted.heading - relation.heading) <=
            config_.uncertainty_multiplier() *
                (predicted.heading_std + relation.heading_std);
  }
  if (!same_reference) {
    ++lane_reference_generation_;
  }
  lane_geometry_ = std::move(geometry);
  lane_local_ = observation_local;
  lane_time_ = lane_frontier_;
  lane_source_ = message->source_topic();
  ++lane_sequence_;
  lane_result_ = {};
}

bool LocalizationHealthBridge::LocalUsable(double now) const {
  if (!LocalIntegrity(now)) {
    return false;
  }
  const auto& local = history_.back();
  constexpr uint64_t required =
      LOCAL_POSE_VALID | LOCAL_POSE_CONTINUOUS | VELOCITY_VALID | HEADING_VALID;
  return local.covariance_model_valid && latest_.precision_valid() &&
         Fresh(local.stamp.time, now, config_.local_timeout(), config_.future_tolerance()) &&
         now <= latest_.valid_until() &&
         local_assessment_.session_id() == local.epoch.session &&
         local_assessment_.epoch_generation() == local.epoch.generation &&
         Fresh(local_assessment_.publish_time(), now, config_.local_timeout(),
               config_.future_tolerance()) &&
         Fresh(local_assessment_.measurement_time(), now, config_.local_timeout(),
               config_.future_tolerance()) &&
         (local_assessment_.capabilities() & required) == required &&
         local_assessment_.covariance_valid() &&
         ValidCovariance(PoseCovariance(local)) &&
         local.covariance.block<3, 3>(0, 0).diagonal().maxCoeff() <=
             std::pow(config_.max_local_position_std(), 2) &&
         local.covariance.block<3, 3>(6, 6).diagonal().maxCoeff() <=
             std::pow(config_.max_local_attitude_std(), 2);
}

bool LocalizationHealthBridge::LocalIntegrity(double now) const {
  if (history_.empty() || !latest_.valid() || !local_assessment_.IsInitialized()) {
    return false;
  }
  const auto& local = history_.back();
  return local_assessment_.propagation_valid() &&
         local_assessment_.output_continuous() &&
         local_assessment_.session_id() == local.epoch.session &&
         local_assessment_.epoch_generation() == local.epoch.generation &&
         Fresh(local.stamp.time, now, config_.local_timeout(), config_.future_tolerance()) &&
         Fresh(local_assessment_.measurement_time(), now, config_.local_timeout(),
               config_.future_tolerance()) &&
         Fresh(local_assessment_.publish_time(), now, config_.local_timeout(),
               config_.future_tolerance()) &&
         now <= latest_.valid_until() && ValidCovariance(PoseCovariance(local));
}

LaneRelativeState LocalizationHealthBridge::EvaluateLaneNow(double now) const {
  LaneRelativeState message;
  message.mutable_epoch()->set_producer_session(
      history_.empty() ? "" : history_.back().epoch.session);
  message.mutable_epoch()->set_generation(
      history_.empty() ? 0 : history_.back().epoch.generation);
  message.set_observation_sequence(lane_sequence_);
  message.set_measurement_time(lane_time_);
  message.set_evaluation_time(history_.empty() ? 0.0 : history_.back().stamp.time);
  message.set_valid_until(lane_time_ + config_.lane_timeout());
  message.set_source(lane_source_);
  message.set_calibration_id(config_.lane_calibration_id());
  message.set_valid(false);
  message.set_containment_valid(false);
  message.set_reference_distance(config_.lane_reference_distance());
  message.set_reference_id(
      lane_source_ + ":" + config_.lane_calibration_id() + ":" +
      std::to_string(lane_reference_generation_) + ":station:" +
      std::to_string(config_.lane_reference_distance()));
  message.set_containment_reason("LANE_EVIDENCE_UNAVAILABLE");
  message.set_reason("LANE_EVIDENCE_UNAVAILABLE");
  if (!config_.enable_lane_evidence() || !LocalIntegrity(now) || !lane_result_.ok() ||
      !Fresh(lane_time_, now, config_.lane_timeout(), config_.future_tolerance()) ||
      lane_local_.epoch != history_.back().epoch) {
    return message;
  }
  const auto& current = history_.back();
  if (!current.covariance_model_valid || !lane_local_.covariance_model_valid) {
    message.set_reason("LOCAL_COVARIANCE_MODEL_UNAVAILABLE");
    return message;
  }
  if (current.covariance.block<3, 3>(3, 3).diagonal().maxCoeff() >
          std::pow(config_.max_local_velocity_std(), 2)) {
    message.set_reason("RELATIVE_MOTION_BUDGET_EXCEEDED");
    return message;
  }
  const Eigen::Isometry3d delta = Pose(current).inverse() * Pose(lane_local_);
  LaneGeometry geometry;
  for (const auto& point : lane_geometry_.left) {
    geometry.left.push_back(delta * point);
  }
  for (const auto& point : lane_geometry_.right) {
    geometry.right.push_back(delta * point);
  }
  const double age = current.stamp.time - lane_local_.stamp.time;
  if (age < 0.0 || age > config_.lane_timeout()) {
    message.set_reason("RELATIVE_MOTION_INTERVAL_UNAVAILABLE");
    return message;
  }
  const double rotation_std = age * config_.relative_heading_std_per_second();
  const double position_std = age * config_.relative_position_std_per_second() +
      config_.lane_reference_distance() * rotation_std;
  LaneRelation relation;
  const auto result = EvaluateLane(geometry, lane_policy_, position_std,
                                   rotation_std, &relation);
  message.set_reason(ReasonName(result.reason));
  message.set_valid(result.ok());
  message.set_containment_valid(result.ok() && relation.contained);
  message.set_containment_reason(ReasonName(relation.containment_reason));
  if (result.ok()) {
    message.set_lateral_offset(relation.lateral);
    message.set_heading_error(relation.heading);
    message.set_lane_width(relation.width);
    if (relation.containment_reason != Reason::HISTORY_UNAVAILABLE) {
      message.set_left_clearance(relation.left_clearance);
      message.set_right_clearance(relation.right_clearance);
    }
    message.set_lateral_std(relation.lateral_std);
    message.set_heading_std(relation.heading_std);
  }
  return message;
}

void LocalizationHealthBridge::Tick() {
  std::lock_guard<std::mutex> lock(mutex_);
  const double now = cyber::Clock::NowInSeconds();
  const LocalState* local = history_.empty() ? nullptr : &history_.back();
  const bool local_valid = LocalUsable(now);
  const bool integrity = LocalIntegrity(now);
  const bool covariance_valid =
      integrity && local->covariance_model_valid &&
      local_assessment_.covariance_valid();
  const auto lane = EvaluateLaneNow(now);
  apollo::localization::LocalizationAssessment assessment;
  assessment.mutable_header()->set_timestamp_sec(now);
  assessment.set_timestamp_sec(now);
  assessment.set_sequence(++assessment_sequence_);
  assessment.set_session_id(local == nullptr ? "initializing" :
      local->epoch.session + ":" + std::to_string(local->epoch.generation));
  assessment.set_odom_generation(local == nullptr ? 0 : local->epoch.generation);
  assessment.set_local_measurement_time(local == nullptr ? 0.0 : local->stamp.time);
  const bool running = local_assessment_.IsInitialized() &&
      Fresh(local_assessment_.publish_time(), now, config_.local_timeout(),
            config_.future_tolerance());
  assessment.set_estimator_running(running);
  assessment.set_estimator_converged(local != nullptr);
  assessment.set_pose_valid(integrity);
  assessment.set_velocity_valid(covariance_valid &&
      local->covariance.block<3, 3>(3, 3).diagonal().maxCoeff() <=
          std::pow(config_.max_local_velocity_std(), 2));
  assessment.set_heading_valid(covariance_valid &&
      local->covariance.block<3, 3>(6, 6).diagonal().maxCoeff() <=
          std::pow(config_.max_local_attitude_std(), 2));
  assessment.set_covariance_valid(covariance_valid);
  assessment.set_local_consistency_valid(integrity);
  assessment.set_output_continuous(integrity);
  if (covariance_valid) {
    assessment.set_position_std_x(std::sqrt(local->covariance(0, 0)));
    assessment.set_position_std_y(std::sqrt(local->covariance(1, 1)));
    assessment.set_position_std_z(std::sqrt(local->covariance(2, 2)));
    const Eigen::Vector3d forward = local->orientation * Eigen::Vector3d::UnitX();
    const double horizontal_squared = forward.head<2>().squaredNorm();
    if (horizontal_squared > 1e-6) {
      Eigen::RowVector3d yaw_jacobian;
      yaw_jacobian << -forward.x() * forward.z() / horizontal_squared,
          -forward.y() * forward.z() / horizontal_squared, 1.0;
      yaw_jacobian *= local->orientation.toRotationMatrix();
      const double variance =
          (yaw_jacobian * local->covariance.block<3, 3>(6, 6) *
           yaw_jacobian.transpose())(0, 0);
      assessment.set_yaw_std(std::sqrt(variance));
    } else {
      assessment.set_heading_valid(false);
    }
    const Matrix6d covariance = PoseCovariance(*local);
    for (int index = 0; index < 6; ++index) {
      auto* direction = assessment.add_directional_constraints();
      direction->set_id("odom:" + std::to_string(index));
      direction->set_reference_frame(config_.odom_frame());
      direction->set_reference_id(local->epoch.session + ":" +
          std::to_string(local->epoch.generation) + ":origin");
      Eigen::Matrix<double, 1, 6> projection =
          Eigen::Matrix<double, 1, 6>::Zero();
      if (index < 3) {
        projection(index) = 1.0;
      } else {
        projection.tail<3>() =
            local->orientation.toRotationMatrix().col(index - 3).transpose();
      }
      for (int col = 0; col < 6; ++col) {
        direction->add_projection(projection(col));
      }
      direction->set_unit(index < 3 ? "meters" : "radians");
      direction->set_standard_deviation(std::sqrt(covariance(index, index)));
      direction->set_error_budget(index < 3 ? config_.max_local_position_std()
                                            : config_.max_local_attitude_std());
      direction->set_evaluation_time(local->stamp.time);
      direction->set_valid_until(latest_.valid_until());
      direction->set_source("local_dead_reckoning");
      direction->set_mode(apollo::localization::DIRECTION_PROPAGATED);
      direction->set_independent(false);
    }
    if (lane.valid()) {
      for (int index : {1, 5}) {
        auto* direction = assessment.add_directional_constraints();
        direction->set_id(index == 1 ? "lane-lateral" : "lane-heading");
        direction->set_reference_frame(config_.base_frame());
        direction->set_reference_id(lane.reference_id());
        for (int col = 0; col < 6; ++col) {
          direction->add_projection(col == index ? 1.0 : 0.0);
        }
        direction->set_unit(index == 1 ? "meters" : "radians");
        direction->set_standard_deviation(index == 1 ? lane.lateral_std()
                                                     : lane.heading_std());
        direction->set_error_budget(index == 1 ? config_.max_lane_lateral_std()
                                               : config_.max_lane_heading_std());
        direction->set_last_observation_time(lane.measurement_time());
        direction->set_evaluation_time(lane.evaluation_time());
        direction->set_valid_until(lane.valid_until());
        direction->set_source(lane.source());
        direction->set_mode(apollo::localization::DIRECTION_CONSTRAINED);
        direction->set_independent(true);
        direction->set_observation_sequence(lane.observation_sequence());
      }
    }
    if (global_assessment_.IsInitialized() &&
        global_assessment_.session_id() == local->epoch.session &&
        global_assessment_.epoch_generation() == local->epoch.generation &&
        Fresh(global_assessment_.publish_time(), now, config_.local_timeout(),
              config_.future_tolerance())) {
      for (const auto& direction : global_assessment_.directional_constraints()) {
        *assessment.add_directional_constraints() = direction;
      }
    }
  }
  bool global_valid = local_valid && global_.IsInitialized() &&
      global_assessment_.IsInitialized() &&
      global_.epoch().producer_session() == local->epoch.session &&
      global_.epoch().generation() == local->epoch.generation &&
      global_assessment_.session_id() == local->epoch.session &&
      global_assessment_.epoch_generation() == local->epoch.generation &&
      global_assessment_.map_alignment_valid() &&
      global_assessment_.covariance_valid() &&
      Fresh(global_assessment_.publish_time(), now, config_.local_timeout(),
            config_.future_tolerance()) &&
      Fresh(global_.localization().measurement_time(), now, config_.local_timeout(),
            config_.future_tolerance()) &&
      Fresh(global_.last_observation_time(), now, config_.global_timeout(),
            config_.future_tolerance());
  if (global_valid) {
    Eigen::Isometry3d pose;
    Matrix6d covariance;
    if (!DecodePose(global_.localization(), &pose).ok() || global_.covariance_size() != 36) {
      global_valid = false;
    } else {
      for (int row = 0; row < 6; ++row) {
        for (int col = 0; col < 6; ++col) {
          covariance(row, col) = global_.covariance(row * 6 + col);
        }
      }
      global_valid = ValidCovariance(covariance);
    }
  }
  assessment.set_global_pose_valid(global_valid);
  assessment.set_map_alignment_valid(global_valid);
  assessment.set_georeference_valid(global_valid && global_.georeferenced());
  assessment.set_lane_level_valid(lane.valid() &&
      lane.lateral_std() <= config_.max_lane_lateral_std() &&
      lane.heading_std() <= config_.max_lane_heading_std());
  assessment.set_lane_evidence_independent(lane.valid());
  assessment.set_lane_evidence_time(lane.measurement_time());
  assessment.set_lane_evidence_valid_until(lane.valid_until());
  assessment.set_lane_source(lane.source());
  const bool recovery = integrity && global_assessment_.IsInitialized() &&
      Fresh(global_assessment_.publish_time(), now, config_.local_timeout(),
            config_.future_tolerance()) &&
      global_assessment_.session_id() == local->epoch.session &&
      global_assessment_.epoch_generation() == local->epoch.generation &&
      (global_assessment_.capabilities() & RELOCALIZATION_AVAILABLE) != 0;
  assessment.set_recovery_available(recovery);
  assessment.set_relocalization_phase(
      recovery && !global_valid ? apollo::localization::RECOVERY_RELOCALIZING
                                : apollo::localization::RECOVERY_IDLE);
  assessment.set_correction_in_progress(false);
  assessment.set_innovation_test_valid(false);
  auto* source = assessment.add_source_health();
  source->set_source_name("local_odometry");
  source->set_is_healthy(integrity);
  source->set_last_seen_timestamp_sec(local == nullptr ? 0.0 : local->stamp.time);
  if (!assessment_writer_->Write(assessment)) {
    AERROR << "FAULT: branch-compatible health assessment write";
  }
  if (config_.enable_lane_evidence() && !lane_writer_->Write(lane)) {
    AERROR << "FAULT: lane-relative output write";
  }
}

}  // namespace unified
}  // namespace localization
}  // namespace apollo
