// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "modules/localization/runtime/global_component.h"

#include <chrono>
#include <cmath>
#include <system_error>
#include <utility>

#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/localization/common/rigid_transform_helper.h"
#include "modules/localization/core/observed_subspace.h"
#include "modules/localization/runtime/map_resources.h"

namespace apollo {
namespace localization {
namespace unified {

GlobalLocalizationComponent::~GlobalLocalizationComponent() {
  if (health_timer_) {
    health_timer_->Stop();
  }
  if (pending_map_.valid()) {
    pending_map_.wait();
  }
}

void GlobalLocalizationComponent::Report(const Result& result) {
  input_reason_ = result.reason;
  if (!result.ok()) {
    AERROR << ReasonName(result.reason) << ": " << result.message;
  }
}

bool GlobalLocalizationComponent::InitMap() {
  const auto loaded = LoadGlobalMap(config_, &matcher_, &recovery_seeds_);
  if (!loaded.ok()) {
    Report(loaded);
    return false;
  }
  return true;
}

bool GlobalLocalizationComponent::InitGnss() {
  if (!config_.enable_gnss()) {
    return true;
  }
  Eigen::Affine3d extrinsic;
  if (!apollo::localization::common::LookupStaticTransform(
          config_.base_frame(), config_.antenna_frame(), &extrinsic, 3.0f)) {
    AERROR << "CONFIG_INVALID: GNSS antenna extrinsic unavailable";
    return false;
  }
  antenna_in_base_ = extrinsic.translation();
  gnss_ = std::make_unique<GnssAdapter>(config_, antenna_in_base_);
  const auto valid = gnss_->ValidateConfig();
  if (!valid.ok()) {
    Report(valid);
    return false;
  }
  return true;
}

bool GlobalLocalizationComponent::Init() {
  if (!GetProtoConfig(&config_) || !config_.IsInitialized() ||
      config_.map().frame_id().empty() || config_.odom_frame().empty() ||
      config_.base_frame().empty() ||
      config_.map().frame_id() == config_.odom_frame() ||
      config_.map().frame_id() == config_.base_frame() ||
      config_.odom_frame() == config_.base_frame() ||
      !std::isfinite(config_.health_period_ms()) ||
      config_.health_period_ms() <= 0.0 ||
      config_.odometry_topic().empty() || config_.observation_topic().empty() ||
      config_.global_topic().empty() || config_.constraints_topic().empty() ||
      config_.pose_topic().empty() ||
      config_.status_topic().empty() || config_.assessment_topic().empty() ||
      config_.event_topic().empty() || config_.local_assessment_topic().empty()) {
    AERROR << "CONFIG_INVALID: incomplete global configuration";
    return false;
  }
  if (config_.enable_map_switching() &&
      (config_.map_switch_topic().empty() ||
       config_.map_switch_status_topic().empty() ||
       !std::isfinite(config_.map_load_timeout()) ||
       config_.map_load_timeout() <= 0.0)) {
    AERROR << "CONFIG_INVALID: incomplete map switching configuration";
    return false;
  }
  alignment_ = std::make_unique<GlobalAlignment>(GlobalPolicy(config_));
  const auto valid = alignment_->ValidateConfig();
  if (!valid.ok() || !InitMap() || !InitGnss()) {
    if (!valid.ok()) {
      Report(valid);
    }
    return false;
  }
  global_writer_ = node_->CreateWriter<GlobalLocalization>(config_.global_topic());
  constraints_writer_ = node_->CreateWriter<GlobalConstraintSet>(config_.constraints_topic());
  pose_writer_ = node_->CreateWriter<LocalizationEstimate>(config_.pose_topic());
  status_writer_ = node_->CreateWriter<LocalizationStatus>(config_.status_topic());
  assessment_writer_ =
      node_->CreateWriter<LocalizationAssessment>(config_.assessment_topic());
  event_writer_ =
      node_->CreateWriter<LocalizationHealthEvent>(config_.event_topic());
  observation_reader_ = node_->CreateReader<GlobalObservationMessage>(
      config_.observation_topic(), [this](const auto& msg) { OnObservation(msg); });
  local_health_reader_ = node_->CreateReader<LocalizationAssessment>(
      config_.local_assessment_topic(), [this](const auto& msg) { OnLocalHealth(msg); });
  if (!global_writer_ || !constraints_writer_ || !pose_writer_ || !status_writer_ ||
      !assessment_writer_ || !event_writer_ ||
      !observation_reader_ || !local_health_reader_) {
    AERROR << "FAULT: global localization IO initialization failed";
    return false;
  }
  if (config_.enable_map_switching()) {
    map_switch_writer_ =
        node_->CreateWriter<MapSwitchStatus>(config_.map_switch_status_topic());
    map_switch_reader_ = node_->CreateReader<MapSwitchRequest>(
        config_.map_switch_topic(), [this](const auto& msg) { OnMapSwitch(msg); });
    if (!map_switch_writer_ || !map_switch_reader_) {
      AERROR << "FAULT: map switching IO initialization failed";
      return false;
    }
  }
  if (matcher_) {
    cyber::ReaderConfig reader;
    reader.channel_name = config_.lidar_topic();
    reader.pending_queue_size = 1;
    cloud_reader_ = node_->CreateReader<drivers::PointCloud>(
        reader, [this](const auto& msg) { OnCloud(msg); });
    if (!cloud_reader_) {
      AERROR << "FAULT: global cloud reader unavailable";
      return false;
    }
  }
  if (gnss_) {
    gnss_reader_ = node_->CreateReader<drivers::gnss::GnssBestPose>(
        config_.gnss_topic(), [this](const auto& msg) { OnGnss(msg); });
    if (config_.use_gnss_heading()) {
      heading_reader_ = node_->CreateReader<drivers::gnss::Heading>(
          config_.heading_topic(), [this](const auto& msg) { OnHeading(msg); });
    }
    if (!gnss_reader_ || (config_.use_gnss_heading() && !heading_reader_)) {
      AERROR << "FAULT: global GNSS readers unavailable";
      return false;
    }
  }
  broadcaster_ = std::make_unique<transform::TransformBroadcaster>(node_);
  health_timer_ = std::make_unique<cyber::Timer>(
      config_.health_period_ms(), [this]() { PublishHealth(); }, false);
  health_timer_->Start();
  return true;
}

bool GlobalLocalizationComponent::Proc(
    const std::shared_ptr<LocalOdometry>& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  const double now = cyber::Clock::NowInSeconds();
  LocalState local;
  const auto decoded =
      message ? DecodeLocal(*message, config_, now, &local)
              : Result{Reason::INVALID_INPUT, "null local odometry"};
  if (!decoded.ok()) {
    local_valid_ = false;
    alignment_->Invalidate(decoded.reason);
    Report(decoded);
    return false;
  }
  MotionIncrement motion;
  if (config_.enable_fixed_lag_graph() && message->has_motion()) {
    const auto decoded_motion = DecodeMotion(*message, local, &motion);
    if (!decoded_motion.ok()) {
      local_valid_ = false;
      alignment_->Invalidate(decoded_motion.reason);
      Report(decoded_motion);
      return false;
    }
  }
  const auto added = alignment_->AddLocal(
      local, config_.enable_fixed_lag_graph() && message->has_motion()
                 ? &motion : nullptr);
  if (!added.ok()) {
    local_valid_ = false;
    Report(added);
    return false;
  }
  local_valid_ = true;
  local_precision_valid_ = message->precision_valid();
  if (!alignment_->Evaluate(now).ok()) {
    return true;
  }
  GlobalLocalization global;
  EncodeGlobal(alignment_->Predict(local.stamp.time), local, config_, now, &global);
  if (!global_writer_->Write(global) ||
      !pose_writer_->Write(global.localization())) {
    Report({Reason::INVALID_INPUT, "global pose publication failed"});
    return false;
  }
  transform::TransformStamped tf;
  tf.mutable_header()->set_timestamp_sec(local.stamp.time);
  tf.mutable_header()->set_frame_id(config_.map().frame_id());
  tf.set_child_frame_id(config_.odom_frame());
  const auto& pose = global.map_to_odom().pose();
  auto* translation = tf.mutable_transform()->mutable_translation();
  translation->set_x(pose.position().x());
  translation->set_y(pose.position().y());
  translation->set_z(pose.position().z());
  *tf.mutable_transform()->mutable_rotation() = pose.orientation();
  broadcaster_->SendTransform(tf);
  return true;
}

void GlobalLocalizationComponent::OnLocalHealth(
    const std::shared_ptr<LocalizationAssessment>& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto* local = alignment_->latest_local();
  if (!message || !message->IsInitialized() || message->owner() != "local" ||
      local == nullptr || message->session_id() != local->epoch.session ||
      message->epoch_generation() != local->epoch.generation) {
    return;
  }
  if (!message->propagation_valid() ||
      message->availability() == INVALID) {
    local_valid_ = false;
    alignment_->Invalidate(Reason::IMU_STALE);
  }
}

void GlobalLocalizationComponent::OnObservation(
    const std::shared_ptr<GlobalObservationMessage>& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  GlobalObservation observation;
  const double now = cyber::Clock::NowInSeconds();
  const auto decoded =
      message ? DecodeObservation(*message, config_, now, &observation)
              : Result{Reason::INVALID_INPUT, "null global observation"};
  if (!decoded.ok()) {
    Report(decoded);
    return;
  }
  if (!local_valid_) {
    Report({Reason::HISTORY_UNAVAILABLE, "local capability unavailable"});
    return;
  }
  Report(alignment_->Observe(observation));
}

void GlobalLocalizationComponent::OnHeading(
    const std::shared_ptr<drivers::gnss::Heading>& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  Report(message ? gnss_->AddHeading(*message, cyber::Clock::NowInSeconds())
                 : Result{Reason::INVALID_INPUT, "null receiver heading"});
}

void GlobalLocalizationComponent::OnGnss(
    const std::shared_ptr<drivers::gnss::GnssBestPose>& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  const double now = cyber::Clock::NowInSeconds();
  if (!message || !message->has_measurement_time() || !local_valid_) {
    Report({Reason::INVALID_INPUT, "GNSS needs valid message and local history"});
    return;
  }
  LocalState local;
  const auto lookup = alignment_->Lookup(
      message->measurement_time() + config_.gnss_to_unix_offset(), &local);
  if (!lookup.ok()) {
    Report(lookup);
    return;
  }
  GlobalObservation observation;
  const auto produced =
      gnss_->Observe(*message, now, local, &observation, alignment_.get());
  Report(produced.ok() ? alignment_->Observe(observation) : produced);
}

void GlobalLocalizationComponent::OnCloud(
    const std::shared_ptr<drivers::PointCloud>& message) {
  std::lock_guard<std::mutex> cloud_lock(cloud_mutex_);
  const double received = cyber::Clock::NowInSeconds();
  LocalState local;
  Eigen::Isometry3d predicted = Eigen::Isometry3d::Identity();
  bool recovery = false;
  Cloud::Ptr cloud(new Cloud);
  GlobalObservation observation;
  std::shared_ptr<MapMatcher> matcher;
  std::vector<Eigen::Isometry3d> seeds;
  uint64_t generation = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!message || !message->has_measurement_time() ||
        message->point_size() < static_cast<int>(config_.minimum_points()) ||
        message->point_size() > static_cast<int>(config_.maximum_scan_points()) ||
        !local_valid_ || !std::isfinite(message->measurement_time()) ||
        received - message->measurement_time() > config_.max_observation_age() ||
        message->measurement_time() - received > config_.future_tolerance()) {
      ++cloud_health_.rejected;
      cloud_health_.reason = Reason::INVALID_INPUT;
      Report({Reason::INVALID_INPUT, "invalid cloud/time/local capability"});
      return;
    }
    if (message->measurement_time() <= cloud_health_.last_measurement) {
      ++cloud_health_.rejected;
      const bool duplicate =
          message->measurement_time() == cloud_health_.last_measurement;
      cloud_health_.duplicates += duplicate;
      cloud_health_.regressions += !duplicate;
      cloud_health_.reason =
          duplicate ? Reason::DUPLICATE : Reason::TIMESTAMP_REGRESSION;
      Report({cloud_health_.reason, "cloud frontier did not advance"});
      return;
    }
    cloud_health_.last_measurement = message->measurement_time();
    cloud_health_.last_receive = received;
    const auto lookup = alignment_->Lookup(message->measurement_time(), &local);
    if (!lookup.ok()) {
      ++cloud_health_.rejected;
      cloud_health_.reason = lookup.reason;
      Report(lookup);
      return;
    }
    const std::string frame =
        apollo::localization::common::GetPointCloudFrameId(*message);
    Eigen::Affine3d extrinsic;
    if (frame.empty() ||
        !apollo::localization::common::LookupStaticTransform(
            config_.base_frame(), frame, &extrinsic, 0.0f)) {
      ++cloud_health_.rejected;
      cloud_health_.reason = Reason::INVALID_INPUT;
      Report({Reason::INVALID_INPUT, "cloud static calibration unavailable"});
      return;
    }
    cloud->reserve(message->point_size());
    for (const auto& point : message->point()) {
      if (!point.has_x() || !point.has_y() || !point.has_z() ||
          !std::isfinite(point.x()) || !std::isfinite(point.y()) ||
          !std::isfinite(point.z())) {
        ++cloud_health_.rejected;
        cloud_health_.reason = Reason::INVALID_INPUT;
        Report({Reason::INVALID_INPUT, "nonfinite/incomplete cloud point"});
        return;
      }
      Eigen::Vector3d p = extrinsic * Eigen::Vector3d(point.x(), point.y(), point.z());
      if (!config_.cloud_motion_compensated()) {
        LocalState point_local;
        if (!point.has_timestamp() || point.timestamp() == 0) {
          ++cloud_health_.rejected;
          cloud_health_.reason = Reason::INVALID_INPUT;
          Report({Reason::INVALID_INPUT, "raw cloud needs per-point UNIX nanoseconds"});
          return;
        }
        const auto point_lookup =
            alignment_->Lookup(point.timestamp() * 1e-9, &point_local);
        if (!point_lookup.ok()) {
          ++cloud_health_.rejected;
          cloud_health_.reason = point_lookup.reason;
          Report(point_lookup);
          return;
        }
        p = Pose(local).inverse() * Pose(point_local) * p;
      }
      cloud->push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
    }
    // A partial track may remain useful after the full-pose lease expires.
    recovery = !alignment_->state().valid ||
        received - alignment_->state().last_observation > config_.global_timeout();
    predicted = alignment_->state().map_to_odom * Pose(local);
    observation.stamp = {message->measurement_time(), received,
                          ++cloud_sequence_, config_.clock_id()};
    observation.epoch = local.epoch;
    observation.source = "map_match";
    observation.map_id = config_.map().map_id();
    observation.map_version = config_.map().version();
    observation.calibration_id = config_.map().calibration_id();
    observation.recovery = recovery;
    observation.georeferenced = config_.map().georeferenced();
    matcher = matcher_;
    seeds = recovery_seeds_;
    generation = map_generation_;
  }
  MatchResult match;
  const auto matched = recovery ? matcher->Recover(cloud, seeds, &match)
                                : matcher->Match(cloud, predicted, &match);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (generation != map_generation_ ||
        observation.epoch != alignment_->state().epoch) {
      AINFO << "MAP_MISMATCH: discarded result from retired map/ODOM generation";
      return;
    }
    if (!matched.ok()) {
      ++cloud_health_.rejected;
      cloud_health_.reason = matched.reason;
      if (matched.reason == Reason::RELOCALIZATION_AMBIGUOUS) {
        alignment_->Invalidate(matched.reason);
      }
      Report(matched);
      return;
    }
    observation.stamp.receive_time = cyber::Clock::NowInSeconds();
    observation.pose = match.pose;
    observation.covariance = match.covariance;
    if (!match.full_pose) {
      observation.kind = GlobalObservation::Kind::PROJECTED_POSE;
      observation.projection = match.projection;
      observation.projected_covariance = match.projected_covariance;
    }
    observation.quality_valid = true;
    observation.ambiguous = match.ambiguous;
    if (!local_valid_) {
      Report({Reason::HISTORY_UNAVAILABLE, "local capability lost during matching"});
      return;
    }
    const auto observed = alignment_->Observe(observation);
    cloud_health_.reason = observed.reason;
    if (observed.ok()) {
      ++cloud_health_.accepted;
    } else {
      ++cloud_health_.rejected;
    }
    Report(observed);
  }
}

bool GlobalLocalizationComponent::PublishMapSwitch(
    uint64_t request_id, MapSwitchStatus::Phase phase,
    const Result& result, double now) {
  MapSwitchStatus status;
  status.set_request_id(request_id);
  status.set_publish_time(now);
  status.set_phase(phase);
  status.set_reason(std::string(ReasonName(result.reason)) + ": " + result.message);
  status.set_active_map_id(config_.map().map_id());
  status.set_active_map_version(config_.map().version());
  status.set_map_generation(map_generation_);
  if (!map_switch_writer_ || !map_switch_writer_->Write(status)) {
    AERROR << "FAULT: map switch status write failed";
    return false;
  }
  if (!result.ok()) {
    AERROR << status.reason();
  }
  return true;
}

void GlobalLocalizationComponent::OnMapSwitch(
    const std::shared_ptr<MapSwitchRequest>& request) {
  std::lock_guard<std::mutex> lock(mutex_);
  const double now = cyber::Clock::NowInSeconds();
  const auto* local = alignment_->latest_local();
  Result result;
  if (!request || !config_.enable_map_switching() || !local_valid_ ||
      local == nullptr || now - local->stamp.time > config_.local_timeout()) {
    result = {Reason::HISTORY_UNAVAILABLE, "Map switch needs a fresh local epoch"};
  } else {
    result = ValidateMapSwitch(*request, config_, local->epoch,
                               map_request_sequence_, now);
  }
  if (!result.ok()) {
    PublishMapSwitch(request ? request->request_id() : 0,
                     MapSwitchStatus::REJECTED, result, now);
    return;
  }
  if (pending_map_.valid()) {
    PublishMapSwitch(request->request_id(), MapSwitchStatus::REJECTED,
                     {Reason::MAP_NOT_READY, "One map load is already pending"}, now);
    return;
  }
  map_request_sequence_ = request->request_id();
  if (!PublishMapSwitch(request->request_id(), MapSwitchStatus::LOADING, {}, now)) {
    return;
  }
  pending_map_request_ = *request;
  GlobalEstimatorConfig next = config_;
  *next.mutable_map() = request->next_map();
  const Eigen::Vector3d antenna = antenna_in_base_;
  try {
    pending_map_ = std::async(std::launch::async, [next, antenna]() {
      PendingMap pending;
      pending.config = next;
      pending.result = LoadGlobalMap(next, &pending.matcher, &pending.seeds);
      if (pending.result.ok() && next.enable_gnss()) {
        pending.gnss = std::make_unique<GnssAdapter>(next, antenna);
        pending.result = pending.gnss->ValidateConfig();
      }
      return pending;
    });
  } catch (const std::system_error& error) {
    PublishMapSwitch(request->request_id(), MapSwitchStatus::REJECTED,
                     {Reason::MAP_NOT_READY, error.what()}, now);
  }
}

void GlobalLocalizationComponent::FinishMapSwitch(double now) {
  if (!pending_map_.valid() ||
      pending_map_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
    return;
  }
  PendingMap pending = pending_map_.get();
  const auto& request = pending_map_request_;
  Result result = pending.result;
  const auto* latest = alignment_->latest_local();
  if (result.ok() &&
      (now - request.request_time() > config_.map_load_timeout() ||
       !local_valid_ || latest == nullptr ||
       now - latest->stamp.time > config_.local_timeout() ||
       request.current_epoch().producer_session() != latest->epoch.session ||
       request.current_epoch().generation() != latest->epoch.generation)) {
    result = {Reason::EPOCH_MISMATCH, "Map load outlived its valid local epoch/deadline"};
  }
  if (result.ok() &&
      (request.expected_map_id() != config_.map().map_id() ||
       request.expected_map_version() != config_.map().version())) {
    result = {Reason::MAP_MISMATCH, "Active map changed while loading"};
  }
  std::unique_ptr<GlobalAlignment> next;
  if (result.ok()) {
    next = std::make_unique<GlobalAlignment>(GlobalPolicy(pending.config));
    result = next->ValidateConfig();
    if (result.ok()) {
      result = next->AddLocal(*latest);
    }
  }
  if (!result.ok()) {
    PublishMapSwitch(request.request_id(), MapSwitchStatus::REJECTED, result, now);
    return;
  }
  config_ = std::move(pending.config);
  matcher_ = std::move(pending.matcher);
  gnss_ = std::move(pending.gnss);
  recovery_seeds_ = std::move(pending.seeds);
  alignment_ = std::move(next);
  cloud_health_ = SourceCounters();
  input_reason_ = Reason::GLOBAL_UNAVAILABLE;
  ++map_generation_;
  // No registration is assumed. The new map must acquire verified alignment;
  // no local estimator, ODOM identity or local state is reset here.
  PublishMapSwitch(request.request_id(), MapSwitchStatus::COMMITTED, {}, now);
}

void GlobalLocalizationComponent::PublishHealth() {
  std::lock_guard<std::mutex> lock(mutex_);
  const double now = cyber::Clock::NowInSeconds();
  FinishMapSwitch(now);
  const auto* local = alignment_->latest_local();
  const bool local_valid = local_valid_ && local != nullptr &&
                           now - local->stamp.time <= config_.local_timeout() &&
                           local->stamp.time - now <= config_.future_tolerance();
  const auto evaluated = alignment_->Evaluate(now);
  const bool global_valid = local_valid && evaluated.ok();
  Reason reason = evaluated.reason;
  if (!local_valid && local != nullptr) {
    reason = Reason::IMU_STALE;
  } else if (evaluated.ok() && input_reason_ != Reason::NONE) {
    reason = input_reason_;
  }
  Recovery recovery = IDLE;
  if (alignment_->state().verifying) {
    recovery = VERIFYING;
  } else if (alignment_->reason() == Reason::RELOCALIZATION_AMBIGUOUS) {
    recovery = AMBIGUOUS;
  } else if (!global_valid && matcher_ && !recovery_seeds_.empty()) {
    recovery = RELOCALIZING;
  }
  auto assessment = health_.Evaluate(
      "global", local, local_valid && local_precision_valid_, global_valid,
      alignment_->state().georeferenced &&
          now - alignment_->state().last_georeference <= config_.global_timeout(),
      matcher_ && matcher_->ready() && !recovery_seeds_.empty(),
      reason, recovery, now, true, local_valid);
  assessment.set_propagation_valid(local_valid);
  GlobalConstraintSet constraints;
  constraints.mutable_epoch()->set_producer_session(local == nullptr ? "" : local->epoch.session);
  constraints.mutable_epoch()->set_generation(local == nullptr ? 0 : local->epoch.generation);
  constraints.set_publish_time(now);
  if (local_valid) {
    for (const auto& item : alignment_->evidence()) {
      const auto& observation = item.second.observation;
      if (observation.epoch != local->epoch ||
          now - observation.stamp.time > config_.global_timeout()) {
        continue;
      }
      EncodeObservation(observation, config_, now, constraints.add_observations());
      Eigen::MatrixXd projection;
      Eigen::MatrixXd noise;
      if (observation.kind == GlobalObservation::Kind::PROJECTED_POSE) {
        const auto canonical = CanonicalizeObservedSubspace(
            observation.projection, observation.projected_covariance,
            config_.geometry_rotation_length(), &projection, &noise);
        if (!canonical.ok()) {
          AERROR << ReasonName(canonical.reason) << ": " << canonical.message;
          continue;
        }
      } else if (observation.kind == GlobalObservation::Kind::POSITION) {
        projection = Eigen::Matrix<double, 3, 6>::Zero();
        projection.leftCols(3).setIdentity();
        noise = observation.covariance.topLeftCorner<3, 3>();
      } else {
        projection = Matrix6d::Identity();
        noise = observation.covariance;
      }
      for (int row = 0; row < projection.rows(); ++row) {
        // GNSS gravity/roll/pitch reuse local IMU; they are not independent rows.
        if (observation.source == "gnss" &&
            observation.kind == GlobalObservation::Kind::POSE &&
            (row == 3 || row == 4)) {
          continue;
        }
        auto* direction = assessment.add_directional_constraints();
        direction->set_id(item.first + ":" + std::to_string(row));
        direction->set_reference_frame(config_.map().frame_id());
        std::string reference = config_.base_frame();
        if (observation.kind == GlobalObservation::Kind::POSITION) {
          reference += ":point:" + std::to_string(observation.point_in_base.x()) +
              "," + std::to_string(observation.point_in_base.y()) +
              "," + std::to_string(observation.point_in_base.z());
        }
        direction->set_reference_id(
            config_.map().map_id() + "@" + config_.map().version() + ":" +
            observation.calibration_id + ":" + reference);
        const bool angular = projection.row(row).head(3).isZero(1e-12);
        const double normalization = angular ? projection.row(row).tail(3).norm() : 1.0;
        for (int col = 0; col < 6; ++col) {
          direction->add_projection(projection(row, col) / normalization);
        }
        direction->set_unit(angular ? "radians" : "meters");
        direction->set_standard_deviation(std::sqrt(noise(row, row)) / normalization);
        direction->set_error_budget(
            angular ? config_.max_attitude_std()
                    : config_.max_position_std() * projection.row(row).head(3).norm() +
                          config_.max_attitude_std() * projection.row(row).tail(3).norm());
        direction->set_last_observation_time(observation.stamp.time);
        direction->set_evaluation_time(observation.stamp.time);
        direction->set_valid_until(observation.stamp.time + config_.global_timeout());
        direction->set_source(item.first);
        direction->set_mode(apollo::localization::DIRECTION_CONSTRAINED);
        direction->set_independent(true);
        direction->set_observation_sequence(observation.stamp.sequence);
      }
    }
  }
  if (global_valid) {
    const Matrix6d covariance =
        GlobalPoseCovarianceBound(alignment_->Predict(local->stamp.time), *local);
    assessment.set_position_std(
        std::sqrt(covariance.topLeftCorner<3, 3>().diagonal().maxCoeff()));
    assessment.set_attitude_std(
        std::sqrt(covariance.bottomRightCorner<3, 3>().diagonal().maxCoeff()));
  }
  for (const auto& source : alignment_->sources()) {
    EncodeSource(source.first, source.second, now, config_.global_timeout(),
                 assessment.add_sources());
  }
  if (matcher_) {
    EncodeSource("pointcloud", cloud_health_, now, config_.global_timeout(),
                 assessment.add_sources());
  }
  if (gnss_) {
    EncodeSource("heading", gnss_->heading_health(), now,
                 config_.heading_pair_window(), assessment.add_sources());
    EncodeSource("gnss_input", gnss_->position_health(), now,
                 config_.global_timeout(), assessment.add_sources());
  }
  if (!assessment_writer_->Write(assessment)) {
    AERROR << "FAULT: global health publication failed";
  }
  if (!constraints_writer_->Write(constraints)) {
    AERROR << "FAULT: global partial-constraint publication failed";
  }
  LocalizationHealthEvent event;
  if (health_.TakeEvent(&event) && !event_writer_->Write(event)) {
    AERROR << "FAULT: global health event publication failed";
  }
  LocalizationStatus legacy;
  legacy.mutable_header()->set_timestamp_sec(now);
  legacy.set_measurement_time(local == nullptr ? 0.0 : local->stamp.time);
  legacy.set_fusion_status(global_valid ? MeasureState::OK
                                       : MeasureState::CRITICAL_ERROR);
  legacy.set_state_message(ReasonName(reason));
  if (!status_writer_->Write(legacy)) {
    AERROR << "FAULT: compatibility status publication failed";
  }
}

}  // namespace unified
}  // namespace localization
}  // namespace apollo
