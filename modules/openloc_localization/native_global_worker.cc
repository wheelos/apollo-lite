// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "native_global_worker.h"

#include <cmath>
#include <algorithm>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace apollo::localization::openloc_diagnostic {
namespace loc = ::openloc::localization;
namespace {

int64_t Nanoseconds(double seconds) {
  const long double ns = static_cast<long double>(seconds) * 1000000000.0L;
  if (!std::isfinite(seconds) || seconds <= 0 ||
      ns > std::numeric_limits<int64_t>::max() - 1.0L)
    throw std::invalid_argument("invalid diagnostic source timestamp");
  return static_cast<int64_t>(std::round(ns));
}

loc::Status Rejected(const char* reason) {
  return {loc::StatusCode::kRejectedMeasurement, reason};
}

void Require(const loc::Status& status) {
  if (!status.ok()) throw std::runtime_error(status.message());
}

bool ValidIdentity(const std::string& value) {
  return !value.empty() && value.size() <= 128 &&
      std::all_of(value.begin(), value.end(), [](unsigned char c) {
        return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
            (c >= '0' && c <= '9') || c == '_' || c == '-';
      });
}

std::string Csv(const std::string& text) {
  std::string result = "\"";
  for (const char c : text) {
    result += c;
    if (c == '"') result += c;
  }
  return result + '"';
}

template <typename Derived>
std::string Matrix(const Eigen::MatrixBase<Derived>& value) {
  std::ostringstream stream;
  stream << value.rows() << 'x' << value.cols() << ':'
         << std::setprecision(17);
  for (Eigen::Index row = 0; row < value.rows(); ++row) {
    for (Eigen::Index column = 0; column < value.cols(); ++column) {
      if (row != 0 || column != 0) stream << ';';
      stream << value(row, column);
    }
  }
  return stream.str();
}

std::string PriorPose(const loc::Pose& pose) {
  std::ostringstream stream;
  stream << std::setprecision(17) << pose.translation.x() << ';'
         << pose.translation.y() << ';' << pose.translation.z() << ';'
         << pose.rotation.x() << ';' << pose.rotation.y() << ';'
         << pose.rotation.z() << ';' << pose.rotation.w();
  return stream.str();
}

loc::ObservationDependency Dependency(NativeGlobalGnssInput::Dependency value) {
  switch (value) {
    case NativeGlobalGnssInput::ABSENT:
      return loc::ObservationDependency::kAbsent;
    case NativeGlobalGnssInput::PRESENT:
      return loc::ObservationDependency::kPresent;
    default:
      return loc::ObservationDependency::kUnknown;
  }
}

template <typename Input>
loc::GlobalObservationContext Context(
    const Input& input, const loc::DiagnosticAlignmentConfig& policy,
    const char* source) {
  loc::GlobalObservationContext context;
  context.measurement_time = {input.measurement_ns(), loc::ClockDomain::kUnixEpoch};
  context.receive_time = {input.receive_ns(), loc::ClockDomain::kUnixEpoch};
  context.epoch = {input.session(), input.generation()};
  context.sequence = input.sequence();
  context.source = source;
  context.reference = policy.reference;
  context.map_content_id = policy.map_content_id;
  context.map_frame = policy.map_frame;
  context.base_frame = policy.base_frame;
  context.provenance.acquisition_id = input.session();
  context.provenance.preparation_id = input.preparation_id();
  context.provenance.gnss_map_construction =
      Dependency(input.gnss_map_construction());
  return context;
}

void SetPose(const loc::Pose& pose, NativeGlobalPose* output) {
  output->set_x(pose.translation.x());
  output->set_y(pose.translation.y());
  output->set_z(pose.translation.z());
  output->set_qx(pose.rotation.x());
  output->set_qy(pose.rotation.y());
  output->set_qz(pose.rotation.z());
  output->set_qw(pose.rotation.w());
}

loc::GraphSourceQualification Qualification(
    const NativeGraphSourceQualification& input, uint32_t columns,
    uint32_t maximum_dimension) {
  loc::GraphSourceQualification result;
  if (!input.IsInitialized())
    throw std::invalid_argument("incomplete graph source qualification");
  result.uncertainty_model_id = input.uncertainty_model_id();
  result.independence_evidence_id = input.independence_evidence_id();
  result.quality_valid = input.quality_valid();
  result.uncertainty_qualified = input.uncertainty_qualified();
  result.independent_of_local = input.independent_of_local();
  result.independent_of_graph_sources = input.independent_of_graph_sources();
  if (input.has_local_correlation()) {
    const auto& source = input.local_correlation();
    if (!source.IsInitialized() || source.dimension() < 15 ||
        source.dimension() > maximum_dimension || source.conditioning_ns() <= 0 ||
        static_cast<uint64_t>(source.cross_covariance_size()) !=
            static_cast<uint64_t>(source.dimension()) * columns)
      throw std::invalid_argument("invalid graph source cross covariance shape");
    result.local_correlation.emplace();
    auto& correlation = *result.local_correlation;
    correlation.motion_error_model_id = source.motion_error_model_id();
    correlation.conditional_independence_evidence_id =
        source.conditional_independence_evidence_id();
    correlation.odometry_sequence = source.odometry_sequence();
    correlation.conditioning_time =
        loc::Timestamp{source.conditioning_ns(), loc::ClockDomain::kUnixEpoch};
    correlation.conditional_model_qualified = source.conditional_model_qualified();
    correlation.residual_independent_of_history_and_sources =
        source.residual_independent_of_history_and_sources();
    correlation.cross_covariance.resize(source.dimension(), columns);
    for (uint32_t row = 0; row < source.dimension(); ++row)
      for (uint32_t col = 0; col < columns; ++col) {
        const double value = source.cross_covariance(row * columns + col);
        if (!std::isfinite(value))
          throw std::invalid_argument("nonfinite graph source cross covariance");
        correlation.cross_covariance(row, col) = value;
      }
  }
  return result;
}

}  // namespace

NativeGlobalWorker::NativeGlobalWorker(const NativeGlobalHostConfig& config,
                                       Callback processed,
                                       MapOperation map_operation)
    : config_(config), processed_(std::move(processed)) {
  if (!config.IsInitialized() || !config.assume_gravity_aligned_odom() ||
      config.odometry_topic().empty() || config.gnss_topic().empty() ||
      config.output_topic().empty() ||
      config.output_topic().rfind("/openloc/diagnostic/", 0) != 0 ||
      config.odometry_topic() == config.gnss_topic() ||
      config.output_topic() == config.odometry_topic() ||
      config.output_topic() == config.gnss_topic() ||
      (config.has_map_topic() &&
       (config.map_topic().empty() ||
        config.map_topic() == config.odometry_topic() ||
        config.map_topic() == config.gnss_topic() ||
        config.map_topic() == config.output_topic() ||
        config.map_topic() == config.output_topic() + "/processed")) ||
      config.maximum_odometry_age_ms() == 0 ||
      config.maximum_odometry_age_ms() > 1000 ||
      (config.target_graph().recorded_source_clock() && !config.recorded_input()) ||
      !std::isfinite(config.diagnostic_position_sigma_m()) ||
      config.diagnostic_position_sigma_m() <= 0 ||
      !std::isfinite(config.diagnostic_rotation_sigma_rad()) ||
      config.diagnostic_rotation_sigma_rad() <= 0 ||
      config.journal().empty() || std::filesystem::exists(config.journal()))
    throw std::invalid_argument("invalid or existing global host profile/journal");
  policy_.reference = {config.reference_id(), config.reference_version(),
                       config.calibration_id()};
  policy_.reference.geodetic.state = loc::GeodeticReferenceState::kDiagnosticOnly;
  policy_.map_content_id = config.content_id();
  policy_.map_frame = {"diagnostic-global-xy"};
  policy_.odom_frame = {"native-odom"};
  policy_.base_frame = {"imu"};
  policy_.enable_horizontal_bootstrap = true;
  policy_.enable_heading_bootstrap = config.structured_gnss().has_heading();
  policy_.assume_gravity_aligned_odom = true;
  policy_.maximum_odometry_age_ns =
      static_cast<int64_t>(config.maximum_odometry_age_ms()) * 1000000;
  if (config.has_online_map()) {
    std::set<std::string> topics{
        config.odometry_topic(), config.gnss_topic(), config.output_topic(),
        config.output_topic() + "/processed"};
    if (config.has_reset_topic()) {
      topics.insert(config.reset_topic());
      topics.insert(config.reset_topic() + "/ack");
    }
    if (config.has_map_topic() ||
        !topics.insert(config.online_map().cloud_topic()).second)
      throw std::invalid_argument("raw map topic conflicts with prepared map/host topics");
    if (config.online_map().has_recorded_ack_session() &&
        (!config.recorded_input() ||
         !ValidIdentity(config.online_map().recorded_ack_session()) ||
         !topics.insert(config.online_map().cloud_topic() + "/recorded_ack").second))
      throw std::invalid_argument("invalid recorded raw map acknowledgement profile");
    map_matcher_ = std::make_unique<NativeMapMatcher>(config);
    map_operation_ = map_operation ? std::move(map_operation)
        : MapOperation([this](const NativeMapJob& job) {
            return map_matcher_->Match(job);
          });
  } else if (map_operation) {
    throw std::invalid_argument("map operation requires online map mode");
  }
  if (config.has_reset_topic()) {
    std::set<std::string> topics{
        config.odometry_topic(), config.gnss_topic(), config.output_topic(),
        config.output_topic() + "/processed"};
    if (config.has_map_topic()) topics.insert(config.map_topic());
    if (config.reset_topic().rfind("/openloc/diagnostic/", 0) != 0 ||
        !topics.insert(config.reset_topic()).second ||
        !topics.insert(config.reset_topic() + "/ack").second ||
        !ValidIdentity(config.alignment_session()) ||
        config.reset_consumers_size() < 1 || config.reset_consumers_size() > 16)
      throw std::invalid_argument("invalid global reset topic/session/consumers");
    std::set<std::string> consumers;
    for (const auto& consumer : config.reset_consumers()) {
      if (!ValidIdentity(consumer) || !consumers.insert(consumer).second)
        throw std::invalid_argument("invalid or duplicate reset consumer");
    }
  } else if (config.has_alignment_session() || config.reset_consumers_size()) {
    throw std::invalid_argument("reset session/consumers require a reset topic");
  }
  if (config.has_structured_gnss()) {
    if (config.has_raw_gga())
      throw std::invalid_argument("structured GNSS and raw GGA are exclusive");
    const auto& structured = config.structured_gnss();
    if (structured.has_heading()) {
      std::set<std::string> topics{
          config.odometry_topic(), config.gnss_topic(), config.output_topic(),
          config.output_topic() + "/processed"};
      if (config.has_map_topic()) topics.insert(config.map_topic());
      if (config.has_online_map()) topics.insert(config.online_map().cloud_topic());
      if (config.has_reset_topic()) {
        topics.insert(config.reset_topic());
        topics.insert(config.reset_topic() + "/ack");
      }
      if (!topics.insert(structured.heading().topic()).second)
        throw std::invalid_argument("dual antenna topic conflicts with host topics");
    }
    structured_gnss_ = std::make_unique<NativeStructuredGnssAdapter>(config);
  }
  ConfigureOwner();
  if (config.has_raw_gga()) gga_ = std::make_unique<NativeGgaAdapter>(config);
  journal_.open(config.journal());
  if (!journal_) throw std::runtime_error("cannot open global owner journal");
  journal_ << "kind,sequence,measurement_ns,receive_ns,status,correction_id,"
              "alignment_updated,detail,observation_source,decision_id,"
              "odometry_initialization,odometry_deskew,shared_local_scans,"
              "gnss_map_construction,alignment_session,alignment_generation,"
              "reset_sequence\n";
  const auto innovation_path = config.journal() + ".innovations.csv";
  if (std::filesystem::exists(innovation_path))
    throw std::invalid_argument("global innovation journal already exists");
  innovation_journal_.open(innovation_path);
  if (!innovation_journal_)
    throw std::runtime_error("cannot open global innovation journal");
  innovation_journal_
      << "decision_id,timestamp_ns,receive_ns,sequence,status,message,source,"
         "innovation_available,innovation_unavailable_reason,"
         "observed_dimension,prior_state_frame,prior_state_dimension,"
         "pre_update_residual,innovation_covariance,nis,"
         "prior_pose_xyz_qxyzw,prior_covariance\n";
  thread_ = std::thread([this] { Run(); });
}

NativeGlobalWorker::~NativeGlobalWorker() { Stop(); }

void NativeGlobalWorker::ConfigureOwner() {
  owner_.reset();
  graph_owner_.reset();
  if (!config_.has_target_graph()) {
    owner_ = std::make_unique<loc::DiagnosticGlobalAlignment>();
    Require(owner_->Configure(policy_));
    return;
  }
  const auto& input = config_.target_graph();
  if (!input.IsInitialized() || !input.allow_uncalibrated_inputs() ||
      input.maximum_error_dimension() > kMaximumNativeGraphDimension ||
      input.history_duration_ms() == 0 || input.history_duration_ms() > 30000 ||
      input.maximum_observation_age_ms() == 0 ||
      input.maximum_observation_age_ms() > input.history_duration_ms())
    throw std::invalid_argument("invalid explicit native target graph profile");
  loc::FixedLagAlignmentConfig config;
  config.enable_heading_bootstrap = config_.structured_gnss().has_heading();
  config.assume_gravity_aligned_odom = config_.assume_gravity_aligned_odom();
  config.reference = policy_.reference;
  config.map_content_id = policy_.map_content_id;
  config.map_frame = policy_.map_frame;
  config.odom_frame = policy_.odom_frame;
  config.base_frame = policy_.base_frame;
  config.motion_error_model_id = input.motion_error_model_id();
  config.motion_error_model_qualified = input.motion_error_model_qualified();
  config.maximum_error_dimension = input.maximum_error_dimension();
  config.history_duration_ns = static_cast<int64_t>(input.history_duration_ms()) * 1000000;
  config.maximum_observation_age_ns =
      static_cast<int64_t>(input.maximum_observation_age_ms()) * 1000000;
  config.maximum_odometry_age_ns = policy_.maximum_odometry_age_ns;
  config.maximum_full_pose_age_ns = policy_.maximum_full_pose_age_ns;
  config.maximum_states = input.maximum_states();
  config.solve_budget_ms = input.solve_budget_ms();
  config.verification_frames = input.verification_frames();
  config.allow_uncalibrated_inputs = true;
  graph_owner_ = std::make_unique<loc::FixedLagGlobalAlignment>();
  Require(graph_owner_->Configure(config));
}

loc::Status NativeGlobalWorker::OwnerResult(
    loc::DiagnosticAlignmentResult* result, std::string* graph_assessment_reason) const {
  if (owner_) return owner_->GetResult(result);
  loc::FixedLagAlignmentResult graph;
  const auto status = graph_owner_->GetResult(
      {Nanoseconds(latest_->measurement_time()), loc::ClockDomain::kUnixEpoch}, &graph);
  if (!status.ok()) return status;
  if (graph_assessment_reason) *graph_assessment_reason = graph.assessment.message();
  *result = {};
  result->timestamp = graph.timestamp;
  result->evaluation_time = graph.evaluation_time;
  result->epoch = graph.epoch;
  result->reference = graph.reference;
  result->map_content_id = graph.map_content_id;
  result->map_frame = graph.map_frame;
  result->odom_frame = graph.odom_frame;
  result->base_frame = graph.base_frame;
  result->verification_pending = graph.verification_pending;
  result->odometry_fresh = graph.odometry_fresh;
  result->full_pose_evidence_fresh = graph.full_pose_evidence_fresh;
  result->horizontal_evidence_fresh = graph.horizontal_evidence_fresh;
  if (graph.horizontal_estimate) {
    const auto& horizontal = *graph.horizontal_estimate;
    result->horizontal_estimate = loc::DiagnosticHorizontalEstimate{
        horizontal.map_from_odom_xy_yaw, horizontal.map_base_xy,
        horizontal.odom_reference_xy, horizontal.covariance,
        horizontal.last_observation_time, horizontal.correction_id};
  }
  result->decisions_dropped = graph.decisions_dropped;
  if (graph.last_full_pose_time_seconds > 0)
    result->last_full_pose_time = loc::Timestamp{
        Nanoseconds(graph.last_full_pose_time_seconds), loc::ClockDomain::kUnixEpoch};
  if (graph.estimate) {
    const auto& estimate = *graph.estimate;
    result->estimate = loc::DiagnosticAlignmentEstimate{
        estimate.map_from_odom, estimate.map_from_base,
        estimate.alignment_covariance, estimate.correction_id};
  }
  std::vector<loc::GraphObservationDecision> decisions;
  const auto journal = graph_owner_->GetDecisions(&decisions);
  if (!journal.ok()) return journal;
  for (const auto& decision : decisions) {
    auto observation = decision.observation;
    const auto& qualification = decision.qualification;
    std::string detail = observation.status.message() +
        " graph_uncertainty_model=" + qualification.uncertainty_model_id +
        " graph_independence_evidence=" + qualification.independence_evidence_id +
        " uncertainty_qualified=" + std::to_string(qualification.uncertainty_qualified);
    if (qualification.local_correlation) {
      const auto& correlation = *qualification.local_correlation;
      detail += " conditional_model=" + correlation.motion_error_model_id +
          " conditional_evidence=" + correlation.conditional_independence_evidence_id +
          " conditioning_sequence=" + std::to_string(correlation.odometry_sequence);
      if (correlation.conditioning_time)
        detail += " conditioning_ns=" +
            std::to_string(correlation.conditioning_time->nanoseconds);
    }
    observation.status = {observation.status.code(), std::move(detail)};
    result->decisions.push_back(std::move(observation));
  }
  return loc::Status::Ok();
}

void NativeGlobalWorker::Enqueue(Event event) {
  const auto maximum = std::holds_alternative<NativeJointOdometryInput>(event.message)
      ? kMaximumNativeGraphBytes
      : config_.has_target_graph() ? std::size_t{256 * 1024}
      : std::holds_alternative<RawMapPacket>(event.message) ? 512 * 1024 : 65536;
  if (event.arrival > Clock::now() || event.bytes > maximum)
    throw std::invalid_argument("invalid or oversized global host input");
  std::lock_guard<std::mutex> lock(mutex_);
  if (error_) std::rethrow_exception(error_);
  if (stopping_) throw std::runtime_error("global worker has stopped");
  const std::size_t queue_bound = config_.has_target_graph()
      ? 64 * 1024 * 1024 : 1024 * 1024;
  if (outstanding_ >= 256 || event.bytes > queue_bound - bytes_) {
    error_ = std::make_exception_ptr(
        std::runtime_error("global owner input queue overflow"));
    condition_.notify_all();
    std::rethrow_exception(error_);
  }
  ++outstanding_;
  bytes_ += event.bytes;
  queue_.push_back(std::move(event));
  condition_.notify_one();
}

void NativeGlobalWorker::AddOdometry(NativeOdometryDiagnostic input,
                                     Clock::time_point arrival) {
  if (config_.has_target_graph())
    throw std::invalid_argument("target graph requires coherent joint ODOM packets");
  const auto bytes = input.ByteSizeLong();
  Enqueue({std::move(input), arrival, bytes});
}

void NativeGlobalWorker::AddGraphOdometry(NativeJointOdometryInput input,
                                          Clock::time_point arrival) {
  if (!config_.has_target_graph())
    throw std::invalid_argument("joint ODOM requires explicit target graph selection");
  const auto bytes = input.ByteSizeLong();
  Enqueue({std::move(input), arrival, bytes});
}

void NativeGlobalWorker::AddGnss(NativeGlobalGnssInput input,
                                 Clock::time_point arrival) {
  if (gga_ || structured_gnss_)
    throw std::invalid_argument("prepared GNSS input disabled in receiver mode");
  const auto bytes = input.ByteSizeLong();
  Enqueue({std::move(input), arrival, bytes});
}

void NativeGlobalWorker::AddRawGga(RawGgaPacket packet,
                                  Clock::time_point arrival) {
  if (!gga_ || packet.bytes.empty() || packet.bytes.size() > 4096 ||
      packet.sequence == 0 || packet.receive_ns <= 0)
    throw std::invalid_argument("raw GGA input disabled or invalid packet");
  const auto bytes = sizeof(RawGgaPacket) + packet.bytes.size();
  Enqueue({std::move(packet), arrival, bytes});
}

void NativeGlobalWorker::AddBestPose(BestPosePacket packet, Clock::time_point arrival) {
  if (!structured_gnss_ || packet.receive_ns <= 0)
    throw std::invalid_argument("structured GNSS input disabled or invalid receive time");
  const auto bytes = packet.message.ByteSizeLong() + sizeof(packet.receive_ns);
  Enqueue({std::move(packet), arrival, bytes});
}

void NativeGlobalWorker::AddHeading(HeadingPacket packet, Clock::time_point arrival) {
  if (!structured_gnss_ || !config_.structured_gnss().has_heading() ||
      packet.receive_ns <= 0)
    throw std::invalid_argument("dual antenna input disabled or invalid receive time");
  const auto bytes = packet.message.ByteSizeLong() + sizeof(packet.receive_ns);
  Enqueue({std::move(packet), arrival, bytes});
}

void NativeGlobalWorker::AddMap(NativeGlobalMapInput input,
                                Clock::time_point arrival) {
  if (!config_.has_map_topic())
    throw std::invalid_argument("map input is not enabled in this profile");
  const auto bytes = input.ByteSizeLong();
  Enqueue({std::move(input), arrival, bytes});
}

void NativeGlobalWorker::AddRawMap(RawMapPacket packet, Clock::time_point arrival) {
  if (config_.has_target_graph() || !map_matcher_ || !packet.cloud.has_header() ||
      !packet.cloud.header().has_sequence_num() ||
      packet.cloud.header().sequence_num() == 0 ||
      !packet.cloud.has_measurement_time() || packet.cloud.point_size() == 0 ||
      packet.cloud.point_size() > 200000 ||
      packet.cloud.ByteSizeLong() > 8 * 1024 * 1024 || packet.receive_ns <= 0)
    throw std::invalid_argument("raw map input disabled or invalid");
  const auto stamp = NativeMapMatcher::Reference(packet.cloud);
  packet.reference_ns = stamp;
  packet.source_points = packet.cloud.point_size();
  packet.no_returns = 0;
  uint64_t previous = 0;
  for (const auto& point : packet.cloud.point()) {
    if (!point.has_timestamp() || point.timestamp() == 0 ||
        point.timestamp() > static_cast<uint64_t>(stamp) ||
        point.timestamp() < previous || !point.has_x() || !point.has_y() ||
        !point.has_z() || !std::isfinite(point.x()) ||
        !std::isfinite(point.y()) || !std::isfinite(point.z()))
      throw std::invalid_argument("invalid raw map point before bounded sampling");
    previous = point.timestamp();
    if (point.x() == 0 && point.y() == 0 && point.z() == 0) ++packet.no_returns;
  }
  const auto returns = packet.source_points - packet.no_returns;
  if (!returns) throw std::invalid_argument("raw map scan has no returns");
  packet.sample_stride = (returns + 11999) / 12000;
  if (packet.no_returns || packet.sample_stride > 1) {
    std::remove_reference_t<decltype(*packet.cloud.mutable_point())> points;
    points.Swap(packet.cloud.mutable_point());
    uint32_t index = 0;
    for (const auto& point : points) {
      if (point.x() == 0 && point.y() == 0 && point.z() == 0) continue;
      if (index++ % packet.sample_stride == 0)
        *packet.cloud.add_point() = point;
    }
  }
  const auto bytes = packet.cloud.ByteSizeLong() + sizeof(packet.receive_ns);
  Enqueue({std::move(packet), arrival, bytes});
}

void NativeGlobalWorker::AddReset(NativeGlobalResetRequest input,
                                 Clock::time_point arrival) {
  if (!config_.has_reset_topic())
    throw std::invalid_argument("global reset input disabled");
  const auto bytes = input.ByteSizeLong();
  Enqueue({std::move(input), arrival, bytes});
}

void NativeGlobalWorker::AddResetAck(NativeGlobalResetAck input,
                                    Clock::time_point arrival) {
  if (!config_.has_reset_topic())
    throw std::invalid_argument("global reset acknowledgement disabled");
  const auto bytes = input.ByteSizeLong();
  Enqueue({std::move(input), arrival, bytes});
}

loc::Status NativeGlobalWorker::CheckAlignment(
    const std::string& session, uint64_t generation, int64_t measurement) const {
  if (!config_.has_reset_topic()) return loc::Status::Ok();
  if (session != config_.alignment_session() || generation != reset_.generation)
    return {loc::StatusCode::kEpochMismatch, "absolute_alignment_generation_mismatch"};
  if (!reset_.pending.empty())
    return Rejected("waiting_for_reset_consumers");
  if (measurement <= reset_frontier_ns_)
    return Rejected("absolute_measurement_precedes_reset_frontier");
  return loc::Status::Ok();
}

loc::Status NativeGlobalWorker::Process(const NativeGlobalResetRequest& input) {
  if (!input.IsInitialized() || input.sequence() == 0 ||
      input.reason().empty() || input.reason().size() > 256)
    return {loc::StatusCode::kInvalidArgument, "incomplete_global_reset_request"};
  if (input.alignment_session() != config_.alignment_session())
    return {loc::StatusCode::kEpochMismatch, "reset_publisher_session_mismatch"};
  if (reset_.request && input.sequence() == reset_.request->sequence()) {
    if (input.SerializeAsString() != reset_.request->SerializeAsString())
      return Rejected("conflicting_reset_request_retry");
    return loc::Status::Ok();
  }
  if (input.expected_alignment_generation() != reset_.generation ||
      (reset_.request && input.sequence() <= reset_.request->sequence()))
    return {loc::StatusCode::kEpochMismatch, "stale_reset_request"};
  if (!reset_.pending.empty()) return Rejected("reset_already_pending");
  if (reset_.generation == std::numeric_limits<uint64_t>::max())
    return Rejected("alignment_generation_exhausted");
  // Retire the old owner before creating its replacement. No overlap and no
  // local owner operation: this discards global alignment/history only.
  ConfigureOwner();
  last_decision_ = 0;
  owner_has_odometry_ = false;
  ++reset_.generation;
  reset_.request = input;
  reset_.pending = std::set<std::string>(config_.reset_consumers().begin(),
                                         config_.reset_consumers().end());
  reset_frontier_ns_ = latest_ ? Nanoseconds(latest_->measurement_time()) : 0;
  if (gga_)
    gga_->Reset(latest_ ? latest_->session() : "",
                latest_ ? latest_->generation() : 0, reset_frontier_ns_);
  if (structured_gnss_)
    structured_gnss_->Reset(latest_ ? latest_->session() : "",
                            latest_ ? latest_->generation() : 0, reset_frontier_ns_);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.reset();
  }
  return {loc::StatusCode::kOk, "global_alignment_retired"};
}

loc::Status NativeGlobalWorker::Process(const NativeGlobalResetAck& input) {
  if (!input.IsInitialized() || !ValidIdentity(input.consumer_id()))
    return {loc::StatusCode::kInvalidArgument, "incomplete_reset_consumer_ack"};
  if (input.alignment_session() != config_.alignment_session() ||
      input.alignment_generation() != reset_.generation || !reset_.request ||
      input.sequence() != reset_.request->sequence())
    return {loc::StatusCode::kEpochMismatch, "stale_reset_consumer_ack"};
  if (std::find(config_.reset_consumers().begin(),
                config_.reset_consumers().end(), input.consumer_id()) ==
      config_.reset_consumers().end())
    return Rejected("unknown_reset_consumer");
  const bool was_pending = !reset_.pending.empty();
  reset_.pending.erase(input.consumer_id());
  if (was_pending && reset_.pending.empty()) {
    reset_frontier_ns_ = latest_ ? Nanoseconds(latest_->measurement_time()) : 0;
    if (gga_)
      gga_->Reset(latest_ ? latest_->session() : "",
                  latest_ ? latest_->generation() : 0, reset_frontier_ns_);
    if (structured_gnss_)
      structured_gnss_->Reset(latest_ ? latest_->session() : "",
                              latest_ ? latest_->generation() : 0, reset_frontier_ns_);
  }
  return {loc::StatusCode::kOk, "reset_consumer_retired:" + input.consumer_id()};
}

loc::Status NativeGlobalWorker::Process(const NativeJointOdometryInput& input,
                                       Clock::time_point arrival) {
  if (!input.IsInitialized())
    throw std::invalid_argument("incomplete coherent native graph input");
  if (!input.model_available()) {
    if (input.model_detail().empty() || input.model_detail().size() > 1024)
      throw std::invalid_argument("unavailable native graph input needs a reason");
    auto unavailable = input.odometry();
    unavailable.set_usable(false);
    unavailable.clear_native_error_state();
    const auto status = Process(unavailable, arrival);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      unavailable_model_detail_ = input.model_detail();
    }
    return {status.code(), "native_joint_model_unavailable:" + input.model_detail()};
  }
  auto decoded = DecodeNativeGraph(input,
      config_.target_graph().motion_error_model_id(),
      config_.target_graph().maximum_error_dimension());
  const auto status = Process(input.odometry(), arrival, &decoded);
  if (!status.ok() || !decoded.scan || !map_matcher_) return status;
  if (!reset_.pending.empty()) return Rejected("waiting_for_reset_consumers");
  if (map_job_) return Rejected("map_matcher_busy");
  const auto measurement = Nanoseconds(decoded.scan->reference.time);
  if (measurement <= reset_frontier_ns_)
    return Rejected("absolute_measurement_precedes_reset_frontier");
  NativeMapJob job;
  job.joint_scan = std::move(decoded.scan);
  job.reference_ns = measurement;
  job.receive_ns = input.receive_ns();
  job.session = latest_->session();
  job.generation = latest_->generation();
  job.alignment_session = config_.alignment_session();
  job.alignment_generation = reset_.generation;
  map_job_ = std::move(job);
  map_local_revision_ = local_revision_;
  map_alignment_generation_ = reset_.generation;
  map_deadline_ = arrival +
      std::chrono::milliseconds(config_.online_map().task_deadline_ms());
  return {loc::StatusCode::kPending, "shared_native_map_task_admitted"};
}

loc::Status NativeGlobalWorker::Process(const NativeOdometryDiagnostic& input,
                                       Clock::time_point arrival,
                                       const NativeDecodedGraph* graph) {
  if (!input.IsInitialized() ||
      !input.has_publication_sequence() || input.publication_sequence() == 0 ||
      (input.session() == publication_session_ &&
       input.publication_sequence() <= last_publication_sequence_) ||
      !input.diagnostic_only() || input.trusted() ||
      input.recorded_input() != config_.recorded_input() ||
      input.recorded_source_clock() !=
          (config_.has_target_graph() && config_.target_graph().recorded_source_clock()) ||
      (input.recorded_source_clock() &&
       (!input.recorded_input() || !input.has_processing_delay_s() ||
        !std::isfinite(input.processing_delay_s()) || input.processing_delay_s() < 0)))
    throw std::invalid_argument("invalid global host ODOM publication identity");
  if (input.has_native_error_state()) {
    const auto& error = input.native_error_state();
    const auto finite_vector = [](const apollo::common::Point3D& vector) {
      return vector.has_x() && vector.has_y() && vector.has_z() &&
             std::isfinite(vector.x()) && std::isfinite(vector.y()) &&
             std::isfinite(vector.z());
    };
    if (!input.usable() || error.covariance_size() != 225 ||
        !std::all_of(error.covariance().begin(), error.covariance().end(),
                     [](double value) { return std::isfinite(value); }) ||
        !finite_vector(error.gyro_bias()) ||
        !finite_vector(error.accel_bias()) ||
        !finite_vector(error.angular_velocity()))
      throw std::invalid_argument("invalid native ODOM error snapshot");
  }
  last_publication_sequence_ = input.publication_sequence();
  publication_session_ = input.session();
  if (!input.usable()) {
    local_available_ = false;
    if (map_matcher_) {
      map_history_.clear();
      ++local_revision_;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.reset();
    return Rejected("local_output_unavailable");
  }
  const auto measurement = Nanoseconds(input.measurement_time());
  const auto receive = Nanoseconds(input.receive_time());
  if (input.session().empty() || input.generation() == 0 ||
      input.source_sequence() == 0 || !input.has_measurement_age() ||
      !std::isfinite(input.measurement_age()) || input.measurement_age() < 0 ||
      !input.has_odom_from_imu() || receive < measurement - 20000000)
    throw std::invalid_argument("invalid usable native ODOM metadata");
  const auto& transform = input.odom_from_imu();
  const auto suffix = input.session() + "/" + std::to_string(input.generation());
  if (transform.header().frame_id() != "openloc_diag/odom/" + suffix ||
      transform.child_frame_id() != "openloc_diag/imu/" + suffix ||
      transform.header().timestamp_sec() != input.measurement_time())
    throw std::invalid_argument("native ODOM source/frame identity mismatch");
  if (!input.new_measurement()) {
    if (!latest_ || input.source_sequence() != latest_->source_sequence() ||
        input.session() != latest_->session() ||
        input.generation() != latest_->generation() ||
        input.measurement_time() != latest_->measurement_time() ||
        input.receive_time() != latest_->receive_time() ||
        transform.SerializeAsString() !=
            latest_->odom_from_imu().SerializeAsString() ||
        input.has_native_error_state() != latest_->has_native_error_state() ||
        input.native_error_state().SerializeAsString() !=
            latest_->native_error_state().SerializeAsString())
      throw std::invalid_argument("held ODOM changed its source snapshot");
    return loc::Status::Ok();
  }
  if (latest_ && input.session() == latest_->session() &&
      input.generation() == latest_->generation() &&
      input.source_sequence() <= latest_->source_sequence())
    throw std::invalid_argument("native ODOM source sequence regressed");
  loc::OdometrySample sample;
  sample.timestamp = {measurement, loc::ClockDomain::kUnixEpoch};
  sample.epoch = {input.session(), input.generation()};
  sample.parent_frame = policy_.odom_frame;
  sample.child_frame = policy_.base_frame;
  const auto& pose = transform.transform();
  if (!pose.has_translation() || !pose.translation().has_x() ||
      !pose.translation().has_y() || !pose.translation().has_z() ||
      !pose.has_rotation() || !pose.rotation().has_qx() ||
      !pose.rotation().has_qy() || !pose.rotation().has_qz() ||
      !pose.rotation().has_qw())
    throw std::invalid_argument("native ODOM pose fields missing");
  sample.pose.translation = {pose.translation().x(), pose.translation().y(),
                             pose.translation().z()};
  sample.pose.rotation = Eigen::Quaterniond(
      pose.rotation().qw(), pose.rotation().qx(), pose.rotation().qy(),
      pose.rotation().qz());
  Eigen::Matrix<double, 6, 6> covariance =
      Eigen::Matrix<double, 6, 6>::Zero();
  covariance.diagonal().head<3>().setConstant(
      std::pow(config_.diagnostic_position_sigma_m(), 2));
  covariance.diagonal().tail<3>().setConstant(
      std::pow(config_.diagnostic_rotation_sigma_rad(), 2));
  if (graph_owner_) {
    if (!graph) throw std::invalid_argument("native graph snapshot is missing");
    const auto status = graph_owner_->AddOdometry(graph->odometry,
        owner_has_odometry_ && graph->motion ? &*graph->motion : nullptr);
    if (!status.ok()) {
      local_available_ = false;
      std::lock_guard<std::mutex> lock(mutex_);
      snapshot_.reset();
      return status;
    }
  } else {
    Require(owner_->AddOdometry(sample, covariance));
  }
  owner_has_odometry_ = true;
  if (gga_ && (!latest_ || input.session() != latest_->session() ||
               input.generation() != latest_->generation()))
    gga_->Reset(input.session(), input.generation(), reset_frontier_ns_);
  if (structured_gnss_ && (!latest_ || input.session() != latest_->session() ||
                           input.generation() != latest_->generation()))
    structured_gnss_->Reset(input.session(), input.generation(), reset_frontier_ns_);
  if (map_matcher_) {
    if (!latest_ || input.session() != latest_->session() ||
        input.generation() != latest_->generation()) {
      map_history_.clear();
      ++local_revision_;
    }
    if (!map_history_.empty() && measurement <= map_history_.back().time_ns)
      throw std::invalid_argument("raw map ODOM history time regressed");
    Eigen::Isometry3d value = Eigen::Isometry3d::Identity();
    value.translation() = sample.pose.translation;
    value.linear() = sample.pose.rotation.toRotationMatrix();
    map_history_.push_back({measurement, value});
    while (map_history_.size() > 512 ||
           measurement - map_history_.front().time_ns > 1000000000)
      map_history_.pop_front();
  }
  latest_ = input;
  latest_arrival_ = arrival;
  local_available_ = true;
  return loc::Status::Ok();
}

loc::Status NativeGlobalWorker::Process(const NativeGlobalGnssInput& input) {
  if (!input.IsInitialized() || !input.diagnostic_only() ||
      input.recorded_input() != config_.recorded_input() ||
      input.preparation_id().empty() || input.covariance_source().empty() ||
      input.sequence() == 0 || input.measurement_ns() <= 0 ||
      input.receive_ns() <= 0 || input.session().empty() ||
      input.generation() == 0)
    throw std::invalid_argument("incomplete prepared diagnostic GNSS contract");
  const auto alignment = CheckAlignment(input.alignment_session(),
      input.alignment_generation(), input.measurement_ns());
  if (!alignment.ok()) return alignment;
  if (!input.receiver_admitted())
    return Rejected("receiver_adapter_did_not_admit");
  if (input.reference_id() != config_.reference_id() ||
      input.reference_version() != config_.reference_version() ||
      input.calibration_id() != config_.calibration_id() ||
      input.content_id() != config_.content_id())
    return {loc::StatusCode::kMapReferenceMismatch, "GNSS reference mismatch"};
  if (!latest_ || !local_available_ || !owner_has_odometry_)
    return Rejected("native_odometry_unavailable");
  loc::GlobalHorizontalObservation observation;
  observation.context = Context(input, policy_,
      gga_ ? "raw-gga-horizontal" :
      structured_gnss_ ? "novatel-best-pose-horizontal" : "prepared-gnss-xy");
  observation.position = {input.x(), input.y()};
  observation.covariance << input.covariance_xx(), input.covariance_xy(),
      input.covariance_xy(), input.covariance_yy();
  observation.point_in_base = {input.lever_x(), input.lever_y(), input.lever_z()};
  observation.covariance_source = input.covariance_source();
  return graph_owner_
      ? graph_owner_->SubmitHorizontalObservation(observation,
          input.has_graph_qualification()
              ? Qualification(input.graph_qualification(), 2,
                    config_.target_graph().maximum_error_dimension())
              : loc::GraphSourceQualification{})
      : owner_->SubmitHorizontalObservation(observation);
}

loc::Status NativeGlobalWorker::Process(const BestPosePacket& packet) {
  if (!reset_.pending.empty()) return Rejected("waiting_for_reset_consumers");
  const auto decision = structured_gnss_->Position(packet.message, packet.receive_ns);
  journal_ << "best_pose_gate," << decision.sequence << ','
           << decision.measurement_ns << ',' << decision.receive_ns << ','
           << static_cast<int>(decision.status.code()) << ",0,0,"
           << Csv(decision.status.message())
           << ",\"novatel-best-pose-horizontal\",0,0,0,0,0";
  JournalControl();
  if (!decision.observation) return decision.status;
  auto input = *decision.observation;
  if (config_.has_reset_topic()) {
    input.set_alignment_session(config_.alignment_session());
    input.set_alignment_generation(reset_.generation);
  }
  return Process(input);
}

loc::Status NativeGlobalWorker::Process(const HeadingPacket& packet) {
  if (!reset_.pending.empty()) return Rejected("waiting_for_reset_consumers");
  const auto decision = structured_gnss_->Heading(packet.message, packet.receive_ns);
  journal_ << "heading_gate," << decision.sequence << ','
           << decision.measurement_ns << ',' << decision.receive_ns << ','
           << static_cast<int>(decision.status.code()) << ",0,0,"
           << Csv(decision.status.message())
           << ",\"novatel-dual-antenna-heading\",0,0,0,0,0";
  JournalControl();
  if (!decision.status.ok()) return decision.status;
  if (!latest_ || !local_available_ || !owner_has_odometry_)
    return Rejected("native_odometry_unavailable");
  loc::GlobalHeadingObservation observation;
  auto& context = observation.context;
  context.measurement_time = {decision.measurement_ns, loc::ClockDomain::kUnixEpoch};
  context.receive_time = {decision.receive_ns, loc::ClockDomain::kUnixEpoch};
  context.sequence = decision.sequence;
  context.source = "novatel-dual-antenna-heading";
  context.epoch = {latest_->session(), latest_->generation()};
  context.reference = policy_.reference;
  context.map_content_id = policy_.map_content_id;
  context.map_frame = policy_.map_frame;
  context.base_frame = policy_.base_frame;
  const auto& policy = config_.structured_gnss();
  context.provenance.acquisition_id = latest_->session();
  context.provenance.preparation_id = policy.preparation_id();
  context.provenance.gnss_map_construction = Dependency(policy.gnss_map_construction());
  observation.heading_rad = decision.baseline_yaw_rad;
  observation.variance_rad2 = decision.variance_rad2;
  observation.direction_in_base = {policy.heading().baseline_x(),
      policy.heading().baseline_y(), policy.heading().baseline_z()};
  observation.covariance_source = "receiver-baseline-heading-uncalibrated";
  const auto alignment = CheckAlignment(config_.alignment_session(),
      config_.has_reset_topic() ? reset_.generation : 0, decision.measurement_ns);
  if (!alignment.ok()) return alignment;
  return graph_owner_
      ? graph_owner_->SubmitHeadingObservation(observation,
          policy.has_heading_graph_qualification()
              ? Qualification(policy.heading_graph_qualification(), 1,
                    config_.target_graph().maximum_error_dimension())
              : loc::GraphSourceQualification{})
      : owner_->SubmitHeadingObservation(observation);
}

loc::Status NativeGlobalWorker::Process(const RawGgaPacket& packet) {
  if (raw_sequence_ && packet.sequence != raw_sequence_ + 1)
    throw std::invalid_argument("raw GGA packet sequence gap or regression");
  raw_sequence_ = packet.sequence;
  if (!reset_.pending.empty()) return Rejected("waiting_for_reset_consumers");
  loc::Status result{loc::StatusCode::kPending, "no_complete_gga_in_packet"};
  gga_->Feed(packet.bytes, packet.receive_ns,
      [&](const NativeGgaDecision& decision) {
        result = decision.status;
        journal_ << "gga," << decision.sequence << ',' << decision.measurement_ns
                 << ',' << decision.receive_ns << ','
                 << static_cast<int>(decision.status.code())
                 << ",0,0," << Csv(decision.status.message())
                 << ",\"raw-gga-horizontal\",0,0,0,0,0";
        JournalControl();
        if (decision.observation) {
          auto observation = *decision.observation;
          observation.set_alignment_session(config_.alignment_session());
          observation.set_alignment_generation(reset_.generation);
          result = Process(observation);
          journal_ << "gnss," << decision.sequence << ',' << decision.measurement_ns
                   << ',' << decision.receive_ns << ','
                   << static_cast<int>(result.code()) << ",0,0,"
                   << Csv(result.message())
                   << ",\"raw-gga-horizontal\",0,0,0,0,0";
          JournalControl();
        }
      });
  return result;
}

loc::Status NativeGlobalWorker::Process(RawMapPacket input, Clock::time_point arrival) {
  const uint64_t sequence = input.cloud.header().sequence_num();
  if (sequence <= raw_map_sequence_)
    throw std::invalid_argument("raw map sequence regressed");
  raw_map_sequence_ = sequence;
  const auto measurement = input.reference_ns;
  if (!reset_.pending.empty()) return Rejected("waiting_for_reset_consumers");
  if (measurement <= reset_frontier_ns_)
    return Rejected("absolute_measurement_precedes_reset_frontier");
  if (!latest_ || !local_available_ || !owner_has_odometry_)
    return Rejected("native_odometry_unavailable");
  if (map_job_) return Rejected("map_matcher_busy");
  const auto now = Clock::now();
  if (latest_->measurement_age() +
      std::chrono::duration<double>(now - latest_arrival_).count() >
      config_.maximum_odometry_age_ms() / 1000.0)
    return Rejected("map_odometry_stale");
  NativeMapJob job;
  job.cloud = std::move(input.cloud);
  job.reference_ns = measurement;
  job.receive_ns = input.receive_ns;
  job.session = latest_->session();
  job.generation = latest_->generation();
  job.alignment_session = config_.alignment_session();
  job.alignment_generation = reset_.generation;
  loc::DiagnosticAlignmentResult result;
  Require(OwnerResult(&result));
  if (result.estimate && result.last_full_pose_time &&
      measurement >= result.last_full_pose_time->nanoseconds &&
      measurement - result.last_full_pose_time->nanoseconds <=
          policy_.maximum_full_pose_age_ns) {
    Eigen::Isometry3d value = Eigen::Isometry3d::Identity();
    value.translation() = result.estimate->map_from_odom.translation;
    value.linear() = result.estimate->map_from_odom.rotation.toRotationMatrix();
    job.map_from_odom = value;
  }
  map_job_ = std::move(job);
  map_local_revision_ = local_revision_;
  map_alignment_generation_ = reset_.generation;
  map_deadline_ = arrival +
      std::chrono::milliseconds(config_.online_map().task_deadline_ms());
  return {loc::StatusCode::kPending, "raw_map_task_admitted"};
}

void NativeGlobalWorker::StartPendingMap() {
  if (!map_job_ || map_future_.valid()) return;
  std::string reason;
  if (map_alignment_generation_ != reset_.generation)
    reason = "map_task_alignment_retired";
  else if (map_local_revision_ != local_revision_)
    reason = "map_task_local_epoch_retired";
  else if (Clock::now() > map_deadline_)
    reason = "map_task_deadline_expired";
  else if (map_history_.empty() ||
           map_history_.back().time_ns < map_job_->reference_ns) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!stopping_ || !queue_.empty()) return;
    reason = "map_task_shutdown_without_odometry_frontier";
  }
  if (!reason.empty()) {
    NativeMapMatch match;
    match.sequence = map_job_->joint_scan ? map_job_->joint_scan->reference.sequence
        : map_job_->cloud.header().sequence_num();
    match.measurement_ns = map_job_->reference_ns;
    match.receive_ns = map_job_->receive_ns;
    match.reason = reason;
    FinalizeMap(std::move(match));
    return;
  }
  auto job = std::move(*map_job_);
  job.history = map_history_;
  map_job_ = NativeMapJob{};
  map_job_->session = job.session;
  map_job_->generation = job.generation;
  map_future_ = std::async(std::launch::async, [this, job = std::move(job)] {
    return map_operation_(job);
  });
}

void NativeGlobalWorker::CompleteMap() {
  if (map_future_.valid() &&
      map_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    FinalizeMap(map_future_.get());
}

void NativeGlobalWorker::FinalizeMap(NativeMapMatch match) {
  loc::Status status = Rejected(match.reason.c_str());
  if (map_alignment_generation_ != reset_.generation)
    status = Rejected("map_task_alignment_retired");
  else if (map_local_revision_ != local_revision_ || !latest_ ||
           latest_->session() != map_job_->session ||
           latest_->generation() != map_job_->generation)
    status = Rejected("map_task_local_epoch_retired");
  else if (Clock::now() > map_deadline_)
    status = Rejected("map_task_deadline_expired");
  else if (!local_available_ ||
           Nanoseconds(latest_->measurement_time()) - match.measurement_ns >
               500000000)
    status = Rejected("map_task_source_stale");
  else if (match.observation)
    status = Process(*match.observation);
  journal_ << "map_match," << match.sequence << ',' << match.measurement_ns
           << ',' << match.receive_ns << ',' << static_cast<int>(status.code())
           << ",0,0," << Csv(status.message() +
               (match.search_detail.empty() ? "" : " " + match.search_detail))
           << ",\"online-map\",0,2,2,2,"
           << static_cast<int>(
                  Dependency(config_.online_map().gnss_map_construction()));
  JournalControl();
  Refresh();
  journal_.flush();
  if (!journal_) throw std::runtime_error("incomplete online map task journal");
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++map_tasks_completed_;
  }
  NativeGlobalProcessed completion;
  completion.set_kind("map_match");
  completion.set_sequence(match.sequence);
  completion.set_session(map_job_->session);
  completion.set_generation(map_job_->generation);
  completion.set_status(static_cast<int>(status.code()));
  map_job_.reset();
  if (processed_) processed_(completion);
}

loc::Status NativeGlobalWorker::Process(const NativeGlobalMapInput& input) {
  if (!input.IsInitialized() || !input.diagnostic_only() ||
      input.recorded_input() != config_.recorded_input() ||
      input.preparation_id().empty() || input.covariance_source().empty() ||
      input.sequence() == 0 || input.measurement_ns() <= 0 ||
      input.receive_ns() <= 0 || input.session().empty() ||
      input.generation() == 0)
    throw std::invalid_argument("incomplete prepared diagnostic map contract");
  const auto alignment = CheckAlignment(input.alignment_session(),
      input.alignment_generation(), input.measurement_ns());
  if (!alignment.ok()) return alignment;
  if (input.reference_id() != config_.reference_id() ||
      input.reference_version() != config_.reference_version() ||
      input.calibration_id() != config_.calibration_id() ||
      input.content_id() != config_.content_id())
    return {loc::StatusCode::kMapReferenceMismatch, "map reference mismatch"};
  if (!latest_ || !local_available_ || !owner_has_odometry_)
    return Rejected("native_odometry_unavailable");
  loc::GlobalMapObservation observation;
  observation.context = Context(input, policy_, map_matcher_ ? "online-map" : "prepared-map");
  auto& provenance = observation.context.provenance;
  provenance.odometry_initialization = Dependency(input.odometry_initialization());
  provenance.odometry_deskew = Dependency(input.odometry_deskew());
  provenance.shared_local_scans = Dependency(input.shared_local_scans());
  observation.covariance_source = input.covariance_source();
  observation.ambiguous = input.ambiguous();
  observation.recovery = input.recovery();
  const auto& pose = input.map_from_imu();
  observation.map_from_base.translation = {pose.x(), pose.y(), pose.z()};
  observation.map_from_base.rotation =
      Eigen::Quaterniond(pose.qw(), pose.qx(), pose.qy(), pose.qz());
  if (input.kind() == NativeGlobalMapInput::FULL_POSE) {
    if (input.covariance_size() != 36 || input.has_rank() ||
        input.projection_size() || input.projected_covariance_size())
      return {loc::StatusCode::kInvalidArgument, "invalid full map matrix shape"};
    for (int row = 0; row < 6; ++row)
      for (int col = 0; col < 6; ++col)
        observation.covariance(row, col) = input.covariance(row * 6 + col);
  } else {
    const auto rank = input.rank();
    if (rank < 1 || rank > 5 || input.covariance_size() ||
        input.projection_size() != static_cast<int>(rank * 6) ||
        input.projected_covariance_size() != static_cast<int>(rank * rank))
      return {loc::StatusCode::kInvalidArgument, "invalid projected map matrix shape"};
    observation.kind = loc::GlobalMapObservation::Kind::kProjectedPose;
    observation.projection.resize(rank, 6);
    observation.projected_covariance.resize(rank, rank);
    for (uint32_t row = 0; row < rank; ++row) {
      for (int col = 0; col < 6; ++col)
        observation.projection(row, col) = input.projection(row * 6 + col);
      for (uint32_t col = 0; col < rank; ++col)
        observation.projected_covariance(row, col) =
            input.projected_covariance(row * rank + col);
    }
  }
  return graph_owner_
      ? graph_owner_->SubmitMapObservation(observation,
          input.has_graph_qualification()
              ? Qualification(input.graph_qualification(),
                    input.kind() == NativeGlobalMapInput::FULL_POSE ? 6 : input.rank(),
                    config_.target_graph().maximum_error_dimension())
              : loc::GraphSourceQualification{})
      : owner_->SubmitMapObservation(observation);
}

void NativeGlobalWorker::JournalControl() {
  journal_ << ',' << Csv(config_.alignment_session()) << ','
           << reset_.generation << ','
           << (reset_.request ? reset_.request->sequence() : 0) << '\n';
}

void NativeGlobalWorker::Refresh() {
  if (!latest_ || !local_available_ || !owner_has_odometry_) return;
  loc::DiagnosticAlignmentResult result;
  std::string assessment;
  Require(OwnerResult(&result, &assessment));
  for (const auto& decision : result.decisions) {
    if (decision.decision_id <= last_decision_) continue;
    if (decision.decision_id != last_decision_ + 1)
      throw std::runtime_error("global decision journal overrun");
    last_decision_ = decision.decision_id;
    journal_ << "owner," << decision.context.sequence << ','
             << decision.context.measurement_time.nanoseconds << ','
             << decision.context.receive_time.nanoseconds << ','
             << static_cast<int>(decision.status.code()) << ','
             << decision.correction_id << ',' << decision.alignment_updated
             << ',' << Csv(decision.status.message()) << ','
             << Csv(decision.context.source) << ',' << decision.decision_id << ','
             << static_cast<int>(decision.context.provenance.odometry_initialization)
             << ',' << static_cast<int>(decision.context.provenance.odometry_deskew)
             << ',' << static_cast<int>(decision.context.provenance.shared_local_scans)
             << ','
             << static_cast<int>(decision.context.provenance.gnss_map_construction);
    const auto& innovation = decision.innovation;
    innovation_journal_ << decision.decision_id << ','
                        << decision.context.measurement_time.nanoseconds << ','
                        << decision.context.receive_time.nanoseconds << ','
                        << decision.context.sequence << ','
                        << static_cast<int>(decision.status.code()) << ','
                        << Csv(decision.status.message()) << ','
                        << Csv(decision.context.source) << ','
                        << innovation.available << ','
                        << Csv(innovation.unavailable_reason) << ','
                        << innovation.observed_dimension << ','
                        << Csv(innovation.prior_state_frame) << ','
                        << innovation.prior_state_dimension << ','
                        << Matrix(innovation.residual) << ','
                        << Matrix(innovation.covariance) << ','
                        << std::setprecision(17) << innovation.nis << ','
                        << PriorPose(innovation.prior_pose) << ','
                        << Matrix(innovation.prior_covariance) << '\n';
    if (!innovation_journal_)
      throw std::runtime_error("incomplete global innovation journal");
    JournalControl();
  }
  journal_.flush();
  innovation_journal_.flush();
  if (!journal_ || !innovation_journal_)
    throw std::runtime_error("incomplete global decision journal");
  result.decisions.clear();
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_ = Snapshot{*latest_, latest_arrival_, std::move(result), std::move(assessment)};
  unavailable_model_detail_.clear();
}

void NativeGlobalWorker::Run() {
  try {
    for (;;) {
      CompleteMap();
      StartPendingMap();
      Event event;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        if (map_job_) {
          condition_.wait_for(lock, std::chrono::milliseconds(2), [this] {
            return error_ || !queue_.empty();
          });
        } else {
          condition_.wait(lock, [this] {
            return error_ || stopping_ || !queue_.empty();
          });
        }
        if (error_) return;
        if (queue_.empty()) {
          if (map_job_) continue;
          if (stopping_) break;
          continue;
        }
        event = std::move(queue_.front());
        queue_.pop_front();
      }
      NativeGlobalProcessed processed;
      loc::Status status;
      std::string control_detail;
      int64_t measurement = 0, receive = 0;
      if (const auto* joint = std::get_if<NativeJointOdometryInput>(&event.message)) {
        status = Process(*joint, event.arrival);
        processed.set_kind("joint_odom");
        processed.set_sequence(joint->odometry().publication_sequence());
        processed.set_session(joint->odometry().session());
        processed.set_generation(joint->odometry().generation());
        measurement = joint->measurement_ns();
        receive = joint->receive_ns();
        control_detail = "motion_model=" + joint->error_model_id() +
            " dimension=" + std::to_string(joint->dimension()) +
            " previous_ns=" + std::to_string(joint->previous_ns());
      } else if (const auto* input =
              std::get_if<NativeOdometryDiagnostic>(&event.message)) {
        status = Process(*input, event.arrival);
        processed.set_kind("odom");
        processed.set_sequence(input->publication_sequence());
        processed.set_session(input->session());
        processed.set_generation(input->generation());
        if (input->has_measurement_time()) {
          measurement = Nanoseconds(input->measurement_time());
          receive = Nanoseconds(input->receive_time());
        }
      } else if (const auto* gnss =
                     std::get_if<NativeGlobalGnssInput>(&event.message)) {
        status = Process(*gnss);
        processed.set_kind("gnss");
        processed.set_sequence(gnss->sequence());
        processed.set_session(gnss->session());
        processed.set_generation(gnss->generation());
        measurement = gnss->measurement_ns();
        receive = gnss->receive_ns();
      } else if (const auto* map =
                     std::get_if<NativeGlobalMapInput>(&event.message)) {
        status = Process(*map);
        processed.set_kind("map");
        processed.set_sequence(map->sequence());
        processed.set_session(map->session());
        processed.set_generation(map->generation());
        measurement = map->measurement_ns();
        receive = map->receive_ns();
      } else if (const auto* reset =
                     std::get_if<NativeGlobalResetRequest>(&event.message)) {
        status = Process(*reset);
        processed.set_kind("reset");
        processed.set_sequence(reset->sequence());
        processed.set_session(config_.alignment_session());
        processed.set_generation(reset_.generation);
        control_detail = reset->ShortDebugString();
      } else if (const auto* ack =
                     std::get_if<NativeGlobalResetAck>(&event.message)) {
        status = Process(*ack);
        processed.set_kind("reset_ack");
        processed.set_sequence(ack->sequence());
        processed.set_session(config_.alignment_session());
        processed.set_generation(reset_.generation);
        control_detail = ack->ShortDebugString();
      } else if (auto* raw_map = std::get_if<RawMapPacket>(&event.message)) {
        processed.set_kind("raw_map");
        processed.set_sequence(raw_map->cloud.header().sequence_num());
        processed.set_session(latest_ ? latest_->session() : "");
        processed.set_generation(latest_ ? latest_->generation() : 0);
        measurement = raw_map->reference_ns;
        receive = raw_map->receive_ns;
        control_detail = "source_points=" + std::to_string(raw_map->source_points) +
            " no_returns=" + std::to_string(raw_map->no_returns) +
            " sample_stride=" + std::to_string(raw_map->sample_stride);
        status = Process(std::move(*raw_map), event.arrival);
      } else if (const auto* best = std::get_if<BestPosePacket>(&event.message)) {
        status = Process(*best);
        processed.set_kind("best_pose");
        processed.set_sequence(best->message.header().sequence_num());
        processed.set_session(latest_ ? latest_->session() : "");
        processed.set_generation(latest_ ? latest_->generation() : 0);
        receive = best->receive_ns;
        control_detail = "source_measurement=" +
            std::to_string(best->message.measurement_time()) +
            " sol_status=" + std::to_string(best->message.sol_status()) +
            " sol_type=" + std::to_string(best->message.sol_type());
      } else if (const auto* heading = std::get_if<HeadingPacket>(&event.message)) {
        status = Process(*heading);
        processed.set_kind("heading");
        processed.set_sequence(heading->message.header().sequence_num());
        processed.set_session(latest_ ? latest_->session() : "");
        processed.set_generation(latest_ ? latest_->generation() : 0);
        receive = heading->receive_ns;
        control_detail = "source_measurement=" +
            std::to_string(heading->message.measurement_time());
      } else {
        const auto& raw = std::get<RawGgaPacket>(event.message);
        status = Process(raw);
        processed.set_kind("raw_gga");
        processed.set_sequence(raw.sequence);
        processed.set_session(latest_ ? latest_->session() : "");
        processed.set_generation(latest_ ? latest_->generation() : 0);
        receive = raw.receive_ns;
      }
      journal_ << processed.kind() << ',' << processed.sequence() << ','
               << measurement << ',' << receive << ','
               << static_cast<int>(status.code()) << ",0,0,"
               << Csv(status.message() +
                      (control_detail.empty() ? "" : " " + control_detail))
               << ",\"\",0,0,0,0,0";
      JournalControl();
      // Unavailable/held ODOM must not refresh its source handoff clock.
      if (const auto* input =
              std::get_if<NativeOdometryDiagnostic>(&event.message)) {
        if (input->usable() && input->new_measurement()) Refresh();
      } else if (!std::holds_alternative<NativeGlobalResetRequest>(event.message) &&
                 !std::holds_alternative<NativeGlobalResetAck>(event.message)) {
        Refresh();
      }
      journal_.flush();
      if (!journal_) throw std::runtime_error("incomplete global ingress journal");
      {
        std::lock_guard<std::mutex> lock(mutex_);
        ++processed_events_;
        published_reset_ = reset_;
      }
      processed.set_status(static_cast<int>(status.code()));
      if (processed_) processed_(processed);
      {
        std::lock_guard<std::mutex> lock(mutex_);
        --outstanding_;
        bytes_ -= event.bytes;
      }
    }
    if (gga_) gga_->Finish();
  } catch (...) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!error_) error_ = std::current_exception();
  }
}

NativeGlobalDiagnostic NativeGlobalWorker::Sample(double publication_time,
                                                 Clock::time_point now) {
  Check();
  if (!std::isfinite(publication_time) || publication_time <= 0)
    throw std::invalid_argument("invalid global publication time");
  std::lock_guard<std::mutex> serial(sample_mutex_);
  std::optional<Snapshot> snapshot;
  uint64_t processed_events;
  uint64_t map_tasks_completed;
  ResetState reset;
  std::string unavailable_model;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot = snapshot_;
    processed_events = processed_events_;
    map_tasks_completed = map_tasks_completed_;
    reset = published_reset_;
    unavailable_model = unavailable_model_detail_;
  }
  NativeGlobalDiagnostic output;
  output.set_publication_sequence(++publications_);
  output.set_publication_time(publication_time);
  output.set_processed_events(processed_events);
  output.set_map_tasks_completed(map_tasks_completed);
  output.set_recorded_input(config_.recorded_input());
  output.set_diagnostic_only(true);
  output.set_trusted(false);
  output.set_full_pose_available(false);
  output.set_target_graph_available(false);
  output.set_selected_owner(config_.has_target_graph() ? "fixed_lag" : "diagnostic");
  output.set_recorded_source_clock(config_.target_graph().recorded_source_clock());
  if (config_.has_target_graph())
    output.set_motion_error_model_id(config_.target_graph().motion_error_model_id());
  output.set_usable(false);
  output.set_new_measurement(false);
  output.set_epoch_changed(false);
  output.set_reference_id(config_.reference_id());
  output.set_reference_version(config_.reference_version());
  output.set_calibration_id(config_.calibration_id());
  output.set_content_id(config_.content_id());
  if (config_.has_reset_topic()) {
    output.set_alignment_session(config_.alignment_session());
    output.set_alignment_generation(reset.generation);
    output.set_reset_sequence(reset.request ? reset.request->sequence() : 0);
    output.set_reset_pending(!reset.pending.empty());
    if (reset.request) output.set_reset_reason(reset.request->reason());
    for (const auto& consumer : reset.pending)
      output.add_pending_reset_consumers(consumer);
    if (!reset.pending.empty()) {
      output.set_reason("WAITING_FOR_RESET_CONSUMERS");
      return output;
    }
  }
  output.set_reason("LOCAL_ODOMETRY_UNAVAILABLE");
  if (!snapshot) {
    if (!unavailable_model.empty())
      output.set_reason("NATIVE_JOINT_MODEL_UNAVAILABLE:" + unavailable_model);
    return output;
  }
  if (config_.has_target_graph())
    output.set_graph_assessment_reason(snapshot->graph_assessment_reason);
  const auto& source = snapshot->odometry;
  output.set_session(source.session());
  output.set_generation(source.generation());
  output.set_odometry_sequence(source.source_sequence());
  output.set_measurement_ns(Nanoseconds(source.measurement_time()));
  output.set_source_receive_ns(Nanoseconds(source.receive_time()));
  if (source.has_processing_delay_s())
    output.set_local_processing_delay_s(source.processing_delay_s());
  if (now < snapshot->arrival)
    throw std::invalid_argument("global publication precedes source arrival");
  const double age = source.measurement_age() +
      std::chrono::duration<double>(now - snapshot->arrival).count();
  output.set_odometry_age_s(age);
  output.set_reason("LOCAL_ODOMETRY_STALE");
  if (age > config_.maximum_odometry_age_ms() / 1000.0) return output;
  output.set_reason("HORIZONTAL_ALIGNMENT_UNAVAILABLE");
  const auto& result = *snapshot->result;
  const auto fresh = [&](const loc::Timestamp& time) {
    const long double source_age =
        (static_cast<long double>(output.measurement_ns()) - time.nanoseconds) / 1e9L;
    return source_age + age <= policy_.maximum_full_pose_age_ns / 1e9;
  };
  if (result.estimate) {
    output.set_reason("FULL_POSE_STALE");
    if (!result.last_full_pose_time) return output;
    output.set_last_full_pose_measurement_ns(
        result.last_full_pose_time->nanoseconds);
    if (!fresh(*result.last_full_pose_time)) return output;
    const auto& estimate = *result.estimate;
    SetPose(estimate.map_from_base, output.mutable_map_from_imu());
    SetPose(estimate.map_from_odom, output.mutable_map_from_odom());
    output.set_full_pose_available(true);
    output.set_correction_id(estimate.correction_id);
    output.set_x(estimate.map_from_base.translation.x());
    output.set_y(estimate.map_from_base.translation.y());
    output.set_reason("DIAGNOSTIC_FULL_POSE");
    output.set_target_graph_available(config_.has_target_graph());
  } else {
    if (!result.horizontal_estimate) return output;
    const auto& estimate = *result.horizontal_estimate;
    output.set_reason("GNSS_HORIZONTAL_STALE");
    output.set_last_gnss_measurement_ns(estimate.last_observation_time.nanoseconds);
    if (!fresh(estimate.last_observation_time)) return output;
    output.set_correction_id(estimate.correction_id);
    output.set_x(estimate.map_base_xy.x());
    output.set_y(estimate.map_base_xy.y());
    output.set_alignment_x(estimate.map_from_odom_xy_yaw.x());
    output.set_alignment_y(estimate.map_from_odom_xy_yaw.y());
    output.set_alignment_yaw(estimate.map_from_odom_xy_yaw.z());
    output.set_reason("DIAGNOSTIC_HORIZONTAL_ONLY");
  }
  output.set_usable(true);
  output.set_epoch_changed(last_emitted_ &&
      (last_emitted_->session() != output.session() ||
       last_emitted_->generation() != output.generation()));
  output.set_new_measurement(!last_emitted_ || output.epoch_changed() ||
      last_emitted_->alignment_generation() != output.alignment_generation() ||
      last_emitted_->odometry_sequence() != output.odometry_sequence() ||
      last_emitted_->correction_id() != output.correction_id());
  last_emitted_ = output;
  return output;
}

void NativeGlobalWorker::Check() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (error_) std::rethrow_exception(error_);
}

void NativeGlobalWorker::Stop() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_ = true;
  }
  condition_.notify_all();
  if (thread_.joinable()) thread_.join();
  if (map_future_.valid()) map_future_.wait();
}

void NativeGlobalWorker::Finish() {
  Stop();
  Check();
}

}  // namespace apollo::localization::openloc_diagnostic
