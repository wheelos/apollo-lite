// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "native_map_matcher.h"

#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>

#include "pcl/io/pcd_io.h"
#include "openloc/localization/map/scan_to_map.h"
#include "openloc/localization/map/diagnostic_regional_search.h"

extern char** environ;

namespace apollo::localization::openloc_diagnostic {
namespace map = ::openloc::localization::map;
struct NativeMapMatcher::Impl {
  std::unique_ptr<map::ScanToMapLocalizer> matcher;
  std::unique_ptr<map::DiagnosticRegionalSearch> search;
  std::string session;
  uint64_t generation = 0, alignment_generation = 0;
  std::string alignment_session;
  bool tracking = false;
};

NativeMapMatcher::~NativeMapMatcher() = default;
namespace {

void CheckPcdHeader(const std::string& path) {
  std::ifstream file(path, std::ios::binary);
  std::string line;
  uint64_t points = 0, width = 0, height = 0;
  std::string fields, sizes, types, counts;
  for (int lines = 0; lines < 64 && std::getline(file, line); ++lines) {
    if (line.size() > 4096) break;
    if (line.empty() || line[0] == '#') continue;
    std::istringstream row(line);
    std::string key, value, extra;
    row >> key;
    if (key == "FIELDS") {
      std::getline(row >> std::ws, value);
      fields = value;
    } else if (key == "SIZE" || key == "TYPE" || key == "COUNT") {
      std::getline(row >> std::ws, value);
      if (key == "SIZE") sizes = value;
      if (key == "TYPE") types = value;
      if (key == "COUNT") counts = value;
    } else if (key == "POINTS" || key == "WIDTH" || key == "HEIGHT") {
      uint64_t number;
      if (!(row >> number) || row >> extra || number == 0 || number > 2000000)
        throw std::invalid_argument("invalid bounded online map PCD header");
      if (key == "POINTS") points = number;
      if (key == "WIDTH") width = number;
      if (key == "HEIGHT") height = number;
    } else if (key == "DATA") {
      row >> value;
      const bool xyz = fields == "x y z" && sizes == "4 4 4" &&
          types == "F F F" && counts == "1 1 1";
      const bool xyzi = fields == "x y z intensity" && sizes == "4 4 4 4" &&
          types == "F F F F" && counts == "1 1 1 1";
      if ((xyz || xyzi) && points &&
          width * height == points && (value == "ascii" || value == "binary"))
        return;
      break;
    }
  }
  throw std::invalid_argument("online map requires bounded uncompressed float XYZ PCD");
}

}  // namespace

std::string NativeMapMatcher::Sha256(const std::string& path) {
  // Same explicit startup hash utility as the existing native replay, without
  // shell expansion. Hash before/after loading; never trust a caller boolean.
  int fd[2];
  if (pipe(fd) != 0) throw std::runtime_error("cannot create map hash pipe");
  posix_spawn_file_actions_t actions;
  if (posix_spawn_file_actions_init(&actions) != 0) {
    close(fd[0]);
    close(fd[1]);
    throw std::runtime_error("cannot initialize map hash process actions");
  }
  int result = posix_spawn_file_actions_adddup2(&actions, fd[1], STDOUT_FILENO);
  if (!result) result = posix_spawn_file_actions_addclose(&actions, fd[0]);
  if (!result) result = posix_spawn_file_actions_addclose(&actions, fd[1]);
  std::string name = path;
  char command[] = "sha256sum", option[] = "--";
  char* argv[] = {command, option, name.data(), nullptr};
  pid_t pid = -1;
  if (!result)
    result = posix_spawnp(&pid, command, &actions, nullptr, argv, environ);
  posix_spawn_file_actions_destroy(&actions);
  close(fd[1]);
  if (result) {
    close(fd[0]);
    throw std::runtime_error("cannot execute sha256sum for map: " +
                             std::to_string(result));
  }
  std::string text;
  char buffer[256];
  ssize_t size;
  for (;;) {
    size = read(fd[0], buffer, sizeof(buffer));
    if (size < 0 && errno == EINTR) continue;
    if (size <= 0) break;
    if (text.size() < 64) text.append(buffer, std::min<size_t>(size, 64 - text.size()));
  }
  close(fd[0]);
  int status = 0;
  pid_t waited;
  do {
    waited = waitpid(pid, &status, 0);
  } while (waited < 0 && errno == EINTR);
  if (size < 0 || waited != pid || !WIFEXITED(status) || WEXITSTATUS(status) ||
      text.size() != 64 ||
      text.find_first_not_of("0123456789abcdef") != std::string::npos)
    throw std::runtime_error("map SHA256 verification failed");
  return text;
}

int64_t NativeMapMatcher::Nanoseconds(double seconds) {
  const long double ns = static_cast<long double>(seconds) * 1e9L;
  if (!std::isfinite(seconds) || seconds <= 0 ||
      ns > std::numeric_limits<int64_t>::max() - 1.0L)
    throw std::invalid_argument("invalid raw map source timestamp");
  return static_cast<int64_t>(std::round(ns));
}

int64_t NativeMapMatcher::Reference(const drivers::PointCloud& cloud) {
  const auto rounded = Nanoseconds(cloud.measurement_time());
  if (cloud.point_size() == 0) return rounded;
  const auto last = cloud.point(cloud.point_size() - 1).timestamp();
  const long double rounding = 1.0L +
      (std::nextafter(cloud.measurement_time(), std::numeric_limits<double>::infinity()) -
       cloud.measurement_time()) * 0.5e9L +
      (std::nextafter(static_cast<double>(rounded),
                      std::numeric_limits<double>::infinity()) -
       static_cast<double>(rounded)) * 0.5L;
  if (last > static_cast<uint64_t>(rounded) &&
      static_cast<long double>(last) - rounded > rounding)
    throw std::invalid_argument("raw map point exceeds scan reference");
  // Integer ns -> double ns -> double seconds can round twice. Retain an
  // endpoint within that representation bound without shifting point times.
  return last > static_cast<uint64_t>(rounded) ? last : rounded;
}

Eigen::Isometry3d NativeMapMatcher::Pose(const NativeGlobalPose& pose) {
  Eigen::Quaterniond q(pose.qw(), pose.qx(), pose.qy(), pose.qz());
  Eigen::Isometry3d value = Eigen::Isometry3d::Identity();
  value.translation() = Eigen::Vector3d(pose.x(), pose.y(), pose.z());
  if (!pose.IsInitialized() || !value.translation().allFinite() ||
      !q.coeffs().allFinite() || std::abs(q.norm() - 1) > 1e-9)
    throw std::invalid_argument("invalid online map pose/extrinsic");
  value.linear() = q.toRotationMatrix();
  return value;
}

std::optional<Eigen::Isometry3d> NativeMapMatcher::Lookup(
    const std::deque<NativeMapPose>& history, int64_t time) {
  const auto upper = std::lower_bound(history.begin(), history.end(), time,
      [](const auto& value, int64_t stamp) { return value.time_ns < stamp; });
  if (upper == history.end()) return std::nullopt;
  if (upper->time_ns == time) return upper->odom_from_imu;
  if (upper == history.begin()) return std::nullopt;
  const auto lower = std::prev(upper);
  if (upper->time_ns - lower->time_ns > 50000000) return std::nullopt;
  const double fraction =
      static_cast<double>(time - lower->time_ns) / (upper->time_ns - lower->time_ns);
  Eigen::Isometry3d value = Eigen::Isometry3d::Identity();
  value.translation() = lower->odom_from_imu.translation() * (1 - fraction) +
                        upper->odom_from_imu.translation() * fraction;
  value.linear() = Eigen::Quaterniond(lower->odom_from_imu.linear()).slerp(
      fraction, Eigen::Quaterniond(upper->odom_from_imu.linear())).toRotationMatrix();
  return value;
}

NativeMapMatcher::NativeMapMatcher(const NativeGlobalHostConfig& config)
    : config_(config), impl_(std::make_unique<Impl>()) {
  const auto& input = config.online_map();
  if (!input.IsInitialized() || input.cloud_topic().empty() ||
      input.lidar_frame().empty() || input.preparation_id().empty() ||
      input.map_sha256().size() != 64 ||
      input.map_sha256().find_first_not_of("0123456789abcdef") != std::string::npos ||
      input.task_deadline_ms() == 0 || input.task_deadline_ms() > 500 ||
      !std::isfinite(input.window_radius_m()) || input.window_radius_m() < 5 ||
      input.window_radius_m() > 30 || !std::isfinite(input.window_cell_m()) ||
      input.window_cell_m() < 0.5 || input.window_cell_m() > 20)
    throw std::invalid_argument("invalid explicit online map configuration");
  const bool seeded = input.has_initial_map_from_odom() &&
      input.has_hypothesis_minimum_ns() && input.has_hypothesis_maximum_ns() &&
      input.has_hypothesis_odometry_session() &&
      input.has_hypothesis_odometry_generation();
  if (config.has_target_graph() &&
      (!seeded || !config.target_graph().assume_deterministic_map() ||
       input.has_regional_policy_csv() || input.has_recorded_ack_session() ||
       input.gnss_map_construction() != NativeGlobalGnssInput::ABSENT))
    throw std::invalid_argument(
        "shared graph map requires an explicit fixed hypothesis and deterministic "
        "map independent of GNSS; regional/independent raw-cloud mode is disabled");
  if (input.has_regional_policy_csv()) {
    if (!input.has_regional_policy_sha256() ||
        input.has_initial_map_from_odom() || input.has_hypothesis_minimum_ns() ||
        input.has_hypothesis_maximum_ns() || input.has_hypothesis_odometry_session() ||
        input.has_hypothesis_odometry_generation())
      throw std::invalid_argument("regional map policy is exclusive with pose hypothesis");
  } else if (!seeded || input.has_regional_policy_sha256() ||
             input.hypothesis_minimum_ns() <= 0 ||
             input.hypothesis_maximum_ns() < input.hypothesis_minimum_ns() ||
             input.hypothesis_odometry_session().empty() ||
             input.hypothesis_odometry_generation() == 0) {
    throw std::invalid_argument("complete explicit map hypothesis or regional policy required");
  }
  imu_from_lidar_ = Pose(input.imu_from_lidar());
  if (seeded) initial_map_from_odom_ = Pose(input.initial_map_from_odom());
  if (!std::filesystem::is_regular_file(input.map_pcd()) ||
      std::filesystem::file_size(input.map_pcd()) > 128 * 1024 * 1024 ||
      Sha256(input.map_pcd()) != input.map_sha256())
    throw std::invalid_argument("online map file size/hash mismatch");
  CheckPcdHeader(input.map_pcd());
  map::PointCloud::Ptr cloud(new map::PointCloud);
  if (pcl::io::loadPCDFile(input.map_pcd(), *cloud) != 0 ||
      cloud->empty() || cloud->size() > 2000000 ||
      Sha256(input.map_pcd()) != input.map_sha256())
    throw std::runtime_error("invalid bounded or changed online map PCD");
  for (const auto& point : *cloud) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
      throw std::invalid_argument("online map contains nonfinite point");
  }
  map::ScanToMapConfig policy;
  policy.map_frame = "diagnostic-global-xy";
  policy.base_frame = policy.cloud_frame = "imu";
  policy.calibration_id = config.calibration_id();
  policy.algorithm = map::ScanMatchingAlgorithm::kPointToPlaneIcp;
  policy.local_window_radius_m = input.window_radius_m();
  policy.window_cell_size_m = input.window_cell_m();
  if (config.has_target_graph()) {
    const auto& graph = config.target_graph();
    if (!std::isfinite(graph.scan_voxel_m()) || graph.scan_voxel_m() <= 0 ||
        graph.minimum_scan_points() < 6 || graph.minimum_scan_points() > 12000 ||
        graph.normal_neighbors() < 3 || graph.normal_neighbors() > 100)
      throw std::invalid_argument("invalid bounded shared ICP geometry profile");
    policy.retain_joint_error_support = true;
    policy.scan_voxel_size_m = graph.scan_voxel_m();
    policy.minimum_points = graph.minimum_scan_points();
    policy.icp_minimum_pairs = graph.minimum_scan_points();
    policy.normal_neighbors = graph.normal_neighbors();
  }
  impl_->matcher = std::make_unique<map::ScanToMapLocalizer>(policy);
  map::MapMetadata metadata;
  metadata.map_id = config.reference_id();
  metadata.version = config.reference_version();
  metadata.content_id = config.content_id();
  metadata.map_frame = policy.map_frame;
  metadata.source_cloud_frame = "imu";
  metadata.reference_state = map::MapReferenceState::kDiagnosticOnly;
  metadata.integrity_verified = true;
  std::string error;
  if (!impl_->matcher->Configure(metadata, cloud, &error))
    throw std::invalid_argument(error);
  if (input.has_regional_policy_csv()) {
    const auto& path = input.regional_policy_csv();
    if (!std::filesystem::is_regular_file(path) ||
        std::filesystem::file_size(path) > 8192 ||
        Sha256(path) != input.regional_policy_sha256())
      throw std::invalid_argument("regional map policy size/hash mismatch");
    std::ifstream file(path);
    std::string line;
    if (!std::getline(file, line))
      throw std::invalid_argument("missing regional map policy header");
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line != "name,value")
      throw std::invalid_argument("regional map policy header mismatch");
    std::map<std::string, double> values;
    while (std::getline(file, line)) {
      if (!line.empty() && line.back() == '\r') line.pop_back();
      const auto comma = line.find(',');
      if (comma == std::string::npos || values.size() >= 32)
        throw std::invalid_argument("invalid bounded regional policy row");
      const auto name = line.substr(0, comma), number = line.substr(comma + 1);
      size_t consumed = 0;
      const auto value = std::stod(number, &consumed);
      if (consumed != number.size() || !std::isfinite(value) ||
          !values.emplace(name, value).second)
        throw std::invalid_argument("invalid/duplicate regional policy value");
    }
    if (!file.eof() || Sha256(path) != input.regional_policy_sha256())
      throw std::runtime_error("incomplete/changed regional map policy");
    auto take = [&values](const std::string& name) {
      const auto found = values.find(name);
      if (found == values.end())
        throw std::invalid_argument("missing regional policy: " + name);
      const auto value = found->second;
      values.erase(found);
      return value;
    };
    auto vector = [&take](const std::string& prefix) {
      Eigen::Vector3d value;
      for (int axis = 0; axis < 3; ++axis)
        value[axis] = take(prefix + std::string(1, "xyz"[axis]));
      return value;
    };
    map::DiagnosticRegionConfig region;
    region.calibration_id = config.calibration_id();
    region.field_minimum = vector("field_min_");
    region.field_maximum = vector("field_max_");
    region.search_minimum = vector("search_min_");
    region.search_maximum = vector("search_max_");
    region.observed_minimum = vector("observed_min_");
    region.observed_maximum = vector("observed_max_");
    region.yaw_minimum = take("yaw_min_rad");
    region.yaw_maximum = take("yaw_max_rad");
    const double qx = take("tilt_qx"), qy = take("tilt_qy");
    const double qz = take("tilt_qz"), qw = take("tilt_qw");
    region.tilt = Eigen::Quaterniond(qw, qx, qy, qz);
    region.field_resolution_m = take("field_resolution_m");
    region.distance_sigma_m = take("distance_sigma_m");
    region.inlier_distance_m = take("inlier_distance_m");
    if (!values.empty()) throw std::invalid_argument("unknown regional map policy key");
    impl_->search = std::make_unique<map::DiagnosticRegionalSearch>(region, metadata, *cloud);
  }
}

NativeMapMatch NativeMapMatcher::Match(const NativeMapJob& job) {
  if (job.joint_scan) return MatchShared(job);
  if (config_.has_target_graph())
    throw std::invalid_argument("target graph cannot use independently decoded raw scans");
  NativeMapMatch result;
  const auto& scan = job.cloud;
  result.sequence = scan.header().sequence_num();
  result.measurement_ns = job.reference_ns ? job.reference_ns : Reference(scan);
  result.receive_ns = job.receive_ns;
  const auto& config = config_.online_map();
  if (!scan.has_header() || result.sequence == 0 ||
      scan.frame_id() != config.lidar_frame() || scan.point_size() == 0 ||
      scan.point_size() > 12000 || job.session.empty() || job.generation == 0 ||
      job.receive_ns <= 0 || job.receive_ns - result.measurement_ns > 2000000000LL ||
      result.measurement_ns - job.receive_ns > 20000000LL)
    throw std::invalid_argument("invalid raw map scan identity/time/size");
  const auto end = Lookup(job.history, result.measurement_ns);
  if (!end) {
    result.reason = "map_deskew_reference_unavailable";
    return result;
  }
  map::PointCloud::Ptr cloud(new map::PointCloud);
  cloud->reserve(scan.point_size());
  int64_t previous = 0;
  const Eigen::Isometry3d end_from_odom = end->inverse();
  for (const auto& point : scan.point()) {
    if (!point.has_timestamp() || !point.has_x() || !point.has_y() ||
        !point.has_z() || point.timestamp() == 0 ||
        point.timestamp() > static_cast<uint64_t>(result.measurement_ns) ||
        point.timestamp() < static_cast<uint64_t>(previous) ||
        !std::isfinite(point.x()) || !std::isfinite(point.y()) ||
        !std::isfinite(point.z()))
      throw std::invalid_argument("invalid raw map point geometry/time");
    previous = point.timestamp();
    if (point.x() == 0 && point.y() == 0 && point.z() == 0) continue;
    const auto pose = Lookup(job.history, previous);
    if (!pose) {
      result.reason = "map_deskew_history_unavailable";
      return result;
    }
    const Eigen::Vector3d position = end_from_odom * *pose * imu_from_lidar_ *
        Eigen::Vector3d(point.x(), point.y(), point.z());
    cloud->emplace_back(position.x(), position.y(), position.z());
  }
  if (cloud->empty()) throw std::invalid_argument("raw map scan has no returns");
  auto hypothesis = job.map_from_odom;
  map::ScanToMapRequest request;
  request.timestamp_ns = result.measurement_ns;
  request.clock_domain = "unix";
  request.cloud_frame = "imu";
  request.deskewed = true;
  request.cloud = cloud;
  const bool recovery = impl_->search && !hypothesis;
  if (impl_->search) {
    if (job.session != impl_->session || job.generation != impl_->generation ||
        job.alignment_session != impl_->alignment_session ||
        job.alignment_generation != impl_->alignment_generation ||
        (impl_->tracking && !hypothesis))
      impl_->search->Restart();
    impl_->session = job.session;
    impl_->generation = job.generation;
    impl_->alignment_session = job.alignment_session;
    impl_->alignment_generation = job.alignment_generation;
    impl_->tracking = hypothesis.has_value();
    if (!hypothesis) {
      ::openloc::localization::unified::LocalState state;
      state.stamp = {result.measurement_ns / 1e9, job.receive_ns / 1e9,
                     result.sequence, "unix"};
      state.epoch = {job.session, job.generation};
      state.position = end->translation();
      state.orientation = Eigen::Quaterniond(end->linear());
      state.valid = true;
      state.covariance_model_valid = false;
      state.uncertainty_qualified = false;
      const auto proposal = impl_->search->Observe(state, result.sequence, request,
                                                  impl_->matcher.get());
      result.search_detail = "regional=" + proposal.status +
          " modes=" + std::to_string(proposal.modes) +
          " verified=" + std::to_string(proposal.verified_modes) +
          " stable=" + std::to_string(proposal.stable_tracks) +
          " truncated=" + std::to_string(proposal.truncated) +
          " attempt=" + std::to_string(proposal.attempt);
      if (!proposal.map_from_base) {
        result.reason = proposal.status;
        return result;
      }
      hypothesis = *proposal.map_from_base * end->inverse();
    }
  } else if (!hypothesis) {
    if (job.session != config.hypothesis_odometry_session() ||
        job.generation != config.hypothesis_odometry_generation()) {
      result.reason = "explicit_map_hypothesis_epoch_mismatch";
      return result;
    }
    if (result.measurement_ns < config.hypothesis_minimum_ns() ||
        result.measurement_ns > config.hypothesis_maximum_ns()) {
      result.reason = "explicit_map_hypothesis_expired";
      return result;
    }
    hypothesis = initial_map_from_odom_;
  }
  request.map_from_base_initial = *hypothesis * *end;
  const auto match = impl_->matcher->Match(request);
  result.reason = match.reason + ":" + match.detail;
  if (!match.valid || match.ambiguous) return result;
  NativeGlobalMapInput input;
  input.set_session(job.session);
  input.set_generation(job.generation);
  input.set_sequence(result.sequence);
  input.set_measurement_ns(result.measurement_ns);
  input.set_receive_ns(result.receive_ns);
  input.set_reference_id(config_.reference_id());
  input.set_reference_version(config_.reference_version());
  input.set_calibration_id(config_.calibration_id());
  input.set_content_id(config_.content_id());
  input.set_preparation_id(config.preparation_id());
  input.set_covariance_source(match.covariance_source);
  input.set_recorded_input(config_.recorded_input());
  input.set_diagnostic_only(true);
  input.set_ambiguous(false);
  input.set_recovery(recovery);
  input.set_odometry_initialization(NativeGlobalGnssInput::PRESENT);
  input.set_odometry_deskew(NativeGlobalGnssInput::PRESENT);
  input.set_shared_local_scans(NativeGlobalGnssInput::PRESENT);
  input.set_gnss_map_construction(config.gnss_map_construction());
  input.set_alignment_session(job.alignment_session);
  input.set_alignment_generation(job.alignment_generation);
  auto* pose = input.mutable_map_from_imu();
  pose->set_x(match.map_from_base.translation().x());
  pose->set_y(match.map_from_base.translation().y());
  pose->set_z(match.map_from_base.translation().z());
  const Eigen::Quaterniond q(match.map_from_base.linear());
  pose->set_qx(q.x()); pose->set_qy(q.y()); pose->set_qz(q.z()); pose->set_qw(q.w());
  if (match.geometry_full_rank) {
    input.set_kind(NativeGlobalMapInput::FULL_POSE);
    for (int r = 0; r < 6; ++r)
      for (int c = 0; c < 6; ++c) input.add_covariance(match.covariance(r, c));
  } else {
    input.set_kind(NativeGlobalMapInput::PROJECTED_POSE);
    input.set_rank(match.observed_projection.rows());
    for (int r = 0; r < match.observed_projection.rows(); ++r) {
      for (int c = 0; c < 6; ++c)
        input.add_projection(match.observed_projection(r, c));
      for (int c = 0; c < match.observed_covariance.cols(); ++c)
        input.add_projected_covariance(match.observed_covariance(r, c));
    }
  }
  result.observation = std::move(input);
  return result;
}

NativeMapMatch NativeMapMatcher::MatchShared(const NativeMapJob& job) {
  if (!config_.has_target_graph() || !job.joint_scan ||
      !job.cloud.point().empty() || job.map_from_odom)
    throw std::invalid_argument("shared native map mode/source mismatch");
  const auto& scan = *job.joint_scan;
  const auto& state = scan.conditioning_state;
  const auto& config = config_.online_map();
  NativeMapMatch result;
  result.sequence = scan.reference.sequence;
  result.measurement_ns = Nanoseconds(scan.reference.time);
  result.receive_ns = job.receive_ns;
  if (!state.joint_error || state.epoch.session != job.session ||
      state.epoch.generation != job.generation || job.reference_ns != result.measurement_ns ||
      result.receive_ns <= 0 || result.sequence == 0)
    throw std::invalid_argument("shared map source identity mismatch");
  if (job.session != config.hypothesis_odometry_session() ||
      job.generation != config.hypothesis_odometry_generation()) {
    result.reason = "explicit_map_hypothesis_epoch_mismatch";
    return result;
  }
  if (result.measurement_ns < config.hypothesis_minimum_ns() ||
      result.measurement_ns > config.hypothesis_maximum_ns()) {
    result.reason = "explicit_map_hypothesis_expired";
    return result;
  }
  map::ScanToMapRequest request;
  request.timestamp_ns = result.measurement_ns;
  request.clock_domain = "unix";
  request.cloud_frame = "imu";
  request.deskewed = true;
  request.joint_scan = &scan;
  Eigen::Isometry3d conditioning_pose = Eigen::Isometry3d::Identity();
  conditioning_pose.translation() = state.position;
  conditioning_pose.linear() = state.orientation.toRotationMatrix();
  // A fixed configured alignment applied to the conditioning ODOM pose.
  // This is not a graph/GNSS posterior seed. Its complete local loading is
  // retained even when the actual scan reference precedes this error node.
  request.map_from_base_initial = initial_map_from_odom_ * conditioning_pose;
  Eigen::Matrix<double, 6, 15> seed = Eigen::Matrix<double, 6, 15>::Zero();
  seed.topLeftCorner<3, 3>() = initial_map_from_odom_.linear();
  seed.block<3, 3>(3, 6) =
      initial_map_from_odom_.linear() * conditioning_pose.linear();
  request.initial_joint_jacobian = seed * state.joint_error->output_jacobian;
  const auto match = impl_->matcher->Match(request);
  result.reason = match.reason + ":" + match.detail;
  if (!match.valid || match.ambiguous) return result;
  if (!match.geometry_full_rank || !match.joint_error ||
      !match.joint_error->assessment.ok()) {
    result.reason = match.joint_error ? match.joint_error->assessment.message
                                     : "shared_map_error_model_unavailable";
    return result;
  }
  NativeGlobalMapInput input;
  input.set_session(job.session);
  input.set_generation(job.generation);
  input.set_sequence(result.sequence);
  input.set_measurement_ns(result.measurement_ns);
  input.set_receive_ns(result.receive_ns);
  input.set_reference_id(config_.reference_id());
  input.set_reference_version(config_.reference_version());
  input.set_calibration_id(config_.calibration_id());
  input.set_content_id(config_.content_id());
  input.set_preparation_id(config.preparation_id());
  input.set_covariance_source("native-joint-frozen-icp-deterministic-map-uncalibrated");
  input.set_recorded_input(config_.recorded_input());
  input.set_diagnostic_only(true);
  input.set_ambiguous(false);
  input.set_recovery(false);
  input.set_kind(NativeGlobalMapInput::FULL_POSE);
  input.set_odometry_initialization(NativeGlobalGnssInput::PRESENT);
  input.set_odometry_deskew(NativeGlobalGnssInput::PRESENT);
  input.set_shared_local_scans(NativeGlobalGnssInput::PRESENT);
  input.set_gnss_map_construction(NativeGlobalGnssInput::ABSENT);
  input.set_alignment_session(job.alignment_session);
  input.set_alignment_generation(job.alignment_generation);
  auto* pose = input.mutable_map_from_imu();
  pose->set_x(match.map_from_base.translation().x());
  pose->set_y(match.map_from_base.translation().y());
  pose->set_z(match.map_from_base.translation().z());
  const Eigen::Quaterniond q(match.map_from_base.linear());
  pose->set_qx(q.x()); pose->set_qy(q.y()); pose->set_qz(q.z()); pose->set_qw(q.w());
  for (int row = 0; row < 6; ++row)
    for (int col = 0; col < 6; ++col)
      input.add_covariance(match.joint_error->covariance(row, col));
  auto* qualification = input.mutable_graph_qualification();
  qualification->set_uncertainty_model_id(input.covariance_source());
  qualification->set_independence_evidence_id("");
  qualification->set_quality_valid(true);
  qualification->set_uncertainty_qualified(false);
  qualification->set_independent_of_local(false);
  qualification->set_independent_of_graph_sources(false);
  auto* correlation = qualification->mutable_local_correlation();
  correlation->set_motion_error_model_id(config_.target_graph().motion_error_model_id());
  correlation->set_conditional_independence_evidence_id(
      "actual-native-deskew-frozen-icp-deterministic-map:" + config_.content_id());
  correlation->set_odometry_sequence(state.stamp.sequence);
  correlation->set_conditioning_ns(Nanoseconds(state.stamp.time));
  correlation->set_dimension(state.joint_error->covariance.rows());
  correlation->set_conditional_model_qualified(true);
  correlation->set_residual_independent_of_history_and_sources(true);
  const auto& cross = match.joint_error->local_cross_covariance;
  for (Eigen::Index row = 0; row < cross.rows(); ++row)
    for (int col = 0; col < 6; ++col)
      correlation->add_cross_covariance(cross(row, col));
  result.observation = std::move(input);
  return result;
}

}  // namespace apollo::localization::openloc_diagnostic
