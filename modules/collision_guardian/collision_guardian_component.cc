#include "modules/collision_guardian/collision_guardian_component.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "cyber/common/log.h"
#include "cyber/time/time.h"
#include "modules/common/util/message_util.h"

namespace apollo {
namespace collision_guardian {
namespace {

constexpr uint32_t kPointCloudSource = 1U;
constexpr uint32_t kOccupancyGridSource = 2U;

double PointCloudTimestamp(const drivers::PointCloud& pointcloud) {
  if (pointcloud.has_measurement_time()) {
    return pointcloud.measurement_time();
  }
  return pointcloud.header().timestamp_sec();
}

}  // namespace

bool CollisionGuardianComponent::Init() {
  if (!GetProtoConfig(&config_)) {
    AERROR << "Failed to load collision guardian config: "
           << ConfigFilePath();
    return false;
  }
  if (!ValidateConfig()) {
    return false;
  }

  OccupancyDetectorConfig detector_config;
  detector_config.ego_forward_m = config_.ego_forward_m();
  detector_config.ego_backward_m = config_.ego_backward_m();
  detector_config.ego_half_width_m = config_.ego_half_width_m();
  detector_config.roi_forward_m = config_.roi_forward_m();
  detector_config.roi_backward_m = config_.roi_backward_m();
  detector_config.roi_half_width_m = config_.roi_half_width_m();
  detector_config.min_height_m = config_.min_height_m();
  detector_config.max_height_m = config_.max_height_m();
  detector_config.voxel_size_x_m = config_.voxel_size_x_m();
  detector_config.voxel_size_y_m = config_.voxel_size_y_m();
  detector_config.voxel_size_z_m = config_.voxel_size_z_m();
  detector_config.y_axis_is_forward = config_.y_axis_is_forward();
  detector_ = std::make_unique<OccupancyDetector>(detector_config);

  RiskFilterConfig filter_config;
  filter_config.prior_probability = config_.prior_probability();
  filter_config.hit_probability = config_.hit_probability();
  filter_config.miss_probability = config_.miss_probability();
  filter_config.temporal_decay = config_.temporal_decay();
  filter_config.suspected_probability = config_.suspected_probability();
  filter_config.trigger_probability = config_.trigger_probability();
  filter_config.release_probability = config_.release_probability();
  filter_config.min_confirmation_frames = config_.min_confirmation_frames();
  filter_config.min_release_frames = config_.min_release_frames();
  risk_filter_ = std::make_unique<RiskFilter>(filter_config);

  if (config_.enable_pointcloud()) {
    pointcloud_reader_ = node_->CreateReader<drivers::PointCloud>(
        config_.pointcloud_topic(),
        [this](const std::shared_ptr<drivers::PointCloud>& message) {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_pointcloud_ = message;
        });
  }
  if (config_.enable_occupancy_grid()) {
    occupancy_grid_reader_ = node_->CreateReader<OccupancyGrid>(
        config_.occupancy_grid_topic(),
        [this](const std::shared_ptr<OccupancyGrid>& message) {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_occupancy_grid_ = message;
        });
  }
  risk_writer_ = node_->CreateWriter<CollisionRisk>(config_.output_topic());
  return true;
}

bool CollisionGuardianComponent::Proc() {
  std::shared_ptr<drivers::PointCloud> pointcloud;
  std::shared_ptr<OccupancyGrid> grid;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pointcloud = latest_pointcloud_;
    grid = latest_occupancy_grid_;
  }

  std::vector<Eigen::Vector3d> points;
  double source_timestamp_sec = 0.0;
  uint32_t source_mask = 0U;
  bool attempted_input = false;
  bool input_valid = false;

  if (pointcloud != nullptr) {
    const double timestamp = PointCloudTimestamp(*pointcloud);
    if (timestamp > last_pointcloud_timestamp_sec_) {
      attempted_input = true;
      last_pointcloud_timestamp_sec_ = timestamp;
      if (AppendPointCloud(*pointcloud, &points, &source_timestamp_sec)) {
        source_mask |= kPointCloudSource;
        input_valid = true;
      }
    }
  }

  if (grid != nullptr) {
    const double timestamp = grid->header().timestamp_sec();
    if (timestamp > last_grid_timestamp_sec_) {
      attempted_input = true;
      last_grid_timestamp_sec_ = timestamp;
      if (AppendOccupancyGrid(*grid, &points, &source_timestamp_sec)) {
        source_mask |= kOccupancyGridSource;
        input_valid = true;
      }
    }
  }

  const double now_sec = cyber::Time::Now().ToSecond();
  if (!attempted_input) {
    if (!fault_published_ &&
        now_sec - last_valid_input_time_sec_ >
            config_.input_stale_timeout_sec()) {
      fault_published_ = true;
      const auto filter_result = risk_filter_->Update(false, false);
      PublishRisk(filter_result, OccupancyResult{}, false, 0.0, 0U);
    }
    return true;
  }

  if (!input_valid) {
    fault_published_ = true;
    const auto filter_result = risk_filter_->Update(false, false);
    PublishRisk(filter_result, OccupancyResult{}, false, source_timestamp_sec,
                source_mask);
    return true;
  }

  last_valid_input_time_sec_ = now_sec;
  fault_published_ = false;
  const OccupancyResult occupancy_result = detector_->Evaluate(points);
  const bool occupied =
      occupancy_result.occupied_voxel_count >= config_.min_occupied_voxels();
  const auto filter_result = risk_filter_->Update(occupied, true);
  PublishRisk(filter_result, occupancy_result, true, source_timestamp_sec,
              source_mask);
  return true;
}

bool CollisionGuardianComponent::ValidateConfig() const {
  const auto valid_probability = [](double probability) {
    return probability > 0.0 && probability < 1.0;
  };
  if (!config_.enable_pointcloud() && !config_.enable_occupancy_grid()) {
    AERROR << "Collision guardian requires at least one input source.";
    return false;
  }
  if ((config_.enable_pointcloud() && config_.pointcloud_topic().empty()) ||
      (config_.enable_occupancy_grid() &&
       config_.occupancy_grid_topic().empty()) ||
      config_.output_topic().empty()) {
    AERROR << "Collision guardian has an enabled source with an empty topic.";
    return false;
  }
  if (config_.ego_forward_m() < 0.0 || config_.ego_backward_m() < 0.0 ||
      config_.ego_half_width_m() < 0.0 ||
      config_.roi_forward_m() < config_.ego_forward_m() ||
      config_.roi_backward_m() < config_.ego_backward_m() ||
      config_.roi_half_width_m() < config_.ego_half_width_m() ||
      config_.min_height_m() >= config_.max_height_m() ||
      config_.voxel_size_x_m() <= 0.0 ||
      config_.voxel_size_y_m() <= 0.0 ||
      config_.voxel_size_z_m() <= 0.0 ||
      config_.occupied_probability_threshold() < 0.0 ||
      config_.occupied_probability_threshold() > 1.0 ||
      config_.min_occupied_voxels() == 0 ||
      config_.min_confirmation_frames() == 0 ||
      config_.min_release_frames() == 0 ||
      config_.input_stale_timeout_sec() <= 0.0 ||
      config_.transform_timeout_sec() < 0.0) {
    AERROR << "Collision guardian spatial or temporal config is invalid.";
    return false;
  }
  if (!valid_probability(config_.prior_probability()) ||
      !valid_probability(config_.hit_probability()) ||
      !valid_probability(config_.miss_probability()) ||
      !valid_probability(config_.suspected_probability()) ||
      !valid_probability(config_.trigger_probability()) ||
      !valid_probability(config_.release_probability()) ||
      config_.temporal_decay() < 0.0 || config_.temporal_decay() > 1.0 ||
      config_.release_probability() >= config_.suspected_probability() ||
      config_.suspected_probability() >= config_.trigger_probability()) {
    AERROR << "Collision guardian Bayesian thresholds are invalid.";
    return false;
  }
  if (config_.use_case() == USE_CASE_UNKNOWN) {
    AERROR << "Collision guardian requires an explicit special-scene use case.";
    return false;
  }
  return true;
}

bool CollisionGuardianComponent::AppendPointCloud(
    const drivers::PointCloud& pointcloud,
    std::vector<Eigen::Vector3d>* points, double* timestamp_sec) {
  const double timestamp = PointCloudTimestamp(pointcloud);
  const std::string source_frame = pointcloud.header().has_frame_id()
                                       ? pointcloud.header().frame_id()
                                       : pointcloud.frame_id();
  Eigen::Affine3d sensor_to_vehicle;
  if (!QueryTransform(timestamp, source_frame, &sensor_to_vehicle)) {
    return false;
  }

  points->reserve(points->size() + pointcloud.point_size());
  for (const auto& point : pointcloud.point()) {
    if (!std::isfinite(point.x()) || !std::isfinite(point.y()) ||
        !std::isfinite(point.z())) {
      continue;
    }
    points->emplace_back(
        sensor_to_vehicle * Eigen::Vector3d(point.x(), point.y(), point.z()));
  }
  *timestamp_sec = std::max(*timestamp_sec, timestamp);
  return true;
}

bool CollisionGuardianComponent::AppendOccupancyGrid(
    const OccupancyGrid& grid, std::vector<Eigen::Vector3d>* points,
    double* timestamp_sec) {
  if (grid.resolution() <= 0.0 || grid.width() == 0 || grid.height() == 0) {
    AERROR << "Received an invalid occupancy grid.";
    return false;
  }
  Eigen::Affine3d grid_to_vehicle;
  if (!QueryTransform(grid.header().timestamp_sec(), grid.header().frame_id(),
                      &grid_to_vehicle)) {
    return false;
  }

  points->reserve(points->size() + grid.occupied_cell_size());
  for (const auto& cell : grid.occupied_cell()) {
    if (cell.x_index() >= grid.width() || cell.y_index() >= grid.height() ||
        cell.occupancy_probability() <
            config_.occupied_probability_threshold()) {
      continue;
    }
    const double min_height = cell.has_min_height() ? cell.min_height() : 0.0;
    const double max_height = cell.has_max_height() ? cell.max_height() : 0.0;
    if (min_height > max_height) {
      continue;
    }
    const Eigen::Vector3d center(
        grid.origin_x() + (cell.x_index() + 0.5) * grid.resolution(),
        grid.origin_y() + (cell.y_index() + 0.5) * grid.resolution(),
        0.5 * (min_height + max_height));
    points->emplace_back(grid_to_vehicle * center);
  }
  *timestamp_sec =
      std::max(*timestamp_sec, grid.header().timestamp_sec());
  return true;
}

bool CollisionGuardianComponent::QueryTransform(
    double timestamp_sec, const std::string& source_frame,
    Eigen::Affine3d* transform) {
  if (source_frame.empty()) {
    AERROR << "Collision guardian input has no frame id.";
    return false;
  }
  std::string error;
  if (!transform_query_.LookupTransformToAffine(
          config_.vehicle_frame_id(), source_frame, cyber::Time(timestamp_sec),
          transform, static_cast<float>(config_.transform_timeout_sec()),
          &error)) {
    AERROR << "Failed to transform collision input from " << source_frame
           << " to " << config_.vehicle_frame_id() << ": " << error;
    return false;
  }
  return true;
}

void CollisionGuardianComponent::PublishRisk(
    const RiskFilterResult& filter_result,
    const OccupancyResult& occupancy_result, bool input_valid,
    double source_timestamp_sec, uint32_t source_mask) {
  auto risk = std::make_shared<CollisionRisk>();
  common::util::FillHeader("collision_guardian", risk.get());
  risk->mutable_header()->set_frame_id(config_.vehicle_frame_id());
  risk->set_state(ToProtoState(filter_result.state));
  risk->set_zone(DetermineZone(occupancy_result));
  risk->set_use_case(config_.use_case());
  risk->set_risk_probability(filter_result.probability);
  if (std::isfinite(occupancy_result.nearest_distance_m)) {
    risk->set_nearest_distance_m(occupancy_result.nearest_distance_m);
  }
  risk->set_occupied_voxel_count(occupancy_result.occupied_voxel_count);
  risk->set_input_valid(input_valid);
  risk->set_source_timestamp_sec(source_timestamp_sec);
  risk->set_source_mask(source_mask);
  risk_writer_->Write(risk);
}

RiskState CollisionGuardianComponent::ToProtoState(FilterState state) {
  switch (state) {
    case FilterState::kClear:
      return RISK_STATE_CLEAR;
    case FilterState::kSuspected:
      return RISK_STATE_SUSPECTED;
    case FilterState::kConfirmed:
      return RISK_STATE_CONFIRMED;
    case FilterState::kReleasing:
      return RISK_STATE_RELEASING;
    case FilterState::kFault:
      return RISK_STATE_FAULT;
  }
  return RISK_STATE_UNKNOWN;
}

RiskZone CollisionGuardianComponent::DetermineZone(
    const OccupancyResult& result) {
  if (!std::isfinite(result.nearest_distance_m)) {
    return RISK_ZONE_UNKNOWN;
  }
  if (std::abs(result.nearest_forward_m) >=
      std::abs(result.nearest_lateral_m)) {
    return result.nearest_forward_m >= 0.0 ? RISK_ZONE_FRONT : RISK_ZONE_REAR;
  }
  return result.nearest_lateral_m >= 0.0 ? RISK_ZONE_LEFT : RISK_ZONE_RIGHT;
}

}  // namespace collision_guardian
}  // namespace apollo
