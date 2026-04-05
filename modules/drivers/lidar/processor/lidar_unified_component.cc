#include "modules/drivers/lidar/processor/lidar_unified_component.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

#include "modules/drivers/lidar/processor/policy/lidar_policy_common.h"

namespace apollo {
namespace drivers {
namespace lidar {

namespace {

std::string ResolvePolicyMode(const LidarUnifiedComponentConfig& config) {
#ifdef APOLLO_LIDAR_POLICY_FORCE_CPU
  (void)config;
  return "cpu";
#endif
#ifdef APOLLO_LIDAR_POLICY_FORCE_GPU
  (void)config;
  return "gpu";
#endif
  switch (config.compute_mode()) {
    case LidarUnifiedComponentConfig::COMPUTE_MODE_GPU:
      return "gpu";
    case LidarUnifiedComponentConfig::COMPUTE_MODE_CPU:
    default:
      return "cpu";
  }
}

apollo::transform::PoseCacheOptions BuildPoseCacheOptions(
    const LidarUnifiedComponentConfig& config) {
  apollo::transform::PoseCacheOptions options;
  options.cache_duration_sec = config.sensor_pose_cache_duration_sec();
  options.max_extrapolation_sec =
      config.sensor_pose_cache_max_extrapolation_sec();
  options.query_timeout_sec =
      static_cast<float>(config.sensor_pose_query_timeout_sec());
  return options;
}

}  // namespace

bool LidarUnifiedComponent::Init() {
  if (!GetProtoConfig(&config_)) {
    AERROR << "Load config failed, config file: " << ConfigFilePath();
    return false;
  }

  if (!ValidateConfig()) {
    return false;
  }

  writer_ =
      node_->CreateWriter<::apollo::drivers::PointCloud>(config_.output_channel());
  tf_buffer_ = apollo::transform::Buffer::Instance();

  const std::string policy_mode = ResolvePolicyMode(config_);
  deskew_policy_ = LidarPolicyFactory::CreateDeskewPolicy(policy_mode);
  fusion_policy_ = LidarPolicyFactory::CreateFusionPolicy(policy_mode);
  filter_policy_ = LidarPolicyFactory::CreateFilterPolicy(policy_mode);
  if (deskew_policy_ == nullptr || fusion_policy_ == nullptr ||
      filter_policy_ == nullptr) {
    AERROR << "Failed to create lidar policies for mode=" << policy_mode;
    return false;
  }
  if (!deskew_policy_->Init(config_, tf_buffer_) ||
      !fusion_policy_->Init(config_, tf_buffer_) ||
      !filter_policy_->Init(config_)) {
    AERROR << "Failed to initialize lidar policies for mode=" << policy_mode;
    return false;
  }

  ts_sanity_.SetConfig(config_.ts_sanity_min_interval_ms(),
                       config_.ts_sanity_max_interval_ms(),
                       config_.ts_sanity_max_jump_ms());
  degrade_policy_.SetConfig(config_.degrade_on_ts_anomaly(),
                            config_.ts_sanity_max_consecutive_errors());

  sensor_buffer_capacity_ =
      std::max<size_t>(2, static_cast<size_t>(config_.sensor_buffer_size()));

  auxiliary_inputs_.clear();
  auxiliary_readers_.clear();
  auxiliary_sensor_ids_by_topic_.clear();
  sensor_states_.clear();
  primary_sensor_id_.clear();
  base_link_pose_cache_ = std::make_unique<apollo::transform::TransformFrameCache>(
      tf_buffer_, config_.world_frame_id(), config_.base_link_frame_id(),
      BuildPoseCacheOptions(config_));

  for (const auto& input_cfg : config_.auxiliary_lidar_inputs()) {
    auxiliary_inputs_.push_back(SensorInput{input_cfg.topic_name()});

    auto reader = node_->CreateReader<::apollo::drivers::PointCloud>(
        input_cfg.topic_name(), [this, topic_name = input_cfg.topic_name()](
                                    const std::shared_ptr<::apollo::drivers::PointCloud>& msg) {
          OnAuxiliaryLidarMessage(topic_name, msg);
        });
    auxiliary_readers_.emplace(input_cfg.topic_name(), reader);
  }

  const size_t max_points = std::max<size_t>(
      1, static_cast<size_t>(config_.max_full_pointcloud_points()));
  full_pointcloud_buffer_.resize(max_points);

  AINFO << "LidarUnifiedComponent initialized. compute_mode=" << policy_mode
        << ", main_sensor=<auto>"
        << ", aux_count=" << auxiliary_inputs_.size()
        << ", max_points=" << max_points;
  return true;
}

bool LidarUnifiedComponent::Proc(
  const std::shared_ptr<::apollo::drivers::PointCloud>& point_cloud) {
  return OnReceiveMainLidar(point_cloud);
}

bool LidarUnifiedComponent::OnReceiveMainLidar(
    const PointCloudConstPtr& point_cloud) {
  if (point_cloud == nullptr) {
    AERROR << "Input point cloud is null";
    return false;
  }

  const std::string sensor_id = ResolveSensorId(point_cloud, std::string());
  if (sensor_id.empty()) {
    AERROR << "Failed to resolve primary lidar sensor id from incoming frame";
    return false;
  }

  if (primary_sensor_id_.empty()) {
    primary_sensor_id_ = sensor_id;
    AINFO << "Resolved primary lidar sensor id from input stream: "
          << primary_sensor_id_;
  } else if (primary_sensor_id_ != sensor_id) {
    AERROR << "Primary lidar sensor id changed from " << primary_sensor_id_
           << " to " << sensor_id << ", drop frame to avoid TF mismatch";
    return false;
  }

  std::shared_ptr<BufferedFrame> buffered_frame;
  if (!PrepareBufferedFrame(primary_sensor_id_, point_cloud, &buffered_frame)) {
    AERROR << "Failed to prefetch motion poses for primary lidar "
           << primary_sensor_id_;
    return false;
  }
  PushToBuffer(primary_sensor_id_, buffered_frame);

  if (config_.ts_sanity_enabled()) {
    const TsSanityResult ts_result =
        ts_sanity_.Check(point_cloud->measurement_time());
    if (ts_result.status != TsSanityStatus::kOk &&
        ts_result.status != TsSanityStatus::kFirstFrame) {
      dtc_reporter_.ReportTsAnomaly(ts_result.status, ts_result.interval_ms,
                                    ts_result.consecutive_errors,
                                    primary_sensor_id_);
      const DegradeEvent event =
          degrade_policy_.OnTsAnomaly(ts_result.consecutive_errors);
      dtc_reporter_.ReportDegradeTransition(event);
    }
  }

  FrameMetrics frame_metrics;
  std::vector<FrameHandle> frame_handles;
  if (!CollectNearestFrames(point_cloud->measurement_time(), primary_sensor_id_,
                            &frame_handles, &frame_metrics)) {
    AERROR << "Failed to collect nearest frames for ref timestamp "
           << point_cloud->measurement_time();
    return false;
  }

  for (const auto& frame_handle : frame_handles) {
    if (!frame_handle.is_primary) {
      UpdateSensorTimingModel(frame_handle, point_cloud->measurement_time(),
                              &frame_metrics);
    }
  }

  if (degrade_policy_.CurrentMode() == DegradeMode::kSingleLidar) {
    frame_handles.erase(
        std::remove_if(
            frame_handles.begin(), frame_handles.end(),
            [](const FrameHandle& handle) { return !handle.is_primary; }),
        frame_handles.end());
    frame_metrics.expected_sensor_count = 1;
    frame_metrics.matched_sensor_count = frame_handles.size();
    frame_metrics.missing_auxiliary_count = 0;
    frame_metrics.time_delta_exceeded_count = 0;
  }

  std::shared_ptr<::apollo::drivers::PointCloud> unified_output;
  if (!BuildUnifiedPointCloud(point_cloud, frame_handles, &frame_metrics,
                              &unified_output)) {
    AERROR << "Failed to build unified point cloud";
    return false;
  }

  dtc_reporter_.ReportDegradeTransition(degrade_policy_.OnFrameOk());

  LogFrameMetrics(frame_metrics);
  writer_->Write(unified_output);
  return true;
}

void LidarUnifiedComponent::PushToBuffer(
    const std::string& sensor_id,
    const std::shared_ptr<BufferedFrame>& buffered_frame) {
  const auto sensor_state = GetSensorState(sensor_id);
  if (sensor_state == nullptr || buffered_frame == nullptr) {
    return;
  }
  std::lock_guard<std::mutex> lock(sensor_state->mutex);
  sensor_state->frames.push_back(buffered_frame);
}

bool LidarUnifiedComponent::CollectNearestFrames(
    double ref_timestamp, const std::string& primary_sensor_id,
    std::vector<FrameHandle>* frame_handles, FrameMetrics* frame_metrics) {
  if (frame_handles == nullptr || frame_metrics == nullptr) {
    return false;
  }

  std::vector<std::string> auxiliary_topics;
  auxiliary_topics.reserve(auxiliary_inputs_.size());
  for (const auto& auxiliary_input : auxiliary_inputs_) {
    auxiliary_topics.push_back(auxiliary_input.topic_name);
  }

  SyncGateMetrics sync_metrics;
  const bool ok = sync_gate_.SelectFrames(
      ref_timestamp, primary_sensor_id, auxiliary_topics,
      config_.max_ref_time_delta_ms(), config_.strict_auxiliary_sync(),
      [this](const std::string& topic_name, std::string* sensor_id) {
        if (sensor_id == nullptr) {
          return false;
        }
        std::lock_guard<std::mutex> lock(sensor_registry_mutex_);
        const auto it = auxiliary_sensor_ids_by_topic_.find(topic_name);
        if (it == auxiliary_sensor_ids_by_topic_.end()) {
          return false;
        }
        *sensor_id = it->second;
        return !sensor_id->empty();
      },
      [this, primary_sensor_id](const std::string& sensor_id, double timestamp,
                                uint32_t max_delta_ms,
                                FrameHandle* frame_handle,
                                bool* time_delta_exceeded) {
        if (time_delta_exceeded != nullptr) {
          *time_delta_exceeded = false;
        }
        FrameLookupFailureReason failure_reason =
            FrameLookupFailureReason::kNone;
        const bool found = FindNearestFrame(GetSensorState(sensor_id), sensor_id,
                                            timestamp, max_delta_ms,
                                            frame_handle, &failure_reason);
        if (found && sensor_id != primary_sensor_id && frame_handle != nullptr &&
            frame_handle->overlap_quality_weight <
                config_.auxiliary_min_overlap_quality_weight()) {
          return false;
        }
        if (!found && time_delta_exceeded != nullptr) {
          *time_delta_exceeded =
              failure_reason == FrameLookupFailureReason::kTimeDeltaExceeded;
        }
        return found;
      },
      frame_handles, &sync_metrics);

  frame_metrics->expected_sensor_count = sync_metrics.expected_sensor_count;
  frame_metrics->matched_sensor_count = sync_metrics.matched_sensor_count;
  frame_metrics->missing_auxiliary_count = sync_metrics.missing_auxiliary_count;
  frame_metrics->time_delta_exceeded_count =
      sync_metrics.time_delta_exceeded_count;
  return ok;
}

bool LidarUnifiedComponent::FindNearestFrame(
    const std::shared_ptr<SensorState>& sensor_state,
    const std::string& sensor_id, double ref_timestamp,
    uint32_t max_ref_time_delta_ms, FrameHandle* frame_handle,
    FrameLookupFailureReason* failure_reason) const {
  if (frame_handle == nullptr || failure_reason == nullptr) {
    return false;
  }

  *failure_reason = FrameLookupFailureReason::kNone;
  if (sensor_state == nullptr) {
    *failure_reason = FrameLookupFailureReason::kBufferEmpty;
    return false;
  }

  std::lock_guard<std::mutex> lock(sensor_state->mutex);
  if (sensor_state->frames.empty()) {
    *failure_reason = FrameLookupFailureReason::kBufferEmpty;
    return false;
  }

  std::shared_ptr<const BufferedFrame> best_frame;
  double best_delta_ms = std::numeric_limits<double>::max();
  double best_timestamp = 0.0;
  constexpr double kTimestampTieEpsilonMs = 1e-6;
  const double fixed_delay_sec =
      sensor_state->fixed_delay_initialized ? sensor_state->fixed_delay_sec : 0.0;

  // Auxiliary lidar frames can arrive slightly out of order, so choose the
  // nearest sample with a linear scan instead of assuming a sorted buffer.
  for (const auto& candidate : sensor_state->frames) {
    if (candidate == nullptr || candidate->point_cloud == nullptr ||
        !candidate->pose_prefetch_ok) {
      continue;
    }

    const double candidate_timestamp = candidate->point_cloud->measurement_time();
    const double delta_ms =
        std::fabs((candidate_timestamp + fixed_delay_sec) - ref_timestamp) *
        1000.0;
    const bool same_delta =
      std::fabs(delta_ms - best_delta_ms) <= kTimestampTieEpsilonMs;
    if (delta_ms + kTimestampTieEpsilonMs < best_delta_ms ||
        (same_delta && candidate_timestamp < best_timestamp)) {
      best_frame = candidate;
      best_delta_ms = delta_ms;
      best_timestamp = candidate_timestamp;
    }
  }

  if (best_frame == nullptr) {
    *failure_reason = FrameLookupFailureReason::kBufferEmpty;
    return false;
  }

  if (best_delta_ms > static_cast<double>(max_ref_time_delta_ms)) {
    *failure_reason = FrameLookupFailureReason::kTimeDeltaExceeded;
    return false;
  }

  frame_handle->sensor_id = sensor_id;
  frame_handle->point_cloud = best_frame->point_cloud;
  frame_handle->buffered_frame = best_frame;
  frame_handle->clock_offset_residual_ms =
      (ref_timestamp - (best_frame->point_cloud->measurement_time() +
                        fixed_delay_sec)) *
      1000.0;
  frame_handle->overlap_quality_weight = sensor_state->overlap_quality_weight;
  return true;
}

bool LidarUnifiedComponent::ResolveWorldToBase(double ref_timestamp_sec,
                                               Eigen::Affine3d* world2base_ref) {
  if (world2base_ref == nullptr || base_link_pose_cache_ == nullptr) {
    return false;
  }

  if (!base_link_pose_cache_->Prefetch(ref_timestamp_sec)) {
    total_tf_query_failures_.fetch_add(1);
  }

  Eigen::Affine3d world_from_base = Eigen::Affine3d::Identity();
  if (!base_link_pose_cache_->QueryCached(ref_timestamp_sec, &world_from_base)) {
    total_tf_query_failures_.fetch_add(1);
    return false;
  }

  *world2base_ref = world_from_base.inverse();
  return true;
}

void LidarUnifiedComponent::UpdateSensorTimingModel(
    const FrameHandle& frame_handle, double ref_timestamp_sec,
    FrameMetrics* frame_metrics) {
  if (frame_handle.is_primary || frame_handle.point_cloud == nullptr) {
    return;
  }

  const auto sensor_state = GetSensorState(frame_handle.sensor_id);
  if (sensor_state == nullptr) {
    return;
  }

  const double observed_delay_sec =
      ref_timestamp_sec - frame_handle.point_cloud->measurement_time();
  std::lock_guard<std::mutex> lock(sensor_state->mutex);
  if (!sensor_state->fixed_delay_initialized) {
    sensor_state->fixed_delay_sec = observed_delay_sec;
    sensor_state->fixed_delay_initialized = true;
  } else if (std::fabs((observed_delay_sec - sensor_state->fixed_delay_sec) *
                       1000.0) <= config_.fixed_delay_update_limit_ms()) {
    const double alpha = config_.fixed_delay_ema_alpha();
    sensor_state->fixed_delay_sec =
        sensor_state->fixed_delay_sec * (1.0 - alpha) + observed_delay_sec * alpha;
  }

  if (sensor_state->timing_observation_count == 0U) {
    sensor_state->smoothed_clock_offset_ms = frame_handle.clock_offset_residual_ms;
  } else {
    const double alpha = config_.clock_offset_ema_alpha();
    sensor_state->smoothed_clock_offset_ms =
        sensor_state->smoothed_clock_offset_ms * (1.0 - alpha) +
        frame_handle.clock_offset_residual_ms * alpha;
  }
  sensor_state->last_clock_offset_ms = frame_handle.clock_offset_residual_ms;
  ++sensor_state->timing_observation_count;

  if (frame_metrics != nullptr) {
    frame_metrics->max_abs_clock_offset_ms = std::max(
        frame_metrics->max_abs_clock_offset_ms,
        std::fabs(frame_handle.clock_offset_residual_ms));
  }
}

void LidarUnifiedComponent::UpdateOverlapQualityWeights(
    const std::vector<FrameHandle>& frame_handles,
    const Eigen::Affine3d& world2base_ref, FrameMetrics* frame_metrics) {
  for (const auto& frame_handle : frame_handles) {
    if (frame_handle.is_primary || frame_handle.buffered_frame == nullptr) {
      continue;
    }

    const auto sensor_state = GetSensorState(frame_handle.sensor_id);
    if (sensor_state == nullptr) {
      continue;
    }

    const double observed_weight =
        EstimateOverlapQualityWeight(*frame_handle.buffered_frame,
                                     world2base_ref);
    std::lock_guard<std::mutex> lock(sensor_state->mutex);
    if (sensor_state->overlap_quality_samples == 0U) {
      sensor_state->overlap_quality_weight = observed_weight;
    } else {
      const double alpha = config_.overlap_quality_ema_alpha();
      sensor_state->overlap_quality_weight =
          sensor_state->overlap_quality_weight * (1.0 - alpha) +
          observed_weight * alpha;
    }
    ++sensor_state->overlap_quality_samples;

    if (frame_metrics != nullptr) {
      frame_metrics->min_overlap_quality_weight = std::min(
          frame_metrics->min_overlap_quality_weight,
          sensor_state->overlap_quality_weight);
    }
  }
}

double LidarUnifiedComponent::EstimateOverlapQualityWeight(
    const BufferedFrame& buffered_frame,
    const Eigen::Affine3d& world2base_ref) const {
  if (buffered_frame.point_cloud == nullptr || buffered_frame.motion_sample_times.empty() ||
      buffered_frame.motion_sample_times.size() != buffered_frame.motion_poses.size()) {
    return 1.0;
  }

  const size_t stride = std::max<size_t>(
      1, static_cast<size_t>(config_.overlap_quality_sample_stride()));
  size_t sampled_points = 0;
  size_t overlap_points = 0;
  for (size_t index = 0;
       index < static_cast<size_t>(buffered_frame.point_cloud->point_size());
       index += stride) {
    PointXYZIT transformed_point;
    if (!TransformPointToBase(buffered_frame.point_cloud->point(
                                  static_cast<int>(index)),
                              buffered_frame.point_cloud->measurement_time(),
                              buffered_frame.motion_sample_times,
                              buffered_frame.motion_poses, world2base_ref,
                              &transformed_point)) {
      continue;
    }
    ++sampled_points;
    if (IsPointInOverlapRegion(transformed_point)) {
      ++overlap_points;
    }
  }

  if (sampled_points == 0U) {
    return 1.0;
  }
  return static_cast<double>(overlap_points) /
         static_cast<double>(sampled_points);
}

bool LidarUnifiedComponent::IsPointInOverlapRegion(
  const ::apollo::drivers::PointXYZIT& point) const {
  return point.x() <= config_.overlap_region_forward_x() &&
         point.x() >= config_.overlap_region_backward_x() &&
         point.y() <= config_.overlap_region_left_y() &&
         point.y() >= config_.overlap_region_right_y() &&
         point.z() <= config_.overlap_region_max_z() &&
         point.z() >= config_.overlap_region_min_z();
}

bool LidarUnifiedComponent::BuildUnifiedPointCloud(
    const PointCloudConstPtr& main_frame,
    const std::vector<FrameHandle>& frame_handles, FrameMetrics* frame_metrics,
  std::shared_ptr<::apollo::drivers::PointCloud>* output) {
  if (main_frame == nullptr || frame_metrics == nullptr || output == nullptr ||
      frame_handles.empty() || deskew_policy_ == nullptr ||
      fusion_policy_ == nullptr || filter_policy_ == nullptr) {
    return false;
  }

  std::vector<SensorFrameContext> contexts;
  std::vector<std::vector<double>> motion_sample_times;
  std::vector<std::vector<Eigen::Affine3d>> motion_poses;
  contexts.reserve(frame_handles.size());
  motion_sample_times.reserve(frame_handles.size());
  motion_poses.reserve(frame_handles.size());

  size_t required_points = 0;
  if (!pose_bins_builder_.Build(frame_handles, deskew_policy_.get(), &contexts,
                                &motion_sample_times, &motion_poses,
                                &required_points)) {
    AERROR << "No valid sensor context for fusion";
    return false;
  }

  frame_metrics->total_input_points = required_points;
  frame_metrics->matched_sensor_count = contexts.size();

  if (required_points > full_pointcloud_buffer_.size()) {
    AWARN << "Input points exceed preallocated buffer, required="
          << required_points << ", capacity=" << full_pointcloud_buffer_.size();
  }

  Eigen::Affine3d world2base_ref = Eigen::Affine3d::Identity();
  if (!ResolveWorldToBase(main_frame->measurement_time(), &world2base_ref)) {
    AERROR << "Failed to resolve cached base pose at ref timestamp "
           << main_frame->measurement_time();
    return false;
  }
  UpdateOverlapQualityWeights(frame_handles, world2base_ref, frame_metrics);

  PointCloudBuffer fused_buffer;
  fused_buffer.data_ptr = full_pointcloud_buffer_.data();
  fused_buffer.capacity = full_pointcloud_buffer_.size();
  fused_buffer.valid_count = 0;
  fused_buffer.item_size = sizeof(apollo::drivers::PointXYZIT);
  fused_buffer.device_type = MemoryDeviceType::kHost;
  fused_buffer.device_id =
      config_.compute_mode() == LidarUnifiedComponentConfig::COMPUTE_MODE_GPU
          ? static_cast<int>(config_.gpu_device_id())
          : -1;

  if (!fusion_policy_->FuseToBaseLink(main_frame->measurement_time(),
                                      world2base_ref, contexts, motion_poses,
                                      motion_sample_times, &fused_buffer)) {
    AERROR << "Failed to fuse point clouds for ref timestamp "
           << main_frame->measurement_time();
    return false;
  }

  const size_t compact_points = fused_buffer.valid_count;
  size_t ego_filtered_points = 0;
  size_t voxel_filtered_points = 0;
  const size_t valid_size = filter_policy_->ApplyFilters(
      &fused_buffer, &ego_filtered_points, &voxel_filtered_points);
  frame_metrics->compact_points = compact_points;
  frame_metrics->ego_filtered_points = ego_filtered_points;
  frame_metrics->voxel_filtered_points = voxel_filtered_points;
  frame_metrics->output_points = valid_size;

  auto unified = std::make_shared<::apollo::drivers::PointCloud>();
  unified->mutable_header()->CopyFrom(main_frame->header());
  unified->mutable_header()->set_frame_id(config_.base_link_frame_id());
  unified->mutable_header()->set_module_name("lidar_unified_processor");
  unified->mutable_header()->set_timestamp_sec(cyber::Time::Now().ToSecond());
  unified->mutable_header()->set_sequence_num(sequence_num_.fetch_add(1) + 1);
  unified->set_frame_id(config_.base_link_frame_id());
  unified->set_measurement_time(main_frame->measurement_time());
  unified->set_is_dense(true);
  unified->mutable_point()->Reserve(static_cast<int>(valid_size));

  for (size_t i = 0; i < valid_size; ++i) {
    const auto& point = full_pointcloud_buffer_[i];
    auto* out_pt = unified->add_point();
    out_pt->set_x(point.x());
    out_pt->set_y(point.y());
    out_pt->set_z(point.z());
    out_pt->set_intensity(point.intensity());
    out_pt->set_timestamp(point.timestamp());
  }

  unified->set_height(1);
  unified->set_width(unified->point_size());

  *output = unified;
  return true;
}
}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
