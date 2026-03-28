#include "modules/drivers/lidar/processor/lidar_unified_component.h"

#include <algorithm>

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
    case LidarUnifiedComponentConfig::USE_GPU:
      return "gpu";
    case LidarUnifiedComponentConfig::USE_CPU:
    default:
      return "cpu";
  }
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

  writer_ = node_->CreateWriter<PointCloud>(config_.output_channel());
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

  sensor_buffer_capacity_ =
      std::max<size_t>(2, static_cast<size_t>(config_.sensor_buffer_size()));

  auxiliary_inputs_.clear();
  auxiliary_readers_.clear();
  auxiliary_sensor_ids_by_topic_.clear();
  raw_cloud_buffers_.clear();
  primary_sensor_id_.clear();

  for (const auto& input_cfg : config_.auxiliary_lidar_inputs()) {
    auxiliary_inputs_.push_back(SensorInput{input_cfg.topic_name()});

    auto reader = node_->CreateReader<PointCloud>(
        input_cfg.topic_name(), [this, topic_name = input_cfg.topic_name()](
                                    const std::shared_ptr<PointCloud>& msg) {
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
    const std::shared_ptr<PointCloud>& point_cloud) {
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
    AWARN << "Primary lidar sensor id changed from " << primary_sensor_id_
          << " to " << sensor_id << ", keep original primary sensor id";
  }

  EnsureSensorBuffer(primary_sensor_id_);
  PushToBuffer(primary_sensor_id_, point_cloud);

  FrameMetrics frame_metrics;
  std::vector<SensorFrame> frames;
  if (!CollectNearestFrames(point_cloud->measurement_time(), primary_sensor_id_,
                            &frames, &frame_metrics)) {
    AERROR << "Failed to collect nearest frames for ref timestamp "
           << point_cloud->measurement_time();
    return false;
  }

  std::shared_ptr<PointCloud> unified_output;
  if (!BuildUnifiedPointCloud(point_cloud, frames, &frame_metrics,
                              &unified_output)) {
    AERROR << "Failed to build unified point cloud";
    return false;
  }

  LogFrameMetrics(frame_metrics);
  writer_->Write(unified_output);
  return true;
}

void LidarUnifiedComponent::PushToBuffer(
    const std::string& sensor_id, const PointCloudConstPtr& point_cloud) {
  const auto sensor_buffer = GetSensorBuffer(sensor_id);
  if (sensor_buffer == nullptr) {
    return;
  }
  std::lock_guard<std::mutex> lock(sensor_buffer->mutex);
  sensor_buffer->queue.push_back(point_cloud);
}

bool LidarUnifiedComponent::CollectNearestFrames(
    double ref_timestamp, const std::string& primary_sensor_id,
    std::vector<SensorFrame>* frames, FrameMetrics* frame_metrics) {
  if (frames == nullptr || frame_metrics == nullptr) {
    return false;
  }
  frames->clear();
  frame_metrics->expected_sensor_count = 1 + auxiliary_inputs_.size();
  frame_metrics->matched_sensor_count = 0;
  frame_metrics->missing_auxiliary_count = 0;
  frame_metrics->time_delta_exceeded_count = 0;

  const uint32_t max_delta_ms = config_.max_ref_time_delta_ms();
  const auto primary_buffer = GetSensorBuffer(primary_sensor_id);
  PointCloudConstPtr primary_frame;
  FrameLookupFailureReason failure_reason = FrameLookupFailureReason::kNone;
  if (!FindNearestFrame(primary_buffer, ref_timestamp, max_delta_ms,
                        &primary_frame, &failure_reason)) {
    AERROR << "Primary sensor frame unavailable around reference timestamp";
    return false;
  }
  frames->push_back(SensorFrame{primary_sensor_id, primary_frame, true});

  for (const auto& auxiliary_input : auxiliary_inputs_) {
    std::string sensor_id;
    {
      std::lock_guard<std::mutex> lock(sensor_registry_mutex_);
      const auto it =
          auxiliary_sensor_ids_by_topic_.find(auxiliary_input.topic_name);
      if (it != auxiliary_sensor_ids_by_topic_.end()) {
        sensor_id = it->second;
      }
    }

    if (sensor_id.empty()) {
      ++frame_metrics->missing_auxiliary_count;
      if (config_.strict_auxiliary_sync()) {
        AERROR << "Auxiliary topic has not resolved sensor id yet: "
               << auxiliary_input.topic_name;
        return false;
      }
      continue;
    }

    PointCloudConstPtr nearest;
    failure_reason = FrameLookupFailureReason::kNone;
    if (!FindNearestFrame(GetSensorBuffer(sensor_id), ref_timestamp,
                          max_delta_ms, &nearest, &failure_reason)) {
      ++frame_metrics->missing_auxiliary_count;
      if (failure_reason == FrameLookupFailureReason::kTimeDeltaExceeded) {
        ++frame_metrics->time_delta_exceeded_count;
      }
      if (config_.strict_auxiliary_sync()) {
        AERROR << "Auxiliary sensor sync failed: " << sensor_id;
        return false;
      }
      AWARN << "Skip auxiliary sensor " << sensor_id
            << " due to sync miss, topic=" << auxiliary_input.topic_name;
      continue;
    }
    frames->push_back(SensorFrame{sensor_id, nearest, false});
  }

  frame_metrics->matched_sensor_count = frames->size();
  return !frames->empty();
}

bool LidarUnifiedComponent::FindNearestFrame(
    const std::shared_ptr<SensorBuffer>& sensor_buffer, double ref_timestamp,
    uint32_t max_ref_time_delta_ms, PointCloudConstPtr* nearest_frame,
    FrameLookupFailureReason* failure_reason) const {
  if (nearest_frame == nullptr || failure_reason == nullptr) {
    return false;
  }

  *failure_reason = FrameLookupFailureReason::kNone;
  if (sensor_buffer == nullptr) {
    *failure_reason = FrameLookupFailureReason::kBufferEmpty;
    return false;
  }

  std::lock_guard<std::mutex> lock(sensor_buffer->mutex);
  if (sensor_buffer->queue.empty()) {
    *failure_reason = FrameLookupFailureReason::kBufferEmpty;
    return false;
  }

  const auto lower = std::lower_bound(
      sensor_buffer->queue.begin(), sensor_buffer->queue.end(), ref_timestamp,
      [](const PointCloudConstPtr& lhs, double ts) {
        return lhs->measurement_time() < ts;
      });

  auto best_itr = lower;
  if (lower == sensor_buffer->queue.end()) {
    best_itr = std::prev(sensor_buffer->queue.end());
  } else if (lower != sensor_buffer->queue.begin()) {
    const auto prev_itr = std::prev(lower);
    const double diff_prev =
        std::fabs((*prev_itr)->measurement_time() - ref_timestamp);
    const double diff_next =
        std::fabs((*lower)->measurement_time() - ref_timestamp);
    best_itr = (diff_prev <= diff_next) ? prev_itr : lower;
  }

  if (best_itr == sensor_buffer->queue.end()) {
    return false;
  }

  const double delta_ms =
      std::fabs((*best_itr)->measurement_time() - ref_timestamp) * 1000.0;
  if (delta_ms > static_cast<double>(max_ref_time_delta_ms)) {
    *failure_reason = FrameLookupFailureReason::kTimeDeltaExceeded;
    return false;
  }

  *nearest_frame = *best_itr;
  return true;
}

bool LidarUnifiedComponent::BuildUnifiedPointCloud(
    const PointCloudConstPtr& main_frame,
    const std::vector<SensorFrame>& frames, FrameMetrics* frame_metrics,
    std::shared_ptr<PointCloud>* output) {
  if (main_frame == nullptr || frame_metrics == nullptr || output == nullptr ||
      frames.empty() || deskew_policy_ == nullptr ||
      fusion_policy_ == nullptr || filter_policy_ == nullptr) {
    return false;
  }

  std::vector<SensorFrameContext> contexts;
  std::vector<std::vector<double>> motion_sample_times;
  std::vector<std::vector<Eigen::Affine3d>> motion_poses;
  contexts.reserve(frames.size());
  motion_sample_times.reserve(frames.size());
  motion_poses.reserve(frames.size());

  size_t required_points = 0;
  for (const auto& frame_item : frames) {
    if (frame_item.point_cloud == nullptr) {
      if (frame_item.is_primary) {
        AERROR << "Main sensor frame is null: " << frame_item.sensor_id;
        return false;
      }
      AWARN << "Skip auxiliary sensor due to null frame: "
            << frame_item.sensor_id;
      continue;
    }

    SensorFrameContext context;
    context.sensor_id = frame_item.sensor_id;
    context.point_cloud = frame_item.point_cloud;
    context.is_primary = frame_item.is_primary;

    std::vector<double> sample_times;
    std::vector<Eigen::Affine3d> poses;
    if (!deskew_policy_->ComputeMotionCompensationPoses(context, &sample_times,
                                                        &poses) ||
        sample_times.empty() || poses.empty() ||
        sample_times.size() != poses.size()) {
      if (frame_item.is_primary) {
        AERROR << "Failed to compute motion compensation poses for main sensor "
               << frame_item.sensor_id;
        return false;
      }
      AWARN << "Skip auxiliary sensor due to invalid motion compensation data: "
            << frame_item.sensor_id;
      continue;
    }

    contexts.push_back(std::move(context));
    motion_sample_times.push_back(std::move(sample_times));
    motion_poses.push_back(std::move(poses));
    required_points +=
        static_cast<size_t>(frame_item.point_cloud->point_size());
  }

  if (contexts.empty()) {
    AERROR << "No valid sensor context for fusion";
    return false;
  }

  frame_metrics->total_input_points = required_points;
  frame_metrics->matched_sensor_count = contexts.size();

  if (required_points > full_pointcloud_buffer_.size()) {
    AWARN << "Input points exceed preallocated buffer, required="
          << required_points << ", capacity=" << full_pointcloud_buffer_.size();
  }

  PointCloudBuffer fused_buffer;
  fused_buffer.data_ptr = full_pointcloud_buffer_.data();
  fused_buffer.capacity = full_pointcloud_buffer_.size();
  fused_buffer.valid_count = 0;
  fused_buffer.item_size = sizeof(apollo::drivers::PointXYZIT);
  fused_buffer.device_type = MemoryDeviceType::kHost;
  fused_buffer.device_id =
      config_.compute_mode() == LidarUnifiedComponentConfig::USE_GPU
          ? static_cast<int>(config_.gpu_device_id())
          : -1;

  if (!fusion_policy_->FuseToBaseLink(main_frame->measurement_time(), contexts,
                                      motion_poses, motion_sample_times,
                                      &fused_buffer)) {
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

  auto unified = std::make_shared<PointCloud>();
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
