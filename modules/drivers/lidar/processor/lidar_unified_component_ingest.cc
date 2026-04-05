#include <algorithm>
#include <cctype>

#include "modules/drivers/lidar/processor/lidar_unified_component.h"
#include "modules/drivers/lidar/processor/policy/lidar_policy_common.h"

namespace apollo {
namespace drivers {
namespace lidar {

namespace {
constexpr char kPrimaryTopicFallback[] = "primary_lidar";

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

std::string LidarUnifiedComponent::MakeFallbackSensorId(
    const std::string& topic_name) const {
  if (topic_name.empty()) {
    return kPrimaryTopicFallback;
  }

  std::string sensor_id;
  sensor_id.reserve(topic_name.size());
  for (const char ch : topic_name) {
    if (std::isalnum(static_cast<unsigned char>(ch))) {
      sensor_id.push_back(ch);
    } else if (!sensor_id.empty() && sensor_id.back() != '_') {
      sensor_id.push_back('_');
    }
  }

  while (!sensor_id.empty() && sensor_id.back() == '_') {
    sensor_id.pop_back();
  }

  return sensor_id.empty() ? kPrimaryTopicFallback : sensor_id;
}

std::string LidarUnifiedComponent::ResolveSensorId(
    const PointCloudConstPtr& point_cloud,
    const std::string& topic_name) const {
  if (point_cloud != nullptr) {
    if (!point_cloud->frame_id().empty()) {
      return point_cloud->frame_id();
    }
    if (point_cloud->has_header() &&
        !point_cloud->header().frame_id().empty()) {
      return point_cloud->header().frame_id();
    }
  }

  return MakeFallbackSensorId(topic_name);
}

bool LidarUnifiedComponent::PrepareBufferedFrame(
    const std::string& sensor_id, const PointCloudConstPtr& point_cloud,
    std::shared_ptr<BufferedFrame>* buffered_frame) {
  if (sensor_id.empty() || point_cloud == nullptr ||
      buffered_frame == nullptr) {
    return false;
  }

  const auto sensor_state = EnsureSensorState(sensor_id);
  if (sensor_state == nullptr || sensor_state->pose_cache == nullptr) {
    return false;
  }

  const size_t bins = std::max<size_t>(
      1, static_cast<size_t>(config_.motion_compensation_bins()));
  auto frame = std::make_shared<BufferedFrame>();
  frame->point_cloud = point_cloud;
  if (!BuildMotionSampleTimes(*point_cloud, bins,
                              config_.compute_mode() ==
                                  LidarUnifiedComponentConfig::COMPUTE_MODE_GPU,
                              static_cast<int>(config_.gpu_device_id()),
                              &frame->motion_sample_times)) {
    AWARN << "Failed to build motion sample times for sensor " << sensor_id;
    return false;
  }

  if (!sensor_state->pose_cache->PrefetchBatch(frame->motion_sample_times)) {
    std::lock_guard<std::mutex> lock(sensor_state->mutex);
    ++sensor_state->pose_prefetch_timeout_count;
    total_pose_prefetch_timeouts_.fetch_add(1);
  }

  apollo::transform::PoseCacheStatus cache_status =
      apollo::transform::PoseCacheStatus::kOk;
    if (!sensor_state->pose_cache->QueryCachedBatchStrict(
          frame->motion_sample_times, &frame->motion_poses, &cache_status)) {
    AWARN << "Pose cache miss for sensor " << sensor_id
          << ", status=" << static_cast<int>(cache_status);
    total_tf_query_failures_.fetch_add(1);
    return false;
  }

  frame->pose_prefetch_ok = true;
  *buffered_frame = frame;
  return true;
}

std::shared_ptr<LidarUnifiedComponent::SensorState>
LidarUnifiedComponent::EnsureSensorState(const std::string& sensor_id) {
  std::lock_guard<std::mutex> lock(sensor_registry_mutex_);
  auto it = sensor_states_.find(sensor_id);
  if (it != sensor_states_.end()) {
    return it->second;
  }

  auto sensor_state = std::make_shared<SensorState>(sensor_buffer_capacity_);
  sensor_state->pose_cache =
      std::make_unique<apollo::transform::TransformFrameCache>(
          tf_buffer_, config_.world_frame_id(), sensor_id,
          BuildPoseCacheOptions(config_));
  sensor_states_.emplace(sensor_id, sensor_state);
  return sensor_state;
}

std::shared_ptr<LidarUnifiedComponent::SensorState>
LidarUnifiedComponent::GetSensorState(const std::string& sensor_id) const {
  std::lock_guard<std::mutex> lock(sensor_registry_mutex_);
  const auto it = sensor_states_.find(sensor_id);
  if (it == sensor_states_.end()) {
    return nullptr;
  }
  return it->second;
}

void LidarUnifiedComponent::OnAuxiliaryLidarMessage(
    const std::string& topic_name, const PointCloudConstPtr& point_cloud) {
  if (point_cloud == nullptr) {
    return;
  }

  const std::string sensor_id = ResolveSensorId(point_cloud, topic_name);
  if (sensor_id.empty()) {
    AWARN << "Skip auxiliary lidar message due to empty sensor id. topic="
          << topic_name;
    return;
  }

  std::shared_ptr<BufferedFrame> buffered_frame;
  if (!PrepareBufferedFrame(sensor_id, point_cloud, &buffered_frame)) {
    AWARN
        << "Skip auxiliary lidar message due to pose prefetch failure. sensor="
        << sensor_id << ", topic=" << topic_name;
    return;
  }
  {
    std::lock_guard<std::mutex> lock(sensor_registry_mutex_);
    auto& mapped_sensor_id = auxiliary_sensor_ids_by_topic_[topic_name];
    if (!mapped_sensor_id.empty() && mapped_sensor_id != sensor_id) {
      AWARN << "Auxiliary topic sensor id changed from " << mapped_sensor_id
            << " to " << sensor_id << ", topic=" << topic_name;
    }
    mapped_sensor_id = sensor_id;
  }

  PushToBuffer(sensor_id, buffered_frame);
}

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
