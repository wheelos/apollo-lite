#include <algorithm>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <sstream>

#include "modules/drivers/lidar/processor/lidar_unified_component.h"
#include "modules/drivers/lidar/processor/policy/lidar_policy_common.h"

namespace apollo {
namespace drivers {
namespace lidar {

namespace {
constexpr char kPrimaryTopicFallback[] = "primary_lidar";
constexpr int kPosePrefetchLogFrequency = 10;

std::string FormatTimestampForLog(double timestamp_sec) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(9) << timestamp_sec << "s";
  if (timestamp_sec > 0.0) {
    stream << " [" << cyber::Time(timestamp_sec).ToString() << "]";
  }
  return stream.str();
}

std::string FormatDeltaForLog(double delta_sec) {
  const double abs_delta_sec = std::fabs(delta_sec);
  std::ostringstream stream;
  stream << std::showpos << std::fixed
         << std::setprecision(abs_delta_sec >= 1.0 ? 3 : 6) << delta_sec
         << "s" << std::noshowpos;
  if (abs_delta_sec >= 86400.0) {
    stream << " (~" << std::fixed << std::setprecision(3)
           << abs_delta_sec / 86400.0 << " days)";
  } else if (abs_delta_sec >= 3600.0) {
    stream << " (~" << std::fixed << std::setprecision(3)
           << abs_delta_sec / 3600.0 << " h)";
  } else if (abs_delta_sec >= 60.0) {
    stream << " (~" << std::fixed << std::setprecision(3)
           << abs_delta_sec / 60.0 << " min)";
  } else if (abs_delta_sec > 0.0 && abs_delta_sec < 1.0) {
    stream << " (~" << std::fixed << std::setprecision(3)
           << abs_delta_sec * 1000.0 << " ms)";
  }
  return stream.str();
}

const char* DescribeTimeRelation(double delta_sec) {
  return std::fabs(delta_sec) <= 1e-6 ? "aligned"
                                      : (delta_sec > 0.0 ? "ahead"
                                                         : "behind");
}

std::string BuildPointCloudTimeSummary(const PointCloud& point_cloud) {
  double min_point_time_sec = point_cloud.measurement_time();
  double max_point_time_sec = point_cloud.measurement_time();
  ResolvePointTimestampBounds(point_cloud, &min_point_time_sec,
                              &max_point_time_sec);

  const double measurement_time_sec = point_cloud.measurement_time();
  const double now_sec = cyber::Time::Now().ToSecond();
  const double measurement_vs_now_sec = measurement_time_sec - now_sec;

  std::ostringstream message;
  message << "measurement=" << FormatTimestampForLog(measurement_time_sec)
          << ", point_range=[" << FormatTimestampForLog(min_point_time_sec)
          << ", " << FormatTimestampForLog(max_point_time_sec) << "]"
          << ", measurement_vs_now="
          << FormatDeltaForLog(measurement_vs_now_sec) << " ("
          << DescribeTimeRelation(measurement_vs_now_sec) << ")";
  if (std::fabs(max_point_time_sec - min_point_time_sec) > 1e-6) {
    message << ", scan_span="
            << FormatDeltaForLog(max_point_time_sec - min_point_time_sec);
  }
  return message.str();
}

apollo::transform::TimedTransformResolverOptions BuildTransformResolverOptions(
    const LidarUnifiedComponentConfig& config) {
  apollo::transform::TimedTransformResolverOptions options;
  options.query_timeout_sec =
      static_cast<float>(config.sensor_pose_query_timeout_sec());
  options.cache_duration_sec = config.sensor_pose_cache_duration_sec();
  options.max_extrapolation_sec =
      config.sensor_pose_cache_max_extrapolation_sec();
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
  if (sensor_state == nullptr || sensor_state->pose_resolver == nullptr) {
    return false;
  }

  const size_t bins = std::max<size_t>(
      1, static_cast<size_t>(config_.motion_compensation_bins()));
  auto frame = std::make_shared<BufferedFrame>();
  frame->point_cloud = point_cloud;
  bool used_measurement_time_fallback = false;
  if (!BuildMotionSampleTimes(*point_cloud, bins,
                              config_.compute_mode() ==
                                  LidarUnifiedComponentConfig::COMPUTE_MODE_GPU,
                              static_cast<int>(config_.gpu_device_id()),
                              &frame->motion_sample_times,
                              &used_measurement_time_fallback)) {
    AWARN << "Failed to build motion sample times for sensor " << sensor_id;
    return false;
  }

  if (used_measurement_time_fallback) {
    AINFO_EVERY(kPosePrefetchLogFrequency)
        << "Fallback to measurement_time and disable intra-frame deskew. "
        << "sensor=" << sensor_id << ", target=" << config_.map_frame_id()
        << ", " << BuildPointCloudTimeSummary(*point_cloud);
  }

  if (!sensor_state->pose_resolver->PrefetchBatch(frame->motion_sample_times)) {
    std::lock_guard<std::mutex> lock(sensor_state->mutex);
    ++sensor_state->pose_prefetch_timeout_count;
    total_pose_prefetch_timeouts_.fetch_add(1);
  }

  apollo::transform::TransformResolveStatus cache_status =
      apollo::transform::TransformResolveStatus::kOk;
  if (!sensor_state->pose_resolver->QueryCachedBatchStrict(
          frame->motion_sample_times, &frame->motion_poses, &cache_status)) {
    AINFO_EVERY(kPosePrefetchLogFrequency)
      << "Pose prefetch unavailable. sensor=" << sensor_id
      << ", target=" << config_.map_frame_id()
      << ", status=" << static_cast<int>(cache_status) << ", "
      << BuildPointCloudTimeSummary(*point_cloud);
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
  sensor_state->pose_resolver =
      std::make_unique<apollo::transform::TimedTransformResolver>(
          tf_buffer_, config_.map_frame_id(), sensor_id,
          BuildTransformResolverOptions(config_));
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
    AINFO_EVERY(kPosePrefetchLogFrequency)
        << "Skip auxiliary lidar frame due to pose prefetch failure. sensor="
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
