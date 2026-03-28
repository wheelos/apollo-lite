#include "modules/drivers/lidar/processor/lidar_unified_component.h"

#include <cctype>

namespace apollo {
namespace drivers {
namespace lidar {

namespace {
constexpr char kPrimaryTopicFallback[] = "primary_lidar";
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
    const PointCloudConstPtr& point_cloud, const std::string& topic_name) const {
  if (point_cloud != nullptr) {
    if (!point_cloud->frame_id().empty()) {
      return point_cloud->frame_id();
    }
    if (point_cloud->has_header() && !point_cloud->header().frame_id().empty()) {
      return point_cloud->header().frame_id();
    }
  }

  return MakeFallbackSensorId(topic_name);
}

std::shared_ptr<LidarUnifiedComponent::SensorBuffer>
LidarUnifiedComponent::EnsureSensorBuffer(const std::string& sensor_id) {
  std::lock_guard<std::mutex> lock(sensor_registry_mutex_);
  auto it = raw_cloud_buffers_.find(sensor_id);
  if (it != raw_cloud_buffers_.end()) {
    return it->second;
  }

  auto buffer = std::make_shared<SensorBuffer>(sensor_buffer_capacity_);
  raw_cloud_buffers_.emplace(sensor_id, buffer);
  return buffer;
}

std::shared_ptr<LidarUnifiedComponent::SensorBuffer>
LidarUnifiedComponent::GetSensorBuffer(const std::string& sensor_id) const {
  std::lock_guard<std::mutex> lock(sensor_registry_mutex_);
  const auto it = raw_cloud_buffers_.find(sensor_id);
  if (it == raw_cloud_buffers_.end()) {
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

  EnsureSensorBuffer(sensor_id);
  {
    std::lock_guard<std::mutex> lock(sensor_registry_mutex_);
    auto& mapped_sensor_id = auxiliary_sensor_ids_by_topic_[topic_name];
    if (!mapped_sensor_id.empty() && mapped_sensor_id != sensor_id) {
      AWARN << "Auxiliary topic sensor id changed from " << mapped_sensor_id
            << " to " << sensor_id << ", topic=" << topic_name;
    }
    mapped_sensor_id = sensor_id;
  }

  PushToBuffer(sensor_id, point_cloud);
}

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
