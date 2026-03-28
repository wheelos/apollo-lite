#include "modules/drivers/lidar/processor/lidar_unified_component.h"

namespace apollo {
namespace drivers {
namespace lidar {

bool LidarUnifiedComponent::ValidateConfig() const {
  if (config_.output_channel().empty()) {
    AERROR << "output_channel is required";
    return false;
  }

  if (config_.base_link_frame_id().empty()) {
    AERROR << "base_link_frame_id is required";
    return false;
  }

  if (config_.world_frame_id().empty()) {
    AERROR << "world_frame_id is required";
    return false;
  }

  if (config_.max_ref_time_delta_ms() == 0) {
    AERROR << "max_ref_time_delta_ms must be > 0";
    return false;
  }

  if (config_.motion_compensation_bins() == 0) {
    AERROR << "motion_compensation_bins must be > 0";
    return false;
  }

  if (config_.sensor_buffer_size() == 0) {
    AERROR << "sensor_buffer_size must be > 0";
    return false;
  }

  if (config_.max_full_pointcloud_points() == 0) {
    AERROR << "max_full_pointcloud_points must be > 0";
    return false;
  }

  for (const auto& input_cfg : config_.auxiliary_lidar_inputs()) {
    if (input_cfg.topic_name().empty()) {
      AERROR << "auxiliary_lidar_inputs.topic_name is required";
      return false;
    }
  }

  return true;
}

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
