#include "modules/drivers/lidar/processor/lidar_unified_component.h"

#include <set>

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

  if (config_.map_frame_id().empty()) {
    AERROR << "map_frame_id is required";
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

  if (config_.metrics_log_interval() == 0) {
    AERROR << "metrics_log_interval must be > 0";
    return false;
  }

  if (config_.voxel_size() <= 0.0) {
    AERROR << "voxel_size must be > 0";
    return false;
  }

  if (config_.sensor_pose_cache_duration_sec() <= 0.0) {
    AERROR << "sensor_pose_cache_duration_sec must be > 0";
    return false;
  }

  if (config_.sensor_pose_cache_max_extrapolation_sec() < 0.0) {
    AERROR << "sensor_pose_cache_max_extrapolation_sec must be >= 0";
    return false;
  }

  if (config_.sensor_pose_query_timeout_sec() < 0.0) {
    AERROR << "sensor_pose_query_timeout_sec must be >= 0";
    return false;
  }

  if (config_.fixed_delay_ema_alpha() <= 0.0 ||
      config_.fixed_delay_ema_alpha() > 1.0) {
    AERROR << "fixed_delay_ema_alpha must be in (0, 1]";
    return false;
  }

  if (config_.fixed_delay_update_limit_ms() < 0.0) {
    AERROR << "fixed_delay_update_limit_ms must be >= 0";
    return false;
  }

  if (config_.clock_offset_ema_alpha() <= 0.0 ||
      config_.clock_offset_ema_alpha() > 1.0) {
    AERROR << "clock_offset_ema_alpha must be in (0, 1]";
    return false;
  }

  if (config_.overlap_quality_ema_alpha() <= 0.0 ||
      config_.overlap_quality_ema_alpha() > 1.0) {
    AERROR << "overlap_quality_ema_alpha must be in (0, 1]";
    return false;
  }

  if (config_.auxiliary_min_overlap_quality_weight() < 0.0 ||
      config_.auxiliary_min_overlap_quality_weight() > 1.0) {
    AERROR << "auxiliary_min_overlap_quality_weight must be in [0, 1]";
    return false;
  }

  if (config_.overlap_quality_sample_stride() == 0) {
    AERROR << "overlap_quality_sample_stride must be > 0";
    return false;
  }

  if (config_.pending_fusion_queue_size() == 0) {
    AERROR << "pending_fusion_queue_size must be > 0";
    return false;
  }

  if (config_.fusion_flush_interval_ms() == 0) {
    AERROR << "fusion_flush_interval_ms must be > 0";
    return false;
  }

  if (config_.overlap_region_forward_x() <
      config_.overlap_region_backward_x()) {
    AERROR << "overlap_region_forward_x must be >= overlap_region_backward_x";
    return false;
  }

  if (config_.overlap_region_left_y() < config_.overlap_region_right_y()) {
    AERROR << "overlap_region_left_y must be >= overlap_region_right_y";
    return false;
  }

  if (config_.overlap_region_max_z() < config_.overlap_region_min_z()) {
    AERROR << "overlap_region_max_z must be >= overlap_region_min_z";
    return false;
  }

  if (config_.ts_sanity_enabled()) {
    if (config_.ts_sanity_min_interval_ms() == 0) {
      AERROR << "ts_sanity_min_interval_ms must be > 0";
      return false;
    }
    if (config_.ts_sanity_max_interval_ms() <
        config_.ts_sanity_min_interval_ms()) {
      AERROR
          << "ts_sanity_max_interval_ms must be >= ts_sanity_min_interval_ms";
      return false;
    }
    if (config_.ts_sanity_max_jump_ms() < config_.ts_sanity_max_interval_ms()) {
      AERROR << "ts_sanity_max_jump_ms must be >= ts_sanity_max_interval_ms";
      return false;
    }
    if (config_.ts_sanity_max_consecutive_errors() == 0) {
      AERROR << "ts_sanity_max_consecutive_errors must be > 0";
      return false;
    }
  }

  std::set<std::string> auxiliary_topics;
  for (const auto& input_cfg : config_.auxiliary_lidar_inputs()) {
    if (input_cfg.topic_name().empty()) {
      AERROR << "auxiliary_lidar_inputs.topic_name is required";
      return false;
    }
    if (!auxiliary_topics.insert(input_cfg.topic_name()).second) {
      AERROR << "Duplicate auxiliary lidar topic: " << input_cfg.topic_name();
      return false;
    }
  }

  return true;
}

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
