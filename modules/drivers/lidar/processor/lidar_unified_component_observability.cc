#include <algorithm>

#include "modules/drivers/lidar/processor/lidar_unified_component.h"

namespace apollo {
namespace drivers {
namespace lidar {

namespace {

const char* DegradeModeName(DegradeMode mode) {
  switch (mode) {
    case DegradeMode::kNormal:
      return "normal";
    case DegradeMode::kSingleLidar:
      return "single_lidar";
    default:
      return "unknown";
  }
}

}  // namespace

void LidarUnifiedComponent::LogFrameMetrics(const FrameMetrics& frame_metrics) {
  const uint64_t frame_index = frames_total_.fetch_add(1) + 1;
  total_input_points_.fetch_add(frame_metrics.total_input_points);
  total_output_points_.fetch_add(frame_metrics.output_points);
  total_missing_auxiliary_frames_.fetch_add(
      frame_metrics.missing_auxiliary_count);
  total_time_delta_exceeded_.fetch_add(frame_metrics.time_delta_exceeded_count);

  const uint32_t log_interval =
      std::max<uint32_t>(1, config_.metrics_log_interval());
  if (frame_index % log_interval != 0) {
    return;
  }

  const uint64_t total_frames = std::max<uint64_t>(1, frames_total_.load());
  const uint64_t total_input =
      std::max<uint64_t>(1, total_input_points_.load());
  const uint64_t total_output = total_output_points_.load();
  const uint64_t total_aux_missing = total_missing_auxiliary_frames_.load();
  const uint64_t total_time_delta = total_time_delta_exceeded_.load();
  const uint64_t total_deadline_exceeded =
      total_fusion_deadline_exceeded_.load();
  const uint64_t total_pending_dropped = total_pending_fusion_dropped_.load();
  const uint64_t total_pose_prefetch_timeouts =
      total_pose_prefetch_timeouts_.load();
  const uint64_t total_tf_failures = total_tf_query_failures_.load();
  const uint64_t ts_anomalies = dtc_reporter_.ts_anomaly_count();
  const uint64_t degrade_transitions = dtc_reporter_.degrade_transition_count();

  AINFO << "LidarUnifiedProcessor metrics: frame=" << frame_index
        << ", matched_sensors=" << frame_metrics.matched_sensor_count << "/"
        << frame_metrics.expected_sensor_count
        << ", degrade_mode=" << DegradeModeName(degrade_policy_.CurrentMode())
        << ", missing_aux=" << frame_metrics.missing_auxiliary_count
        << ", input_points=" << frame_metrics.total_input_points
        << ", compact_points=" << frame_metrics.compact_points
        << ", voxel_filtered_points=" << frame_metrics.voxel_filtered_points
        << ", ego_filtered_points=" << frame_metrics.ego_filtered_points
        << ", output_points=" << frame_metrics.output_points
        << ", max_abs_clock_offset_ms=" << frame_metrics.max_abs_clock_offset_ms
        << ", min_overlap_quality_weight="
        << frame_metrics.min_overlap_quality_weight
        << ", fusion_wait_ms=" << frame_metrics.fusion_wait_ms
        << ", fusion_deadline_exceeded="
        << frame_metrics.fusion_deadline_exceeded
        << ", ts_anomalies=" << ts_anomalies
        << ", degrade_transitions=" << degrade_transitions
        << ", avg_output_ratio="
        << static_cast<double>(total_output) / static_cast<double>(total_input)
        << ", avg_aux_drop_per_frame="
        << static_cast<double>(total_aux_missing) /
               static_cast<double>(total_frames)
        << ", avg_time_window_violations="
        << static_cast<double>(total_time_delta) /
               static_cast<double>(total_frames)
        << ", avg_fusion_deadline_exceeded="
        << static_cast<double>(total_deadline_exceeded) /
               static_cast<double>(total_frames)
        << ", pending_fusion_dropped=" << total_pending_dropped
        << ", avg_pose_prefetch_timeouts="
        << static_cast<double>(total_pose_prefetch_timeouts) /
               static_cast<double>(total_frames)
        << ", avg_tf_failures="
        << static_cast<double>(total_tf_failures) /
               static_cast<double>(total_frames);
}

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
