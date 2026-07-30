#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <boost/circular_buffer.hpp>

#include "Eigen/Eigen"

#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"
#include "modules/drivers/lidar/proto/lidar_unified_component_config.pb.h"

#include "cyber/cyber.h"
#include "modules/drivers/lidar/processor/control/frame_handle.h"
#include "modules/drivers/lidar/processor/control/pose_bins_builder.h"
#include "modules/drivers/lidar/processor/control/sync_gate.h"
#include "modules/drivers/lidar/processor/policy/lidar_policy_interface.h"
#include "modules/drivers/lidar/processor/safety/degrade_policy.h"
#include "modules/drivers/lidar/processor/safety/dtc_reporter.h"
#include "modules/drivers/lidar/processor/safety/ts_sanity.h"
#include "modules/transform/buffer.h"
#include "modules/transform/timed_transform_resolver.h"

namespace apollo {
namespace drivers {
namespace lidar {

class LidarUnifiedComponent
    : public apollo::cyber::Component<::apollo::drivers::PointCloud> {
 public:
  using PointCloudConstPtr =
      std::shared_ptr<const ::apollo::drivers::PointCloud>;

  bool Init() override;
  bool Proc(const std::shared_ptr<::apollo::drivers::PointCloud>& point_cloud)
      override;

  bool OnReceiveMainLidar(const PointCloudConstPtr& point_cloud);

 private:
  friend class LidarUnifiedComponentTest_RejectsPrimarySensorIdDrift_Test;
  friend class
      LidarUnifiedComponentTest_FindsNearestFrameFromOutOfOrderBuffer_Test;
  friend class
      LidarUnifiedComponentTest_ReportsTimeDeltaExceededForNearestFrame_Test;
  friend class
      LidarUnifiedComponentTest_AppliesFixedDelayDuringFrameLookup_Test;
  friend class LidarUnifiedComponentTest_UpdatesSensorTimingModel_Test;
  friend class
      LidarUnifiedComponentTest_UpdatesLargeFixedDelayWhenInnovationIsWithinLimit_Test;
  friend class
      LidarUnifiedComponentTest_CollectNearestFramesSkipsLowQualityAuxiliary_Test;
  friend class
      LidarUnifiedComponentTest_CollectNearestFramesMatchesThreeSensors_Test;
  friend class
      LidarUnifiedComponentTest_CollectNearestFramesAllowsMissingAuxiliary_Test;
  friend class
      LidarUnifiedComponentTest_CollectNearestFramesFailsStrictMissingAuxiliary_Test;
  friend class LidarUnifiedComponentTest_RejectsDuplicateAuxiliaryTopics_Test;
  friend class LidarUnifiedComponentTest_EstimatesOverlapQualityWeight_Test;

  enum class FrameLookupFailureReason {
    kNone = 0,
    kBufferEmpty = 1,
    kTimeDeltaExceeded = 2,
  };

  struct SensorState {
    explicit SensorState(size_t capacity = 1) : frames(capacity) {}

    boost::circular_buffer<std::shared_ptr<BufferedFrame>> frames;
    std::unique_ptr<apollo::transform::TimedTransformResolver> pose_resolver;
    double fixed_delay_sec = 0.0;
    bool fixed_delay_initialized = false;
    uint64_t timing_observation_count = 0;
    double last_clock_offset_ms = 0.0;
    double smoothed_clock_offset_ms = 0.0;
    double overlap_quality_weight = 1.0;
    uint64_t overlap_quality_samples = 0;
    uint64_t pose_prefetch_timeout_count = 0;
    mutable std::mutex mutex;
  };

  struct SensorInput {
    std::string topic_name;
  };

  struct FrameMetrics {
    size_t expected_sensor_count = 0;
    size_t matched_sensor_count = 0;
    size_t missing_auxiliary_count = 0;
    size_t time_delta_exceeded_count = 0;
    size_t total_input_points = 0;
    size_t compact_points = 0;
    size_t voxel_filtered_points = 0;
    size_t ego_filtered_points = 0;
    size_t output_points = 0;
    double max_abs_clock_offset_ms = 0.0;
    double min_overlap_quality_weight = 1.0;
    double fusion_wait_ms = 0.0;
    bool fusion_deadline_exceeded = false;
  };

  struct PendingFusionFrame {
    PointCloudConstPtr main_frame;
    std::string primary_sensor_id;
    double reference_timestamp_sec = 0.0;
    double enqueue_time_sec = 0.0;
    double deadline_sec = 0.0;
  };

  bool PrepareBufferedFrame(const std::string& sensor_id,
                            const PointCloudConstPtr& point_cloud,
                            std::shared_ptr<BufferedFrame>* buffered_frame);
  void PushToBuffer(const std::string& sensor_id,
                    const std::shared_ptr<BufferedFrame>& buffered_frame);
  void OnAuxiliaryLidarMessage(const std::string& topic_name,
                               const PointCloudConstPtr& point_cloud);
  bool ValidateConfig() const;
  std::string ResolveSensorId(const PointCloudConstPtr& point_cloud,
                              const std::string& topic_name) const;
  std::string MakeFallbackSensorId(const std::string& topic_name) const;
  std::shared_ptr<SensorState> EnsureSensorState(const std::string& sensor_id);
  std::shared_ptr<SensorState> GetSensorState(
      const std::string& sensor_id) const;

  bool CollectNearestFrames(double ref_timestamp,
                            const std::string& primary_sensor_id,
                            std::vector<FrameHandle>* frame_handles,
                            FrameMetrics* frame_metrics);
  void EnqueuePendingFusionFrame(const PointCloudConstPtr& main_frame,
                                 const std::string& primary_sensor_id);
  void TryFlushPendingFusionFrames(bool flush_expired_only);
  void OnFusionFlushTimer();
  bool ProcessFusionFrame(const PendingFusionFrame& pending_frame,
                          std::vector<FrameHandle> frame_handles,
                          FrameMetrics frame_metrics);
  bool FindNearestFrame(const std::shared_ptr<SensorState>& sensor_state,
                        const std::string& sensor_id, double ref_timestamp,
                        uint32_t max_ref_time_delta_ms,
                        FrameHandle* frame_handle,
                        FrameLookupFailureReason* failure_reason) const;
  void UpdateSensorTimingModel(const FrameHandle& frame_handle,
                               double ref_timestamp_sec,
                               FrameMetrics* frame_metrics);
  bool ResolveMapToBase(double ref_timestamp_sec,
                        Eigen::Affine3d* map2base_ref);
  void UpdateOverlapQualityWeights(
      const std::vector<FrameHandle>& frame_handles,
      const Eigen::Affine3d& map2base_ref, FrameMetrics* frame_metrics);
  double EstimateOverlapQualityWeight(
      const BufferedFrame& buffered_frame,
      const Eigen::Affine3d& map2base_ref) const;
  bool IsPointInOverlapRegion(const ::apollo::drivers::PointXYZIT& point) const;

  bool BuildUnifiedPointCloud(
      const PointCloudConstPtr& main_frame,
      const std::vector<FrameHandle>& frame_handles,
      FrameMetrics* frame_metrics,
      std::shared_ptr<::apollo::drivers::PointCloud>* output);
  void LogFrameMetrics(const FrameMetrics& frame_metrics);

 private:
  LidarUnifiedComponentConfig config_;
  std::string primary_sensor_id_;
  size_t sensor_buffer_capacity_ = 1;

  apollo::transform::Buffer* tf_buffer_ = nullptr;
  std::unique_ptr<apollo::transform::TimedTransformResolver>
      base_link_pose_resolver_;
  mutable std::mutex sensor_registry_mutex_;

  std::map<std::string, std::shared_ptr<SensorState>> sensor_states_;
  std::map<std::string, std::string> auxiliary_sensor_ids_by_topic_;
  std::map<
      std::string,
      std::shared_ptr<apollo::cyber::Reader<::apollo::drivers::PointCloud>>>
      auxiliary_readers_;
  std::vector<SensorInput> auxiliary_inputs_;

  std::unique_ptr<LidarDeskewPolicy> deskew_policy_;
  std::unique_ptr<LidarFusionPolicy> fusion_policy_;
  std::unique_ptr<LidarFilterPolicy> filter_policy_;
  PoseBinsBuilder pose_bins_builder_;
  SyncGate sync_gate_;
  TsSanity ts_sanity_;
  DegradePolicy degrade_policy_;
  DtcReporter dtc_reporter_;
  std::shared_ptr<apollo::cyber::Writer<::apollo::drivers::PointCloud>> writer_;
  std::unique_ptr<apollo::cyber::Timer> fusion_flush_timer_;

  std::mutex pending_fusion_mutex_;
  std::mutex fusion_process_mutex_;
  std::deque<PendingFusionFrame> pending_fusion_frames_;

  std::vector<::apollo::drivers::PointXYZIT> full_pointcloud_buffer_;
  std::atomic<uint32_t> sequence_num_{0};
  std::atomic<uint64_t> frames_total_{0};
  std::atomic<uint64_t> total_input_points_{0};
  std::atomic<uint64_t> total_output_points_{0};
  std::atomic<uint64_t> total_missing_auxiliary_frames_{0};
  std::atomic<uint64_t> total_time_delta_exceeded_{0};
  std::atomic<uint64_t> total_fusion_deadline_exceeded_{0};
  std::atomic<uint64_t> total_pending_fusion_dropped_{0};
  std::atomic<uint64_t> total_pose_prefetch_timeouts_{0};
  mutable std::atomic<uint64_t> total_tf_query_failures_{0};
};

CYBER_REGISTER_COMPONENT(LidarUnifiedComponent)

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
