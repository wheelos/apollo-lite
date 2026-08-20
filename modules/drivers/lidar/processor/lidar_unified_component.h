// Copyright 2026 WheelOS All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <boost/circular_buffer.hpp>

#include "Eigen/Eigen"

#include "wheelos_msgs/sensor_msgs/pointcloud.pb.h"
#include "modules/drivers/lidar/proto/lidar_unified_component_config.pb.h"

#include "cyber/cyber.h"

#ifndef FRIEND_TEST
#define FRIEND_TEST(test_case_name, test_name) \
  friend class test_case_name##_##test_name##_Test
#endif
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
  LidarUnifiedComponent() = default;
  ~LidarUnifiedComponent() override;
  bool Init() override;
  bool Proc(const std::shared_ptr<::apollo::drivers::PointCloud>& point_cloud)
      override;

  bool OnReceiveMainLidar(const PointCloudConstPtr& point_cloud);

 private:
  FRIEND_TEST(LidarUnifiedComponentTest, RejectsPrimarySensorIdDrift);
  FRIEND_TEST(LidarUnifiedComponentTest,
              FindsNearestFrameFromOutOfOrderBuffer);
  FRIEND_TEST(LidarUnifiedComponentTest,
              ReportsTimeDeltaExceededForNearestFrame);
  FRIEND_TEST(LidarUnifiedComponentTest, AppliesFixedDelayDuringFrameLookup);
  FRIEND_TEST(LidarUnifiedComponentTest, PrefersMaximumIntervalOverlap);
  FRIEND_TEST(LidarUnifiedComponentTest, BreaksOverlapTieByAnchorDistance);
  FRIEND_TEST(LidarUnifiedComponentTest, ExcludesFramesOnlyAfterCommit);
  FRIEND_TEST(LidarUnifiedComponentTest,
              OffCompensationUsesStaticExtrinsicOnly);
  FRIEND_TEST(LidarUnifiedComponentTest, UpdatesSensorTimingModel);
  FRIEND_TEST(LidarUnifiedComponentTest,
              KeepsOnlineOffsetDisabledForMatching);
  FRIEND_TEST(LidarUnifiedComponentTest,
              UpdatesLargeFixedDelayWhenInnovationIsWithinLimit);
  FRIEND_TEST(LidarUnifiedComponentTest,
              CollectNearestFramesSkipsLowQualityAuxiliary);
  FRIEND_TEST(LidarUnifiedComponentTest,
              CollectNearestFramesMatchesThreeSensors);
  FRIEND_TEST(LidarUnifiedComponentTest,
              CollectNearestFramesAllowsMissingAuxiliary);
  FRIEND_TEST(LidarUnifiedComponentTest,
              CollectNearestFramesFailsStrictMissingAuxiliary);
  FRIEND_TEST(LidarUnifiedComponentTest, RejectsDuplicateAuxiliaryTopics);
  FRIEND_TEST(LidarUnifiedComponentTest, RejectsImpossibleScanDurations);
  FRIEND_TEST(LidarUnifiedComponentTest, EstimatesOverlapQualityWeight);

  enum class FrameLookupFailureReason {
    kNone = 0,
    kBufferEmpty = 1,
    kTimeDeltaExceeded = 2,
  };

  struct SensorState {
    explicit SensorState(size_t capacity = 1) : frames(capacity) {}

    boost::circular_buffer<std::shared_ptr<BufferedFrame>> frames;
    std::set<uint64_t> consumed_frame_ids;
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
    LidarUnifiedComponentConfig::TimeSettings time_settings;
  };

  struct FrameMetrics {
    uint32_t primary_sequence_num = 0;
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
    double frame_selection_ms = 0.0;
    double pose_bins_ms = 0.0;
    double reference_pose_ms = 0.0;
    double fusion_ms = 0.0;
    double filter_ms = 0.0;
    double output_build_ms = 0.0;
    double writer_ms = 0.0;
    double processing_ms = 0.0;
    double end_to_end_ms = 0.0;
    bool fusion_deadline_exceeded = false;
  };

  struct PendingFusionFrame {
    PointCloudConstPtr main_frame;
    std::string primary_sensor_id;
    double reference_timestamp_sec = 0.0;
    std::shared_ptr<const BufferedFrame> primary_buffered_frame;
    double enqueue_time_sec = 0.0;
    double deadline_sec = 0.0;
  };

  bool PrepareBufferedFrame(const std::string& sensor_id,
                            const PointCloudConstPtr& point_cloud,
                            const LidarUnifiedComponentConfig::TimeSettings&
                                time_settings,
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

  bool CollectNearestFrames(const std::string& primary_sensor_id,
                            const std::shared_ptr<const BufferedFrame>&
                                primary_buffered_frame,
                            std::vector<FrameHandle>* frame_handles,
                            FrameMetrics* frame_metrics);
  void EnqueuePendingFusionFrame(
      const PointCloudConstPtr& main_frame,
      const std::string& primary_sensor_id,
      const std::shared_ptr<const BufferedFrame>& primary_buffered_frame);
  void TryFlushPendingFusionFrames(bool flush_expired_only);
  void OnFusionFlushTimer();
  bool ProcessFusionFrame(const PendingFusionFrame& pending_frame,
                          std::vector<FrameHandle> frame_handles,
                          FrameMetrics frame_metrics);
  bool FindNearestFrame(const std::shared_ptr<SensorState>& sensor_state,
                        const std::string& sensor_id,
                        const TimeContract& reference_time,
                        uint32_t max_ref_time_delta_ms,
                        FrameHandle* frame_handle,
                        FrameLookupFailureReason* failure_reason) const;
  void CommitSelectedFrames(const std::vector<FrameHandle>& frame_handles);
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

  apollo::transform::BufferInterface* tf_buffer_ = nullptr;
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
  std::map<std::string, std::shared_ptr<const Eigen::Affine3d>>
      static_extrinsics_;
  mutable std::mutex static_extrinsics_mutex_;

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
  std::atomic<uint64_t> buffered_frame_id_{0};
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
