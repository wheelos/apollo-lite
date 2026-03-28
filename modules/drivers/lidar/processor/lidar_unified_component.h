#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
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
#include "modules/drivers/lidar/processor/policy/lidar_policy_interface.h"
#include "modules/transform/buffer.h"

namespace apollo {
namespace drivers {
namespace lidar {

class LidarUnifiedComponent : public apollo::cyber::Component<PointCloud> {
 public:
  using PointCloudConstPtr = std::shared_ptr<const PointCloud>;

  bool Init() override;
  bool Proc(const std::shared_ptr<PointCloud>& point_cloud) override;

  bool OnReceiveMainLidar(const PointCloudConstPtr& point_cloud);

 private:
  enum class FrameLookupFailureReason {
    kNone = 0,
    kBufferEmpty = 1,
    kTimeDeltaExceeded = 2,
  };

  struct SensorBuffer {
    explicit SensorBuffer(size_t capacity = 1) : queue(capacity) {}
    boost::circular_buffer<PointCloudConstPtr> queue;
    mutable std::mutex mutex;
  };

  struct SensorInput {
    std::string topic_name;
  };

  struct SensorFrame {
    std::string sensor_id;
    PointCloudConstPtr point_cloud;
    bool is_primary = false;
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
  };

  void PushToBuffer(const std::string& sensor_id,
                    const PointCloudConstPtr& point_cloud);
  void OnAuxiliaryLidarMessage(const std::string& topic_name,
                               const PointCloudConstPtr& point_cloud);
  bool ValidateConfig() const;
  std::string ResolveSensorId(const PointCloudConstPtr& point_cloud,
                              const std::string& topic_name) const;
  std::string MakeFallbackSensorId(const std::string& topic_name) const;
  std::shared_ptr<SensorBuffer> EnsureSensorBuffer(
      const std::string& sensor_id);
  std::shared_ptr<SensorBuffer> GetSensorBuffer(
      const std::string& sensor_id) const;

  bool CollectNearestFrames(double ref_timestamp,
                            const std::string& primary_sensor_id,
                            std::vector<SensorFrame>* frames,
                            FrameMetrics* frame_metrics);
  bool FindNearestFrame(const std::shared_ptr<SensorBuffer>& sensor_buffer,
                        double ref_timestamp, uint32_t max_ref_time_delta_ms,
                        PointCloudConstPtr* nearest_frame,
                        FrameLookupFailureReason* failure_reason) const;

  bool BuildUnifiedPointCloud(const PointCloudConstPtr& main_frame,
                              const std::vector<SensorFrame>& frames,
                              FrameMetrics* frame_metrics,
                              std::shared_ptr<PointCloud>* output);
  void LogFrameMetrics(const FrameMetrics& frame_metrics);

 private:
  LidarUnifiedComponentConfig config_;
  std::string primary_sensor_id_;
  size_t sensor_buffer_capacity_ = 1;

  apollo::transform::Buffer* tf_buffer_ = nullptr;
  mutable std::mutex sensor_registry_mutex_;

  std::map<std::string, std::shared_ptr<SensorBuffer>> raw_cloud_buffers_;
  std::map<std::string, std::string> auxiliary_sensor_ids_by_topic_;
  std::map<std::string, std::shared_ptr<apollo::cyber::Reader<PointCloud>>>
      auxiliary_readers_;
  std::vector<SensorInput> auxiliary_inputs_;

  std::unique_ptr<LidarDeskewPolicy> deskew_policy_;
  std::unique_ptr<LidarFusionPolicy> fusion_policy_;
  std::unique_ptr<LidarFilterPolicy> filter_policy_;
  std::shared_ptr<apollo::cyber::Writer<PointCloud>> writer_;

  std::vector<apollo::drivers::PointXYZIT> full_pointcloud_buffer_;
  std::atomic<uint32_t> sequence_num_{0};
  std::atomic<uint64_t> frames_total_{0};
  std::atomic<uint64_t> total_input_points_{0};
  std::atomic<uint64_t> total_output_points_{0};
  std::atomic<uint64_t> total_missing_auxiliary_frames_{0};
  std::atomic<uint64_t> total_time_delta_exceeded_{0};
  mutable std::atomic<uint64_t> total_tf_query_failures_{0};
};

CYBER_REGISTER_COMPONENT(LidarUnifiedComponent)

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
