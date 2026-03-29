#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "modules/drivers/lidar/processor/control/frame_handle.h"

namespace apollo {
namespace drivers {
namespace lidar {

struct SyncGateMetrics {
  size_t expected_sensor_count = 0;
  size_t matched_sensor_count = 0;
  size_t missing_auxiliary_count = 0;
  size_t time_delta_exceeded_count = 0;
};

class SyncGate {
 public:
  using ResolveSensorIdByTopicFn = std::function<bool(
      const std::string& topic_name, std::string* sensor_id)>;

  using LookupNearestFrameFn = std::function<bool(
      const std::string& sensor_id, double ref_timestamp,
      uint32_t max_ref_time_delta_ms,
      std::shared_ptr<const apollo::drivers::PointCloud>* frame,
      bool* time_delta_exceeded)>;

  bool SelectFrames(double ref_timestamp, const std::string& primary_sensor_id,
                    const std::vector<std::string>& auxiliary_topics,
                    uint32_t max_ref_time_delta_ms, bool strict_auxiliary_sync,
                    const ResolveSensorIdByTopicFn& resolve_sensor_id,
                    const LookupNearestFrameFn& lookup_nearest_frame,
                    std::vector<FrameHandle>* frame_handles,
                    SyncGateMetrics* metrics) const;
};

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
