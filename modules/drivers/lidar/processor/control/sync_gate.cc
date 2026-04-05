#include "modules/drivers/lidar/processor/control/sync_gate.h"

#include "cyber/cyber.h"

namespace apollo {
namespace drivers {
namespace lidar {

bool SyncGate::SelectFrames(double ref_timestamp,
                            const std::string& primary_sensor_id,
                            const std::vector<std::string>& auxiliary_topics,
                            uint32_t max_ref_time_delta_ms,
                            bool strict_auxiliary_sync,
                            const ResolveSensorIdByTopicFn& resolve_sensor_id,
                            const LookupNearestFrameFn& lookup_nearest_frame,
                            std::vector<FrameHandle>* frame_handles,
                            SyncGateMetrics* metrics) const {
  if (primary_sensor_id.empty() || !resolve_sensor_id ||
      !lookup_nearest_frame || frame_handles == nullptr || metrics == nullptr) {
    return false;
  }

  frame_handles->clear();
  metrics->expected_sensor_count = 1 + auxiliary_topics.size();
  metrics->matched_sensor_count = 0;
  metrics->missing_auxiliary_count = 0;
  metrics->time_delta_exceeded_count = 0;

  FrameHandle primary_handle;
  bool primary_time_delta_exceeded = false;
  if (!lookup_nearest_frame(primary_sensor_id, ref_timestamp,
                            max_ref_time_delta_ms, &primary_handle,
                            &primary_time_delta_exceeded)) {
    AERROR << "Primary sensor frame unavailable around reference timestamp";
    return false;
  }
  primary_handle.sensor_id = primary_sensor_id;
  primary_handle.is_primary = true;
  frame_handles->push_back(primary_handle);

  for (const auto& topic_name : auxiliary_topics) {
    std::string sensor_id;
    if (!resolve_sensor_id(topic_name, &sensor_id) || sensor_id.empty()) {
      ++metrics->missing_auxiliary_count;
      if (strict_auxiliary_sync) {
        AERROR << "Auxiliary topic has not resolved sensor id yet: "
               << topic_name;
        return false;
      }
      continue;
    }

    FrameHandle nearest_handle;
    bool time_delta_exceeded = false;
    if (!lookup_nearest_frame(sensor_id, ref_timestamp, max_ref_time_delta_ms,
                              &nearest_handle, &time_delta_exceeded)) {
      ++metrics->missing_auxiliary_count;
      if (time_delta_exceeded) {
        ++metrics->time_delta_exceeded_count;
      }
      if (strict_auxiliary_sync) {
        AERROR << "Auxiliary sensor sync failed: " << sensor_id;
        return false;
      }
      AWARN << "Skip auxiliary sensor " << sensor_id
            << " due to sync miss, topic=" << topic_name;
      continue;
    }

    nearest_handle.sensor_id = sensor_id;
    nearest_handle.is_primary = false;
    frame_handles->push_back(nearest_handle);
  }

  metrics->matched_sensor_count = frame_handles->size();
  return !frame_handles->empty();
}

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
