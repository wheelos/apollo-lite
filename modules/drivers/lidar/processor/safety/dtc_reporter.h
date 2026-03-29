#pragma once

#include <atomic>
#include <cstdint>
#include <string>

#include "modules/drivers/lidar/processor/safety/degrade_policy.h"
#include "modules/drivers/lidar/processor/safety/ts_sanity.h"

namespace apollo {
namespace drivers {
namespace lidar {

// Lightweight in-process DTC reporter (log + counters).
// In a later stage this can be bridged to a centralized diagnostics bus.
class DtcReporter {
 public:
  void ReportTsAnomaly(TsSanityStatus status, double interval_ms,
                       int consecutive_errors, const std::string& sensor_id);

  void ReportDegradeTransition(const DegradeEvent& event);

  uint64_t ts_anomaly_count() const;
  uint64_t degrade_transition_count() const;

 private:
  std::atomic<uint64_t> ts_anomaly_count_{0};
  std::atomic<uint64_t> degrade_transition_count_{0};
};

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
