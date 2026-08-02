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
