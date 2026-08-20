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

#include "modules/drivers/lidar/processor/safety/dtc_reporter.h"

#include <string>

#include "cyber/cyber.h"

namespace apollo {
namespace drivers {
namespace lidar {

namespace {

const char* TsStatusName(TsSanityStatus status) {
  switch (status) {
    case TsSanityStatus::kOk:
      return "ok";
    case TsSanityStatus::kFirstFrame:
      return "first_frame";
    case TsSanityStatus::kIntervalTooShort:
      return "interval_too_short";
    case TsSanityStatus::kIntervalTooLong:
      return "interval_too_long";
    case TsSanityStatus::kJump:
      return "jump";
    default:
      return "unknown";
  }
}

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

const char* DegradeReasonName(DegradeReason reason) {
  switch (reason) {
    case DegradeReason::kNone:
      return "none";
    case DegradeReason::kTsAnomaly:
      return "ts_anomaly";
    case DegradeReason::kManualReset:
      return "manual_reset";
    default:
      return "unknown";
  }
}

}  // namespace

void DtcReporter::ReportTsAnomaly(TsSanityStatus status, double interval_ms,
                                  int consecutive_errors,
                                  const std::string& sensor_id) {
  ts_anomaly_count_.fetch_add(1);
  AWARN << "[DTC][LIDAR_TS_SANITY] sensor=" << sensor_id
        << ", status=" << TsStatusName(status)
        << ", interval_ms=" << interval_ms
        << ", consecutive_errors=" << consecutive_errors;
}

void DtcReporter::ReportDegradeTransition(const DegradeEvent& event) {
  if (!event.mode_changed) {
    return;
  }

  degrade_transition_count_.fetch_add(1);
  AERROR << "[DTC][LIDAR_DEGRADE] mode " << DegradeModeName(event.previous_mode)
         << " -> " << DegradeModeName(event.new_mode)
         << ", reason=" << DegradeReasonName(event.reason);
}

uint64_t DtcReporter::ts_anomaly_count() const {
  return ts_anomaly_count_.load();
}

uint64_t DtcReporter::degrade_transition_count() const {
  return degrade_transition_count_.load();
}

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
