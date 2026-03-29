#include "modules/drivers/lidar/processor/safety/dtc_reporter.h"

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
