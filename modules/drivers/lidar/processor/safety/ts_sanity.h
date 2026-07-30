#pragma once

#include <cstdint>

namespace apollo {
namespace drivers {
namespace lidar {

enum class TsSanityStatus {
  kOk = 0,
  kFirstFrame = 1,        // no previous timestamp to compare
  kIntervalTooShort = 2,  // frames arriving faster than min_interval_ms
  kIntervalTooLong = 3,   // gap exceeds max_interval_ms (late / dropped)
  kJump = 4,              // |delta| > max_jump_ms (clock reset / backward jump)
};

struct TsSanityResult {
  TsSanityStatus status = TsSanityStatus::kFirstFrame;
  double interval_ms = 0.0;    // |current - last| in milliseconds
  int consecutive_errors = 0;  // rolling count, reset on kOk
};

// Stateful per-sensor timestamp sanity checker.
// Thread-safe: external locking expected (one LidarUnifiedComponent per Proc).
class TsSanity {
 public:
  // Configure thresholds (milliseconds).
  void SetConfig(uint32_t min_interval_ms, uint32_t max_interval_ms,
                 uint32_t max_jump_ms);

  // Check a new frame timestamp (seconds). Updates internal state.
  TsSanityResult Check(double timestamp_seconds);

  // Reset to initial state (e.g. after degrade recovery).
  void Reset();

 private:
  double last_timestamp_s_ = -1.0;
  int consecutive_errors_ = 0;

  uint32_t min_interval_ms_ = 40;
  uint32_t max_interval_ms_ = 200;
  uint32_t max_jump_ms_ = 500;
};

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
