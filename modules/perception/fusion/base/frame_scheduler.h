#pragma once

#include <cstddef>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include "modules/perception/base/frame.h"

namespace apollo {
namespace perception {
namespace fusion {

// Callers serialize admission and watermark advancement. All inputs share a
// continuous Cartesian frame, SI units and the publication clock domain.
// Drain consumes every queued observation through its watermark; timestamps
// at or behind a completed watermark cannot be replayed into mutable tracks.
class FrameScheduler {
 public:
  explicit FrameScheduler(size_t capacity = 200) : capacity_(capacity) {}

  bool Add(const base::FrameConstPtr& frame);
  bool Drain(double timestamp, std::vector<base::FrameConstPtr>* frames);
  bool HasFreshSensor(const std::string& sensor, double timestamp,
                      double timeout) const;

 private:
  size_t capacity_;
  double watermark_ = -std::numeric_limits<double>::infinity();
  std::map<std::pair<double, std::string>, base::FrameConstPtr> pending_;
  std::map<std::string, double> sensor_timestamps_;
};

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
