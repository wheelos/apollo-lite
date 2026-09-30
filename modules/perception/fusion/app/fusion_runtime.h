#pragma once

#include <functional>
#include <limits>
#include <map>
#include <string>

#include "modules/perception/fusion/base/fusion_frame.h"

namespace apollo {
namespace perception {
namespace fusion {

// Transport-independent runtime; its owner serializes Add, Tick and errors.
// Tick publishes at now - reorder_window; source errors clear only after a
// newer observation is processed. Failed mutable cycles latch until Init.
class FusionRuntime {
 public:
  using Processor = std::function<bool(FusionFrame*)>;
  using SourceValidator = std::function<bool(const std::string&)>;
  bool Init(const std::string& main_sensor, double reorder_window,
            Processor processor, SourceValidator source_validator = {});
  bool Add(const base::FramePtr& frame);
  bool Tick(double now, FusionFrame* output);
  bool ReportSourceError(const std::string& sensor, double timestamp);

 private:
  std::string main_sensor_;
  double reorder_window_ = 0.05;
  double last_tick_ = -std::numeric_limits<double>::infinity();
  std::map<std::string, double> source_errors_;
  std::map<std::string, double> processed_timestamps_;
  bool has_publish_pose_ = false;
  Eigen::Affine3d publish_pose_ = Eigen::Affine3d::Identity();
  base::SensorInfo publish_sensor_info_;
  bool faulted_ = false;
  Processor processor_;
  SourceValidator source_validator_;
};

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
