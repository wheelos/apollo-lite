#include "modules/dreamview/backend/simulator/evaluation/metrics/max_sim_time_metric.h"

namespace apollo {
namespace dreamview {

void MaxSimTimeMetric::Evaluate(const FrameContext& context) {
  if (is_terminal_) {
    return;
  }

  if (context.sim_time_sec >= max_sim_time_sec_) {
    is_terminal_ = true;
    terminal_reason_ = "Simulation time limit exceeded.";
  }
}

void MaxSimTimeMetric::Reset() {
  is_terminal_ = false;
  terminal_reason_.clear();
}

}  // namespace dreamview
}  // namespace apollo
