#pragma once

#include <string>

#include "modules/dreamview/backend/simulator/evaluation/metrics/metric.h"

namespace apollo {
namespace dreamview {

class MaxSimTimeMetric : public Metric {
 public:
  explicit MaxSimTimeMetric(double max_sim_time_sec)
      : max_sim_time_sec_(max_sim_time_sec) {}
  ~MaxSimTimeMetric() override = default;

  std::string Name() const override { return "max_sim_time"; }
  void Evaluate(const FrameContext& context) override;
  void Reset() override;

  bool IsTerminal() const override { return is_terminal_; }
  std::string TerminalReason() const override { return terminal_reason_; }

 private:
  double max_sim_time_sec_ = 0.0;
  bool is_terminal_ = false;
  std::string terminal_reason_;
};

}  // namespace dreamview
}  // namespace apollo
