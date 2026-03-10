#pragma once

#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "modules/dreamview/backend/simulator/base/entity.h"
#include "modules/dreamview/backend/simulator/evaluation/metrics/metric.h"

namespace apollo {
namespace dreamview {

class Evaluator : public SimEntity {
 public:
  struct Summary {
    bool has_terminal_event = false;
    std::string terminal_reason;
    double elapsed_time_sec = 0.0;
    std::size_t evaluator_count = 0;
    std::size_t metric_count = 0;
  };

  Evaluator() = default;
  ~Evaluator() override = default;

  bool Init() override { return true; }
  void AddMetric(std::unique_ptr<Metric> metric);
  void Evaluate(const FrameContext& context) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (has_terminal_event_) return;  // 已停止则不再计算

    for (auto& metric : metrics_) {
      metric->Evaluate(context);
      if (metric->IsTerminal()) {
        has_terminal_event_ = true;
        terminal_reason_ = metric->Name() + ": " + metric->TerminalReason();
        break;
      }
    }
  }

  void Step(double dt) override;
  void Reset() override;
  void Publish() override;

  bool HasTerminalEvent() const;
  std::string TerminalReason() const;
  Summary GetSummary() const;

 private:
  void UpdateTerminalStatusLocked();

  mutable std::mutex mutex_;
  double elapsed_time_sec_ = 0.0;
  bool has_terminal_event_ = false;
  std::string terminal_reason_;

  std::vector<std::unique_ptr<Metric>> metrics_;
};

}  // namespace dreamview
}  // namespace apollo
