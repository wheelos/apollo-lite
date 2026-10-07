#include "modules/dreamview/backend/simulator/evaluation/evaluation_manager.h"

namespace apollo {
namespace dreamview {

void EvaluationManager::AddMetric(std::unique_ptr<MetricBase> metric) {
  std::lock_guard<std::mutex> lock(mutex_);
  metrics_.push_back(std::move(metric));
}

void EvaluationManager::Step(double dt) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (dt < 0.0) {
    dt = 0.0;
  }
  elapsed_time_sec_ += dt;

  // Step 里只做时间推进，不建议在这里调 Evaluate
  // 因为此时其他 Entity 的 Step 可能还没跑完，数据不全
}

void EvaluationManager::Reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  elapsed_time_sec_ = 0.0;
  has_terminal_event_ = false;
  terminal_reason_.clear();

  for (auto& metric : metrics_) {
    metric->Reset();
  }
}

void EvaluationManager::Publish() {
  // Aggregate scores or metrics from all evaluators and publish them via Cyber
  // if necessary To be implemented as the evaluation system grows
}

bool EvaluationManager::HasTerminalEvent() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return has_terminal_event_;
}

std::string EvaluationManager::TerminalReason() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return terminal_reason_;
}

EvaluationManager::Summary EvaluationManager::GetSummary() const {
  std::lock_guard<std::mutex> lock(mutex_);
  Summary summary;
  summary.has_terminal_event = has_terminal_event_;
  summary.terminal_reason = terminal_reason_;
  summary.elapsed_time_sec = elapsed_time_sec_;
  summary.metric_count = metrics_.size();
  return summary;
}

void EvaluationManager::UpdateTerminalStatusLocked() {
  if (has_terminal_event_) {
    return;
  }
  for (const auto& metric : metrics_) {
    if (metric->IsTerminal()) {
      has_terminal_event_ = true;
      terminal_reason_ = metric->TerminalReason();
      return;
    }
  }
}

}  // namespace dreamview
}  // namespace apollo
