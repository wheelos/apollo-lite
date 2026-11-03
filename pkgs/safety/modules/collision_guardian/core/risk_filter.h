#pragma once

#include <cstdint>

namespace apollo {
namespace collision_guardian {

enum class FilterState {
  kClear,
  kSuspected,
  kConfirmed,
  kReleasing,
  kFault,
};

struct RiskFilterConfig {
  double prior_probability = 0.1;
  double hit_probability = 0.7;
  double miss_probability = 0.3;
  double temporal_decay = 0.9;
  double suspected_probability = 0.5;
  double trigger_probability = 0.8;
  double release_probability = 0.3;
  uint32_t min_confirmation_frames = 3;
  uint32_t min_release_frames = 3;
};

struct RiskFilterResult {
  FilterState state = FilterState::kClear;
  double probability = 0.0;
};

class RiskFilter {
 public:
  explicit RiskFilter(const RiskFilterConfig& config);

  RiskFilterResult Update(bool occupied, bool input_valid);

 private:
  static double Logit(double probability);
  static double Probability(double log_odds);

  RiskFilterConfig config_;
  FilterState state_ = FilterState::kClear;
  double prior_log_odds_ = 0.0;
  double log_odds_ = 0.0;
  uint32_t confirmation_frames_ = 0;
  uint32_t release_frames_ = 0;
};

}  // namespace collision_guardian
}  // namespace apollo
