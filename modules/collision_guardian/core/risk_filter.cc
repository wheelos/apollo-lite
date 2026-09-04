#include "modules/collision_guardian/core/risk_filter.h"

#include <algorithm>
#include <cmath>

namespace apollo {
namespace collision_guardian {
namespace {

constexpr double kMinProbability = 1.0e-6;
constexpr double kMaxProbability = 1.0 - kMinProbability;
constexpr double kMinLogOdds = -20.0;
constexpr double kMaxLogOdds = 20.0;

}  // namespace

RiskFilter::RiskFilter(const RiskFilterConfig& config) : config_(config) {
  prior_log_odds_ = Logit(config_.prior_probability);
  log_odds_ = prior_log_odds_;
}

RiskFilterResult RiskFilter::Update(bool occupied, bool input_valid) {
  if (!input_valid) {
    state_ = FilterState::kFault;
    confirmation_frames_ = 0;
    release_frames_ = 0;
    log_odds_ = prior_log_odds_;
    return {state_, Probability(log_odds_)};
  }

  if (state_ == FilterState::kFault) {
    state_ = FilterState::kClear;
  }

  const double observation_probability =
      occupied ? config_.hit_probability : config_.miss_probability;
  log_odds_ =
      prior_log_odds_ +
      config_.temporal_decay * (log_odds_ - prior_log_odds_) +
      Logit(observation_probability) - prior_log_odds_;
  log_odds_ = std::clamp(log_odds_, kMinLogOdds, kMaxLogOdds);
  const double probability = Probability(log_odds_);

  confirmation_frames_ = occupied ? confirmation_frames_ + 1 : 0;

  switch (state_) {
    case FilterState::kClear:
      if (probability >= config_.suspected_probability) {
        state_ = FilterState::kSuspected;
      }
      break;
    case FilterState::kSuspected:
      if (probability >= config_.trigger_probability &&
          confirmation_frames_ >= config_.min_confirmation_frames) {
        state_ = FilterState::kConfirmed;
      } else if (probability < config_.release_probability) {
        state_ = FilterState::kClear;
      }
      break;
    case FilterState::kConfirmed:
      if (probability < config_.release_probability) {
        state_ = FilterState::kReleasing;
        release_frames_ = 1;
      }
      break;
    case FilterState::kReleasing:
      if (probability >= config_.trigger_probability) {
        state_ = FilterState::kConfirmed;
        release_frames_ = 0;
      } else if (probability < config_.release_probability) {
        ++release_frames_;
        if (release_frames_ >= config_.min_release_frames) {
          state_ = FilterState::kClear;
          release_frames_ = 0;
        }
      } else {
        release_frames_ = 0;
      }
      break;
    case FilterState::kFault:
      break;
  }

  return {state_, probability};
}

double RiskFilter::Logit(double probability) {
  const double clamped =
      std::clamp(probability, kMinProbability, kMaxProbability);
  return std::log(clamped / (1.0 - clamped));
}

double RiskFilter::Probability(double log_odds) {
  return 1.0 / (1.0 + std::exp(-log_odds));
}

}  // namespace collision_guardian
}  // namespace apollo
