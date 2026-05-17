#pragma once

#include <algorithm>
#include <chrono>
#include <mutex>

namespace apollo {
namespace cyber {
namespace common {

class TokenBucket {
 public:
  // rate: Number of cycles allowed per second (Hz)
  // burst_size: Allowed burst size (usually set to 1 or a small fraction of the
  // rate)
  TokenBucket(double rate, double burst_size)
      : rate_(rate), max_tokens_(burst_size), tokens_(0.0) {
    last_time_ = std::chrono::steady_clock::now();
  }

  // Modification rate
  void SetRate(double rate) {
    std::lock_guard<std::mutex> lock(mutex_);
    rate_ = rate;
  }

  // Attempt to acquire a token, non-blocking
  // Returns true to allow publishing, false to rate-limit and drop tokens
  bool TryConsume(double tokens_needed = 1.0) {
    std::lock_guard<std::mutex> lock(mutex_);

    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = now - last_time_;
    last_time_ = now;

    // Replenishment Tokens
    tokens_ += elapsed.count() * rate_;
    // Limit maximum tokens (prevent burst after long idle)
    if (tokens_ > max_tokens_) {
      tokens_ = max_tokens_;
    }

    if (tokens_ >= tokens_needed) {
      tokens_ -= tokens_needed;
      return true;
    }
    return false;
  }

 private:
  double rate_;
  double max_tokens_;
  double tokens_;
  std::chrono::steady_clock::time_point last_time_;
  std::mutex mutex_;
};

}  // namespace common
}  // namespace cyber
}  // namespace apollo
