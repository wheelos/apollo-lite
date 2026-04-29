#pragma once

#include <algorithm>
#include <utility>
#include <vector>

#include "modules/perception/traffic_light/ports/provider_ports.h"

namespace apollo {
namespace perception {
namespace traffic_light {

class IHDMapSignalQuerier {
 public:
  virtual ~IHDMapSignalQuerier() = default;
  virtual bool QuerySignals(uint64_t timestamp_ns,
                            std::vector<SignalCandidate>* signals) = 0;
};

class HdMapProviderPort final : public IMapProviderPort {
 public:
  explicit HdMapProviderPort(IHDMapSignalQuerier* querier)
      : querier_(querier) {}

  void SetValidCacheWindowSec(double valid_cache_window_sec) {
    valid_cache_window_sec_ = std::max(0.0, valid_cache_window_sec);
  }

  bool PopulateSignals(PipelineContext* context) override {
    if (context == nullptr || querier_ == nullptr) {
      return false;
    }

    std::vector<SignalCandidate> signals;
    if (querier_->QuerySignals(context->timestamp, &signals)) {
      context->map_signals = std::move(signals);
      context->status.hdmap_available = !context->map_signals.empty();
      if (context->runtime_state != nullptr && !context->map_signals.empty()) {
        context->runtime_state->cached_signals = context->map_signals;
        context->runtime_state->last_signals_ts_sec =
            static_cast<double>(context->timestamp) * 1e-9;
      }
      return context->status.hdmap_available;
    }

    if (context->runtime_state != nullptr &&
        !context->runtime_state->cached_signals.empty()) {
      const double frame_ts_sec =
          static_cast<double>(context->timestamp) * 1e-9;
      const double dt =
          frame_ts_sec - context->runtime_state->last_signals_ts_sec;
      if (dt >= 0.0 && dt <= valid_cache_window_sec_) {
        context->map_signals = context->runtime_state->cached_signals;
        context->status.hdmap_available = true;
        context->AppendDegradeReason("hdmap fallback cache");
        return true;
      }
    }

    context->status.hdmap_available = false;
    context->AppendDegradeReason("hdmap unavailable");
    return false;
  }

 private:
  IHDMapSignalQuerier* querier_ = nullptr;
  double valid_cache_window_sec_ = 1.5;
};

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
