// Copyright 2026 WheelOS. All Rights Reserved.
#pragma once

#include <deque>

#include "modules/transform/timed_transform_resolver.h"

namespace apollo {
namespace transform {
namespace internal {

class TransformCache {
 public:
  void AddTransform(const StampedTransform& transform);
  bool QueryTransform(double timestamp, StampedTransform* transform,
                      double max_duration = 0.0,
                      double* extrapolation_duration = nullptr);
  void SetCacheDuration(double duration) { cache_duration_ = duration; }

 private:
  std::deque<StampedTransform> transforms_;
  double cache_duration_ = 1.0;
};

}  // namespace internal
}  // namespace transform
}  // namespace apollo
