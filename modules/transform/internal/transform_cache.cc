// Copyright 2026 WheelOS. All Rights Reserved.
#include "modules/transform/internal/transform_cache.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "cyber/common/log.h"

namespace apollo {
namespace transform {
namespace internal {
namespace {

constexpr size_t kMaxExtrapolationAnchors = 4;

Eigen::Quaterniond Slerp(const Eigen::Quaterniond& source, double t,
                         const Eigen::Quaterniond& other) {
  const double one = 1.0 - std::numeric_limits<double>::epsilon();
  const double d = source.x() * other.x() + source.y() * other.y() +
                   source.z() * other.z() + source.w() * other.w();
  const double abs_d = std::abs(d);
  double scale0;
  double scale1;
  if (abs_d >= one) {
    scale0 = 1.0 - t;
    scale1 = t;
  } else {
    const double theta = std::acos(abs_d);
    const double sin_theta = std::sin(theta);
    scale0 = std::sin((1.0 - t) * theta) / sin_theta;
    scale1 = std::sin(t * theta) / sin_theta;
  }
  if (d < 0) {
    scale1 = -scale1;
  }
  return Eigen::Quaterniond(scale0 * source.w() + scale1 * other.w(),
                            scale0 * source.x() + scale1 * other.x(),
                            scale0 * source.y() + scale1 * other.y(),
                            scale0 * source.z() + scale1 * other.z());
}

void InterpolateStampedTransform(const StampedTransform& first,
                                 const StampedTransform& second,
                                 double timestamp,
                                 StampedTransform* transform) {
  CHECK_NOTNULL(transform);
  const double duration = second.timestamp - first.timestamp;
  if (duration <= std::numeric_limits<double>::epsilon()) {
    *transform = second;
    transform->timestamp = timestamp;
    return;
  }
  const double ratio = (timestamp - first.timestamp) / duration;
  transform->timestamp = timestamp;
  transform->rotation = Slerp(first.rotation, ratio, second.rotation);
  transform->translation.x() =
      first.translation.x() * (1 - ratio) + second.translation.x() * ratio;
  transform->translation.y() =
      first.translation.y() * (1 - ratio) + second.translation.y() * ratio;
  transform->translation.z() =
      first.translation.z() * (1 - ratio) + second.translation.z() * ratio;
}

}  // namespace

void TransformCache::AddTransform(const StampedTransform& transform) {
  if (transforms_.empty()) {
    transforms_.push_back(transform);
    return;
  }
  double delt = transform.timestamp - transforms_.back().timestamp;
  if (delt < 0.0) {
    ADEBUG << "Ignore out-of-order transform at " << transform.timestamp
           << " for transform cache.";
    return;
  }
  if (delt <= std::numeric_limits<double>::epsilon()) {
    transforms_.back() = transform;
    return;
  }
  do {
    delt = transform.timestamp - transforms_.front().timestamp;
    if (delt < cache_duration_ &&
        transforms_.size() < kMaxExtrapolationAnchors) {
      break;
    }
    transforms_.pop_front();
  } while (!transforms_.empty());
  transforms_.push_back(transform);
}

bool TransformCache::QueryTransform(double timestamp,
                                    StampedTransform* transform,
                                    double max_duration,
                                    double* extrapolation_duration) {
  if (extrapolation_duration != nullptr) {
    *extrapolation_duration = 0.0;
  }
  if (transforms_.empty() || transform == nullptr) {
    return false;
  }
  if (timestamp < transforms_.front().timestamp) {
    ADEBUG << "Transform cache miss: requested timestamp " << timestamp
           << " is earlier than cached range.";
    return false;
  }
  const double delt = timestamp - transforms_.back().timestamp;
  if (delt > max_duration) {
    ADEBUG << "Transform cache miss: requested timestamp is " << delt
           << "s newer than the latest cached sample.";
    return false;
  }
  const int size = static_cast<int>(transforms_.size());
  if (size == 1) {
    if (extrapolation_duration != nullptr) {
      *extrapolation_duration =
          std::max(0.0, timestamp - transforms_.back().timestamp);
    }
    *transform = transforms_.back();
    transform->timestamp = timestamp;
    ADEBUG << "Reuse latest cached transform at "
           << transforms_.back().timestamp << " for " << timestamp << ".";
    return true;
  }
  if (timestamp >= transforms_.back().timestamp) {
    if (extrapolation_duration != nullptr) {
      *extrapolation_duration = timestamp - transforms_.back().timestamp;
    }
    InterpolateStampedTransform(transforms_[size - 2], transforms_[size - 1],
                                timestamp, transform);
    ADEBUG << "Extrapolate cached transform at " << timestamp
           << " using samples at " << transforms_[size - 2].timestamp << " and "
           << transforms_[size - 1].timestamp << ".";
    return true;
  }
  for (int index = 1; index < size; ++index) {
    const auto& previous = transforms_[index - 1];
    const auto& current = transforms_[index];
    if (timestamp <= current.timestamp) {
      if (std::abs(timestamp - current.timestamp) <=
          std::numeric_limits<double>::epsilon()) {
        *transform = current;
        transform->timestamp = timestamp;
      } else {
        InterpolateStampedTransform(previous, current, timestamp, transform);
      }
      return true;
    }
  }
  return false;
}

}  // namespace internal
}  // namespace transform
}  // namespace apollo
