#include "modules/transform/timed_transform_resolver.h"

#include <algorithm>
#include <cmath>
#include <iterator>

#include "cyber/common/log.h"

namespace apollo {
namespace transform {

namespace {

constexpr double kTimestampEpsilonSec = 1e-9;

}  // namespace

TimedTransformResolver::PoseSample TimedTransformResolver::FromAffine(
    double timestamp_sec, const Eigen::Affine3d& pose) {
  PoseSample sample;
  sample.timestamp_sec = timestamp_sec;
  sample.tx = pose.translation().x();
  sample.ty = pose.translation().y();
  sample.tz = pose.translation().z();
  Eigen::Quaterniond rotation(pose.linear());
  rotation.normalize();
  sample.qx = rotation.x();
  sample.qy = rotation.y();
  sample.qz = rotation.z();
  sample.qw = rotation.w();
  return sample;
}

Eigen::Affine3d TimedTransformResolver::ToAffine(const PoseSample& sample) {
  return Eigen::Translation3d(sample.tx, sample.ty, sample.tz) *
         Eigen::Quaterniond(sample.qw, sample.qx, sample.qy, sample.qz);
}

void TimedTransformResolver::ClearCache() {
  std::lock_guard<std::mutex> lock(mutex_);
  samples_.clear();
}

bool TimedTransformResolver::InsertSample(double timestamp_sec,
                                          const Eigen::Affine3d& pose) {
  std::lock_guard<std::mutex> lock(mutex_);
  const PoseSample sample = FromAffine(timestamp_sec, pose);
  const auto it = std::lower_bound(
      samples_.begin(), samples_.end(), timestamp_sec,
      [](const PoseSample& lhs, double rhs_timestamp_sec) {
        return lhs.timestamp_sec + kTimestampEpsilonSec < rhs_timestamp_sec;
      });
  if (it != samples_.end() &&
      std::fabs(it->timestamp_sec - timestamp_sec) <= kTimestampEpsilonSec) {
    *it = sample;
  } else {
    samples_.insert(it, sample);
  }

  while (!samples_.empty() &&
         (samples_.back().timestamp_sec - samples_.front().timestamp_sec) >
             options_.cache_duration_sec) {
    samples_.erase(samples_.begin());
  }
  return !samples_.empty();
}

bool TimedTransformResolver::Prefetch(double timestamp_sec) {
  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  double resolved_timestamp_sec = timestamp_sec;
  if (!QueryTransform(timestamp_sec, &pose, &resolved_timestamp_sec)) {
    return false;
  }
  return InsertSample(resolved_timestamp_sec, pose);
}

bool TimedTransformResolver::PrefetchBatch(
    const std::vector<double>& timestamps_sec) {
  std::vector<double> ordered_timestamps = timestamps_sec;
  std::sort(ordered_timestamps.begin(), ordered_timestamps.end());
  ordered_timestamps.erase(
      std::unique(ordered_timestamps.begin(), ordered_timestamps.end(),
                  [](double lhs, double rhs) {
                    return std::fabs(lhs - rhs) <= kTimestampEpsilonSec;
                  }),
      ordered_timestamps.end());
  for (const double timestamp_sec : ordered_timestamps) {
    Eigen::Affine3d cached_pose = Eigen::Affine3d::Identity();
    if (QueryCachedStrict(timestamp_sec, &cached_pose)) {
      continue;
    }
    if (!Prefetch(timestamp_sec)) {
      return false;
    }
  }
  return true;
}

bool TimedTransformResolver::QueryCachedInternal(
    double timestamp_sec, double max_extrapolation_sec, Eigen::Affine3d* pose,
    TransformResolveStatus* status) const {
  const auto interpolate_pose = [timestamp_sec](const PoseSample& left,
                                                const PoseSample& right)
      -> Eigen::Affine3d {
    const double duration = right.timestamp_sec - left.timestamp_sec;
    if (std::fabs(duration) <= kTimestampEpsilonSec) {
      return ToAffine(right);
    }

    const double ratio = (timestamp_sec - left.timestamp_sec) / duration;
    Eigen::Quaterniond left_rotation(left.qw, left.qx, left.qy, left.qz);
    Eigen::Quaterniond right_rotation(right.qw, right.qx, right.qy, right.qz);
    if (left_rotation.dot(right_rotation) < 0.0) {
      right_rotation.coeffs() *= -1.0;
    }

    const Eigen::Vector3d translation(left.tx + ratio * (right.tx - left.tx),
                                      left.ty + ratio * (right.ty - left.ty),
                                      left.tz + ratio * (right.tz - left.tz));
    return Eigen::Affine3d(Eigen::Translation3d(translation) *
                 left_rotation.slerp(ratio, right_rotation));
  };

  if (pose == nullptr) {
    if (status != nullptr) {
      *status = TransformResolveStatus::kQueryFailed;
    }
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (samples_.empty()) {
    if (status != nullptr) {
      *status = TransformResolveStatus::kEmpty;
    }
    return false;
  }

  if (timestamp_sec + kTimestampEpsilonSec < samples_.front().timestamp_sec) {
    if (status != nullptr) {
      *status = TransformResolveStatus::kTooEarly;
    }
    return false;
  }

  if (timestamp_sec > samples_.back().timestamp_sec + kTimestampEpsilonSec) {
    if ((timestamp_sec - samples_.back().timestamp_sec) >
        max_extrapolation_sec) {
      if (status != nullptr) {
        *status = TransformResolveStatus::kTooOld;
      }
      return false;
    }
    if (samples_.size() == 1) {
      *pose = ToAffine(samples_.back());
    } else {
      *pose = interpolate_pose(samples_[samples_.size() - 2], samples_.back());
    }
    if (status != nullptr) {
      *status = TransformResolveStatus::kOk;
    }
    return true;
  }

  const auto it = std::lower_bound(
      samples_.begin(), samples_.end(), timestamp_sec,
      [](const PoseSample& lhs, double rhs_timestamp_sec) {
        return lhs.timestamp_sec + kTimestampEpsilonSec < rhs_timestamp_sec;
      });
  if (it == samples_.begin()) {
    *pose = ToAffine(*it);
    if (status != nullptr) {
      *status = TransformResolveStatus::kOk;
    }
    return true;
  }
  if (it == samples_.end()) {
    *pose = ToAffine(samples_.back());
    if (status != nullptr) {
      *status = TransformResolveStatus::kOk;
    }
    return true;
  }
  if (std::fabs(it->timestamp_sec - timestamp_sec) <= kTimestampEpsilonSec) {
    *pose = ToAffine(*it);
    if (status != nullptr) {
      *status = TransformResolveStatus::kOk;
    }
    return true;
  }

  const auto left = std::prev(it);
  const double left_time = left->timestamp_sec;
  const double right_time = it->timestamp_sec;
  if (std::fabs(right_time - left_time) <= kTimestampEpsilonSec) {
    *pose = ToAffine(*it);
    if (status != nullptr) {
      *status = TransformResolveStatus::kOk;
    }
    return true;
  }

  *pose = interpolate_pose(*left, *it);
  if (status != nullptr) {
    *status = TransformResolveStatus::kOk;
  }
  return true;
}

bool TimedTransformResolver::QueryCached(
    double timestamp_sec, Eigen::Affine3d* pose,
    TransformResolveStatus* status) const {
  return QueryCachedInternal(timestamp_sec, options_.max_extrapolation_sec,
                             pose, status);
}

bool TimedTransformResolver::QueryCachedStrict(
    double timestamp_sec, Eigen::Affine3d* pose,
    TransformResolveStatus* status) const {
  return QueryCachedInternal(timestamp_sec, 0.0, pose, status);
}

bool TimedTransformResolver::QueryCachedBatch(
    const std::vector<double>& timestamps_sec,
    std::vector<Eigen::Affine3d>* poses,
    TransformResolveStatus* status) const {
  if (poses == nullptr) {
    if (status != nullptr) {
      *status = TransformResolveStatus::kQueryFailed;
    }
    return false;
  }
  poses->clear();
  poses->reserve(timestamps_sec.size());
  for (const double timestamp_sec : timestamps_sec) {
    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    TransformResolveStatus local_status = TransformResolveStatus::kOk;
    if (!QueryCached(timestamp_sec, &pose, &local_status)) {
      if (status != nullptr) {
        *status = local_status;
      }
      poses->clear();
      return false;
    }
    poses->push_back(pose);
  }
  if (status != nullptr) {
    *status = TransformResolveStatus::kOk;
  }
  return true;
}

bool TimedTransformResolver::QueryCachedBatchStrict(
    const std::vector<double>& timestamps_sec,
    std::vector<Eigen::Affine3d>* poses,
    TransformResolveStatus* status) const {
  if (poses == nullptr) {
    if (status != nullptr) {
      *status = TransformResolveStatus::kQueryFailed;
    }
    return false;
  }
  poses->clear();
  poses->reserve(timestamps_sec.size());
  for (const double timestamp_sec : timestamps_sec) {
    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    TransformResolveStatus local_status = TransformResolveStatus::kOk;
    if (!QueryCachedStrict(timestamp_sec, &pose, &local_status)) {
      if (status != nullptr) {
        *status = local_status;
      }
      poses->clear();
      return false;
    }
    poses->push_back(pose);
  }
  if (status != nullptr) {
    *status = TransformResolveStatus::kOk;
  }
  return true;
}

}  // namespace transform
}  // namespace apollo
