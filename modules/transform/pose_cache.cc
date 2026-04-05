#include "modules/transform/pose_cache.h"

#include <algorithm>
#include <cmath>
#include <iterator>

#include "cyber/common/log.h"

namespace apollo {
namespace transform {

namespace {

constexpr double kTimestampEpsilonSec = 1e-9;

double ClampRatio(double value) { return std::max(0.0, std::min(1.0, value)); }

}  // namespace

PoseCache::PoseCache(double cache_duration_sec)
    : cache_duration_sec_(cache_duration_sec) {}

void PoseCache::SetCacheDuration(double cache_duration_sec) {
  std::lock_guard<std::mutex> lock(mutex_);
  cache_duration_sec_ = cache_duration_sec;
  while (!samples_.empty() &&
         (samples_.back().timestamp_sec - samples_.front().timestamp_sec) >
             cache_duration_sec_) {
    samples_.erase(samples_.begin());
  }
}

bool PoseCache::Insert(double timestamp_sec, const Eigen::Affine3d& pose) {
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
             cache_duration_sec_) {
    samples_.erase(samples_.begin());
  }
  return !samples_.empty();
}

bool PoseCache::Query(double timestamp_sec, double max_extrapolation_sec,
                      Eigen::Affine3d* pose, PoseCacheStatus* status) const {
  if (pose == nullptr) {
    if (status != nullptr) {
      *status = PoseCacheStatus::kQueryFailed;
    }
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (samples_.empty()) {
    if (status != nullptr) {
      *status = PoseCacheStatus::kEmpty;
    }
    return false;
  }

  if (timestamp_sec + kTimestampEpsilonSec < samples_.front().timestamp_sec) {
    if (status != nullptr) {
      *status = PoseCacheStatus::kTooEarly;
    }
    return false;
  }

  if (timestamp_sec > samples_.back().timestamp_sec + kTimestampEpsilonSec) {
    if ((timestamp_sec - samples_.back().timestamp_sec) >
        max_extrapolation_sec) {
      if (status != nullptr) {
        *status = PoseCacheStatus::kTooOld;
      }
      return false;
    }
    *pose = ToAffine(samples_.back());
    if (status != nullptr) {
      *status = PoseCacheStatus::kOk;
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
      *status = PoseCacheStatus::kOk;
    }
    return true;
  }
  if (it == samples_.end()) {
    *pose = ToAffine(samples_.back());
    if (status != nullptr) {
      *status = PoseCacheStatus::kOk;
    }
    return true;
  }
  if (std::fabs(it->timestamp_sec - timestamp_sec) <= kTimestampEpsilonSec) {
    *pose = ToAffine(*it);
    if (status != nullptr) {
      *status = PoseCacheStatus::kOk;
    }
    return true;
  }

  const auto left = std::prev(it);
  const double left_time = left->timestamp_sec;
  const double right_time = it->timestamp_sec;
  if (std::fabs(right_time - left_time) <= kTimestampEpsilonSec) {
    *pose = ToAffine(*it);
    if (status != nullptr) {
      *status = PoseCacheStatus::kOk;
    }
    return true;
  }

  const double ratio =
      ClampRatio((timestamp_sec - left_time) / (right_time - left_time));
  Eigen::Quaterniond left_rotation(left->qw, left->qx, left->qy, left->qz);
  Eigen::Quaterniond right_rotation(it->qw, it->qx, it->qy, it->qz);
  if (left_rotation.dot(right_rotation) < 0.0) {
    right_rotation.coeffs() *= -1.0;
  }

  const Eigen::Vector3d translation(left->tx + ratio * (it->tx - left->tx),
                                    left->ty + ratio * (it->ty - left->ty),
                                    left->tz + ratio * (it->tz - left->tz));
  *pose = Eigen::Translation3d(translation) *
          left_rotation.slerp(ratio, right_rotation);
  if (status != nullptr) {
    *status = PoseCacheStatus::kOk;
  }
  return true;
}

size_t PoseCache::Size() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return samples_.size();
}

PoseCache::PoseSample PoseCache::FromAffine(double timestamp_sec,
                                            const Eigen::Affine3d& pose) {
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

Eigen::Affine3d PoseCache::ToAffine(const PoseSample& sample) {
  return Eigen::Translation3d(sample.tx, sample.ty, sample.tz) *
         Eigen::Quaterniond(sample.qw, sample.qx, sample.qy, sample.qz);
}

TransformFrameCache::TransformFrameCache(BufferInterface* buffer,
                                         const std::string& target_frame,
                                         const std::string& source_frame,
                                         const PoseCacheOptions& options)
    : options_(options), pose_cache_(options.cache_duration_sec) {
  Configure(buffer, target_frame, source_frame, options);
}

void TransformFrameCache::Configure(BufferInterface* buffer,
                                    const std::string& target_frame,
                                    const std::string& source_frame,
                                    const PoseCacheOptions& options) {
  buffer_ = buffer;
  target_frame_ = target_frame;
  source_frame_ = source_frame;
  options_ = options;
  pose_cache_.SetCacheDuration(options.cache_duration_sec);
}

bool TransformFrameCache::Prefetch(double timestamp_sec) {
  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  if (!QueryTransform(timestamp_sec, &pose)) {
    return false;
  }
  return StorePose(timestamp_sec, pose);
}

bool TransformFrameCache::PrefetchBatch(
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

bool TransformFrameCache::StorePose(double timestamp_sec,
                                    const Eigen::Affine3d& pose) {
  return pose_cache_.Insert(timestamp_sec, pose);
}

bool TransformFrameCache::QueryCached(double timestamp_sec,
                                      Eigen::Affine3d* pose,
                                      PoseCacheStatus* status) const {
  return pose_cache_.Query(timestamp_sec, options_.max_extrapolation_sec, pose,
                           status);
}

bool TransformFrameCache::QueryCachedStrict(double timestamp_sec,
                                            Eigen::Affine3d* pose,
                                            PoseCacheStatus* status) const {
  return pose_cache_.Query(timestamp_sec, 0.0, pose, status);
}

bool TransformFrameCache::QueryCachedBatch(
    const std::vector<double>& timestamps_sec,
    std::vector<Eigen::Affine3d>* poses, PoseCacheStatus* status) const {
  if (poses == nullptr) {
    if (status != nullptr) {
      *status = PoseCacheStatus::kQueryFailed;
    }
    return false;
  }
  poses->clear();
  poses->reserve(timestamps_sec.size());
  for (const double timestamp_sec : timestamps_sec) {
    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    PoseCacheStatus local_status = PoseCacheStatus::kOk;
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
    *status = PoseCacheStatus::kOk;
  }
  return true;
}

bool TransformFrameCache::QueryCachedBatchStrict(
    const std::vector<double>& timestamps_sec,
    std::vector<Eigen::Affine3d>* poses, PoseCacheStatus* status) const {
  if (poses == nullptr) {
    if (status != nullptr) {
      *status = PoseCacheStatus::kQueryFailed;
    }
    return false;
  }
  poses->clear();
  poses->reserve(timestamps_sec.size());
  for (const double timestamp_sec : timestamps_sec) {
    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    PoseCacheStatus local_status = PoseCacheStatus::kOk;
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
    *status = PoseCacheStatus::kOk;
  }
  return true;
}

bool TransformFrameCache::QueryTransform(double timestamp_sec,
                                         Eigen::Affine3d* pose) const {
  if (buffer_ == nullptr || pose == nullptr || target_frame_.empty() ||
      source_frame_.empty()) {
    return false;
  }

  std::string err;
  if (!buffer_->canTransform(target_frame_, source_frame_,
                             cyber::Time(timestamp_sec),
                             options_.query_timeout_sec, &err)) {
    AWARN << "TransformFrameCache failed to query transform from "
          << source_frame_ << " to " << target_frame_ << " at " << timestamp_sec
          << ": " << err;
    return false;
  }

  try {
    const auto stamped = buffer_->lookupTransform(target_frame_, source_frame_,
                                                  cyber::Time(timestamp_sec),
                                                  options_.query_timeout_sec);
    *pose = Eigen::Translation3d(stamped.transform().translation().x(),
                                 stamped.transform().translation().y(),
                                 stamped.transform().translation().z()) *
            Eigen::Quaterniond(stamped.transform().rotation().qw(),
                               stamped.transform().rotation().qx(),
                               stamped.transform().rotation().qy(),
                               stamped.transform().rotation().qz());
    return true;
  } catch (const std::exception& ex) {
    AWARN << "TransformFrameCache lookupTransform threw: " << ex.what();
    return false;
  }
}

}  // namespace transform
}  // namespace apollo
