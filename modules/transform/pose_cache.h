#pragma once

#include <mutex>
#include <string>
#include <vector>

#include "Eigen/Geometry"

#include "modules/transform/buffer_interface.h"

namespace apollo {
namespace transform {

enum class PoseCacheStatus {
  kOk = 0,
  kEmpty = 1,
  kTooOld = 2,
  kTooEarly = 3,
  kQueryFailed = 4,
};

struct PoseCacheOptions {
  double cache_duration_sec = 1.0;
  double max_extrapolation_sec = 0.15;
  float query_timeout_sec = 0.01f;
};

class PoseCache {
 public:
  explicit PoseCache(double cache_duration_sec = 1.0);

  void SetCacheDuration(double cache_duration_sec);

  bool Insert(double timestamp_sec, const Eigen::Affine3d& pose);

  bool Query(double timestamp_sec, double max_extrapolation_sec,
             Eigen::Affine3d* pose, PoseCacheStatus* status = nullptr) const;

  size_t Size() const;

 private:
  struct PoseSample {
    double timestamp_sec = 0.0;
    double tx = 0.0;
    double ty = 0.0;
    double tz = 0.0;
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = 1.0;
  };

  static PoseSample FromAffine(double timestamp_sec,
                               const Eigen::Affine3d& pose);
  static Eigen::Affine3d ToAffine(const PoseSample& sample);

  mutable std::mutex mutex_;
  std::vector<PoseSample> samples_;
  double cache_duration_sec_ = 1.0;
};

class TransformFrameCache {
 public:
  TransformFrameCache() = default;
  TransformFrameCache(BufferInterface* buffer, const std::string& target_frame,
                      const std::string& source_frame,
                      const PoseCacheOptions& options);

  void Configure(BufferInterface* buffer, const std::string& target_frame,
                 const std::string& source_frame,
                 const PoseCacheOptions& options);

  bool Prefetch(double timestamp_sec);

  bool PrefetchBatch(const std::vector<double>& timestamps_sec);

  bool StorePose(double timestamp_sec, const Eigen::Affine3d& pose);

  bool QueryCached(double timestamp_sec, Eigen::Affine3d* pose,
                   PoseCacheStatus* status = nullptr) const;

  bool QueryCachedStrict(double timestamp_sec, Eigen::Affine3d* pose,
                         PoseCacheStatus* status = nullptr) const;

  bool QueryCachedBatch(const std::vector<double>& timestamps_sec,
                        std::vector<Eigen::Affine3d>* poses,
                        PoseCacheStatus* status = nullptr) const;

  bool QueryCachedBatchStrict(const std::vector<double>& timestamps_sec,
                              std::vector<Eigen::Affine3d>* poses,
                              PoseCacheStatus* status = nullptr) const;

 private:
  bool QueryTransform(double timestamp_sec, Eigen::Affine3d* pose) const;

  BufferInterface* buffer_ = nullptr;
  std::string target_frame_;
  std::string source_frame_;
  PoseCacheOptions options_;
  PoseCache pose_cache_;
};

}  // namespace transform
}  // namespace apollo
