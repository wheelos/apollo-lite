#pragma once

#include <deque>
#include <memory>
#include <shared_mutex>
#include <string>
#include <unordered_map>

#include "Eigen/Core"
#include "Eigen/Dense"

#include "modules/localization/proto/localization.pb.h"

#include "modules/transform/buffer.h"

namespace apollo {
namespace perception {
namespace onboard {

class StateEstimator {
 public:
  struct Status {
    bool success = false;
    double latency = 0.0;
    bool is_extrapolated = false;
  };

  StateEstimator() = default;
  ~StateEstimator() = default;

  // Modern Recommended API: Initialization with base link frame check
  void Init(const std::string& base_link_frame_id = "novatel");

  // Core API: Get precise sensor to world transform
  Status GetSensor2World(const std::string& sensor_id, double timestamp,
                         Eigen::Affine3d* out_pose);

  // High-frequency Localization Entry Point
  static void UpdateLocalization(
      const apollo::localization::LocalizationEstimate& localization);

 private:
  bool LookupStaticExtrinsic(const std::string& sensor_id,
                             Eigen::Affine3d* extrinsic);
  bool GetEgoPose(double timestamp, Eigen::Affine3d* base2world,
                  Status* status);
  bool LookupPoseFromTF(double timestamp, Eigen::Affine3d* base2world);

 private:
  bool inited_ = false;
  std::string base_link_frame_id_ = "novatel";
  std::string world_frame_id_ = "world";

  apollo::transform::Buffer* tf2_buffer_ =
      apollo::transform::Buffer::Instance();

  // Fast mapped caching for static extrinsics querying
  std::unordered_map<std::string, Eigen::Affine3d> extrinsics_cache_;

  // Thread-safe Kinematic Buffer
  static std::deque<apollo::localization::LocalizationEstimate> pose_buffer_;
  static std::shared_mutex buffer_mutex_;
  static const size_t max_buffer_size_ = 200;
};

}  // namespace onboard
}  // namespace perception
}  // namespace apollo
