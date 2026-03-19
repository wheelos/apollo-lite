#include "modules/perception/onboard/state_estimator/state_estimator.h"

#include <algorithm>

#include "cyber/common/log.h"
#include "cyber/time/clock.h"

namespace apollo {
namespace perception {
namespace onboard {

std::deque<apollo::localization::LocalizationEstimate>
    StateEstimator::pose_buffer_;
std::shared_mutex StateEstimator::buffer_mutex_;

void StateEstimator::Init(const std::string& base_link_frame_id) {
  base_link_frame_id_ = base_link_frame_id;
  inited_ = true;
}

void StateEstimator::UpdateLocalization(
    const apollo::localization::LocalizationEstimate& localization) {
  std::unique_lock<std::shared_mutex> lock(buffer_mutex_);
  pose_buffer_.push_back(localization);
  while (pose_buffer_.size() > max_buffer_size_) {
    pose_buffer_.pop_front();
  }
}

StateEstimator::Status StateEstimator::GetSensor2World(
    const std::string& sensor_id, double timestamp, Eigen::Affine3d* out_pose) {
  Status status;
  if (!inited_ || out_pose == nullptr) {
    AERROR << "StateEstimator not initialized or null output";
    return status;
  }

  status.latency = apollo::cyber::Time::Now().ToSecond() - timestamp;

  // 1. Get static extrinsics
  Eigen::Affine3d sensor2base;
  if (!LookupStaticExtrinsic(sensor_id, &sensor2base)) {
    AERROR << "Failed to find static extrinsics for " << sensor_id;
    return status;
  }

  // 2. Get dynamic Ego pose
  Eigen::Affine3d base2world;
  if (!GetEgoPose(timestamp, &base2world, &status)) {
    // Fallback to TF2
    if (!LookupPoseFromTF(timestamp, &base2world)) {
      AWARN << "Failed to resolve ego pose from buffer and TF for time: "
            << timestamp;
      return status;
    }
  }

  *out_pose = base2world * sensor2base;
  status.success = true;
  return status;
}

bool StateEstimator::LookupStaticExtrinsic(const std::string& sensor_id,
                                           Eigen::Affine3d* extrinsic) {
  if (extrinsics_cache_.find(sensor_id) != extrinsics_cache_.end()) {
    *extrinsic = extrinsics_cache_[sensor_id];
    return true;
  }

  try {
    auto stamped_transform = tf2_buffer_->lookupTransform(
        base_link_frame_id_, sensor_id, apollo::cyber::Time(0));
    Eigen::Translation3d translation(
        stamped_transform.transform().translation().x(),
        stamped_transform.transform().translation().y(),
        stamped_transform.transform().translation().z());
    Eigen::Quaterniond rotation(stamped_transform.transform().rotation().qw(),
                                stamped_transform.transform().rotation().qx(),
                                stamped_transform.transform().rotation().qy(),
                                stamped_transform.transform().rotation().qz());

    *extrinsic = translation * rotation;
    extrinsics_cache_[sensor_id] = *extrinsic;
    AINFO << "Successfully cached static extrinsic for sensor: " << sensor_id;
    return true;
  } catch (const std::exception& e) {
    AERROR << "TF Extrinsics lookup failed for " << sensor_id << ": "
           << e.what();
    return false;
  }
}

bool StateEstimator::GetEgoPose(double timestamp, Eigen::Affine3d* base2world,
                                Status* status) {
  std::shared_lock<std::shared_mutex> lock(buffer_mutex_);
  if (pose_buffer_.empty()) return false;

  const auto& latest_msg = pose_buffer_.back();
  const auto& earliest_msg = pose_buffer_.front();

  // --- Scenario A: Extrapolation (need prediction) ---
  if (timestamp > latest_msg.measurement_time()) {
    status->is_extrapolated = true;
    double dt = timestamp - latest_msg.measurement_time();

    // Physics bounds roughly 0.5s max prediction
    if (dt > 0.5) return false;

    Eigen::Vector3d p_last(latest_msg.pose().position().x(),
                           latest_msg.pose().position().y(),
                           latest_msg.pose().position().z());
    Eigen::Quaterniond q_last(latest_msg.pose().orientation().qw(),
                              latest_msg.pose().orientation().qx(),
                              latest_msg.pose().orientation().qy(),
                              latest_msg.pose().orientation().qz());

    // Position Prediction: P_new = P_last + V_world * dt
    Eigen::Vector3d v_world(latest_msg.pose().linear_velocity().x(),
                            latest_msg.pose().linear_velocity().y(),
                            latest_msg.pose().linear_velocity().z());
    Eigen::Vector3d p_pred = p_last + v_world * dt;

    // Rotation Prediction: Q_pred = Q_delta * Q_last
    Eigen::Vector3d omega_world(latest_msg.pose().angular_velocity().x(),
                                latest_msg.pose().angular_velocity().y(),
                                latest_msg.pose().angular_velocity().z());
    double angle = omega_world.norm() * dt;
    if (angle > 1e-6) {
      Eigen::Vector3d axis = omega_world.normalized();
      Eigen::Quaterniond q_delta(Eigen::AngleAxisd(angle, axis));
      q_last = q_delta * q_last;
    }

    *base2world = Eigen::Translation3d(p_pred) * q_last;
    return true;
  }

  // --- Scenario B: Interpolation (Lerp/Slerp) ---
  if (timestamp >= earliest_msg.measurement_time()) {
    auto it =
        std::lower_bound(pose_buffer_.begin(), pose_buffer_.end(), timestamp,
                         [](const apollo::localization::LocalizationEstimate& a,
                            double t) { return a.measurement_time() < t; });

    if (it != pose_buffer_.begin() && it != pose_buffer_.end()) {
      const auto& m1 = *(it - 1);
      const auto& m2 = *it;
      double t1 = m1.measurement_time();
      double t2 = m2.measurement_time();

      double dt1 = timestamp - t1;
      double dt2 = t2 - t1;
      double ratio = (dt2 > 1e-6) ? (dt1 / dt2) : 0.0;

      // Position Lerp
      Eigen::Vector3d p1(m1.pose().position().x(), m1.pose().position().y(),
                         m1.pose().position().z());
      Eigen::Vector3d p2(m2.pose().position().x(), m2.pose().position().y(),
                         m2.pose().position().z());
      Eigen::Vector3d p = p1 * (1.0 - ratio) + p2 * ratio;

      // Rotation Slerp
      Eigen::Quaterniond q1(
          m1.pose().orientation().qw(), m1.pose().orientation().qx(),
          m1.pose().orientation().qy(), m1.pose().orientation().qz());
      Eigen::Quaterniond q2(
          m2.pose().orientation().qw(), m2.pose().orientation().qx(),
          m2.pose().orientation().qy(), m2.pose().orientation().qz());

      *base2world = Eigen::Translation3d(p) * q1.slerp(ratio, q2);
      return true;
    }
  }

  // --- Scenario C: Too old ---
  return false;
}

bool StateEstimator::LookupPoseFromTF(double timestamp,
                                      Eigen::Affine3d* base2world) {
  try {
    auto stamped_transform = tf2_buffer_->lookupTransform(
        world_frame_id_, base_link_frame_id_, apollo::cyber::Time(timestamp));
    Eigen::Translation3d translation(
        stamped_transform.transform().translation().x(),
        stamped_transform.transform().translation().y(),
        stamped_transform.transform().translation().z());
    Eigen::Quaterniond rotation(stamped_transform.transform().rotation().qw(),
                                stamped_transform.transform().rotation().qx(),
                                stamped_transform.transform().rotation().qy(),
                                stamped_transform.transform().rotation().qz());
    *base2world = translation * rotation;
    return true;
  } catch (const std::exception& e) {
    return false;
  }
}

}  // namespace onboard
}  // namespace perception
}  // namespace apollo
