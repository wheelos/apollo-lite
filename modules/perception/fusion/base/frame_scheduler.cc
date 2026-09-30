#include "modules/perception/fusion/base/frame_scheduler.h"

#include <cmath>
#include <memory>
#include <set>

#include "cyber/common/log.h"
#include "modules/perception/fusion/base/observation_validation.h"

namespace apollo {
namespace perception {
namespace fusion {

bool FrameScheduler::Add(const base::FrameConstPtr& frame) {
  if (!frame || !std::isfinite(frame->timestamp) || frame->timestamp < 0 ||
      frame->sensor_info.name.empty() ||
      !frame->sensor2world_pose.matrix().allFinite()) {
    AERROR << "Invalid fusion observation header.";
    return false;
  }
  const Eigen::Matrix3d rotation = frame->sensor2world_pose.linear();
  if (!(rotation.transpose() * rotation).isApprox(
          Eigen::Matrix3d::Identity(), 1e-6) ||
      std::fabs(rotation.determinant() - 1.0) > 1e-6) {
    AERROR << "Fusion observation pose must be a rigid, invertible transform.";
    return false;
  }
  if (frame->timestamp <= watermark_) {
    AWARN << "Reject late fusion observation: " << frame->sensor_info.name
          << " timestamp=" << frame->timestamp << " watermark=" << watermark_;
    return false;
  }
  const auto key = std::make_pair(frame->timestamp, frame->sensor_info.name);
  if (pending_.count(key) != 0) {
    AWARN << "Reject duplicate fusion observation: " << key.second
          << " timestamp=" << key.first;
    return false;
  }
  if (pending_.size() >= capacity_) {
    AERROR << "Fusion observation queue exhausted: capacity=" << capacity_;
    return false;
  }
  auto snapshot = std::make_shared<base::Frame>(*frame);
  std::set<int> local_ids;
  for (auto& object : snapshot->objects) {
    if (!object || !ValidPlanarObservation(*object) ||
        (object->track_id >= 0 && !local_ids.insert(object->track_id).second)) {
      AERROR << "Invalid fusion observation or duplicate local track ID.";
      return false;
    }
    object = std::make_shared<base::Object>(*object);
  }
  pending_.emplace(key, snapshot);
  return true;
}

bool FrameScheduler::Drain(double timestamp,
                           std::vector<base::FrameConstPtr>* frames) {
  if (!frames || !std::isfinite(timestamp) || timestamp <= watermark_) {
    AERROR << "Fusion watermark must advance monotonically: " << timestamp;
    return false;
  }
  frames->clear();
  auto it = pending_.begin();
  while (it != pending_.end() && it->first.first <= timestamp) {
    frames->push_back(it->second);
    sensor_timestamps_[it->first.second] = it->first.first;
    it = pending_.erase(it);
  }
  watermark_ = timestamp;
  return true;
}

bool FrameScheduler::HasFreshSensor(const std::string& sensor, double timestamp,
                                    double timeout) const {
  const auto it = sensor_timestamps_.find(sensor);
  return it != sensor_timestamps_.end() && timestamp >= it->second &&
         timestamp - it->second <= timeout;
}

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
