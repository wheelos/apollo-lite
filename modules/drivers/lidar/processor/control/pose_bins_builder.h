#pragma once

#include <cstddef>
#include <vector>

#include "Eigen/Eigen"

#include "modules/drivers/lidar/processor/control/frame_handle.h"
#include "modules/drivers/lidar/processor/policy/lidar_policy_interface.h"

namespace apollo {
namespace drivers {
namespace lidar {

class PoseBinsBuilder {
 public:
  bool Build(const std::vector<FrameHandle>& frame_handles,
             LidarDeskewPolicy* deskew_policy,
             std::vector<SensorFrameContext>* contexts,
             std::vector<std::vector<double>>* motion_sample_times,
             std::vector<std::vector<Eigen::Affine3d>>* motion_poses,
             size_t* required_points) const;
};

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
