#pragma once

#include <memory>
#include <string>
#include <vector>

#include "Eigen/Geometry"

#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"

namespace apollo {
namespace drivers {
namespace lidar {

struct BufferedFrame {
  std::shared_ptr<const ::apollo::drivers::PointCloud> point_cloud;
  std::vector<double> motion_sample_times;
  std::vector<Eigen::Affine3d> motion_poses;
  bool pose_prefetch_ok = false;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct FrameHandle {
  std::string sensor_id;
  std::shared_ptr<const ::apollo::drivers::PointCloud> point_cloud;
  std::shared_ptr<const BufferedFrame> buffered_frame;
  bool is_primary = false;
  double clock_offset_residual_ms = 0.0;
  double overlap_quality_weight = 1.0;
};

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
