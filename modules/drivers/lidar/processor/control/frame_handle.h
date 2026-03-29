#pragma once

#include <memory>
#include <string>

#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"

namespace apollo {
namespace drivers {
namespace lidar {

struct FrameHandle {
  std::string sensor_id;
  std::shared_ptr<const apollo::drivers::PointCloud> point_cloud;
  bool is_primary = false;
};

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
