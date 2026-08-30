#pragma once

#include <string>

#include "wheelos_msgs/sensor_msgs/pointcloud.pb.h"

#include "modules/lidar_semantic_segmentation/types/semantic_types.h"

namespace apollo {
namespace lidar_semantic_segmentation {

class RangeImageProjector {
 public:
  bool Init(const RangeImageProjectionOptions& options, std::string* error);

  bool Project(const apollo::drivers::PointCloud& cloud, RangeImage* image,
               std::string* error) const;

  const RangeImageProjectionOptions& options() const { return options_; }

 private:
  RangeImageProjectionOptions options_;
  bool initialized_ = false;
};

}  // namespace lidar_semantic_segmentation
}  // namespace apollo
