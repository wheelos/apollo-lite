// Copyright 2026 WheelOS. All Rights Reserved.
#include "modules/dreamview/backend/point_cloud/point_cloud_conversion.h"

#include <cstddef>

namespace apollo {
namespace dreamview {

pcl::PointCloud<pcl::PointXYZ>::Ptr ConvertPCLPointCloud(
    const drivers::PointCloud& cloud,
    const Eigen::Affine3d& target_from_source) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr output(
      new pcl::PointCloud<pcl::PointXYZ>);
  output->width = cloud.width();
  output->height = cloud.height();
  output->is_dense = false;
  if (static_cast<size_t>(output->width) * output->height !=
      static_cast<size_t>(cloud.point_size())) {
    output->width = cloud.point_size();
    output->height = 1;
  }
  output->points.resize(cloud.point_size());
  for (size_t i = 0; i < output->points.size(); ++i) {
    const auto& point = cloud.point(static_cast<int>(i));
    const Eigen::Vector3d transformed =
        target_from_source * Eigen::Vector3d(point.x(), point.y(), point.z());
    output->points[i].x = transformed.x();
    output->points[i].y = transformed.y();
    output->points[i].z = transformed.z();
  }
  return output;
}

}  // namespace dreamview
}  // namespace apollo
