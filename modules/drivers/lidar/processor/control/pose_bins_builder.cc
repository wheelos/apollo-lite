#include "modules/drivers/lidar/processor/control/pose_bins_builder.h"

#include "cyber/cyber.h"

namespace apollo {
namespace drivers {
namespace lidar {

bool PoseBinsBuilder::Build(
    const std::vector<FrameHandle>& frame_handles,
    LidarDeskewPolicy* deskew_policy, std::vector<SensorFrameContext>* contexts,
    std::vector<std::vector<double>>* motion_sample_times,
    std::vector<std::vector<Eigen::Affine3d>>* motion_poses,
    size_t* required_points) const {
  if (deskew_policy == nullptr || contexts == nullptr ||
      motion_sample_times == nullptr || motion_poses == nullptr ||
      required_points == nullptr) {
    return false;
  }

  contexts->clear();
  motion_sample_times->clear();
  motion_poses->clear();
  *required_points = 0;

  contexts->reserve(frame_handles.size());
  motion_sample_times->reserve(frame_handles.size());
  motion_poses->reserve(frame_handles.size());

  for (const auto& handle : frame_handles) {
    if (handle.point_cloud == nullptr) {
      if (handle.is_primary) {
        AERROR << "Main sensor frame is null: " << handle.sensor_id;
        return false;
      }
      AWARN << "Skip auxiliary sensor due to null frame: " << handle.sensor_id;
      continue;
    }

    SensorFrameContext context;
    context.sensor_id = handle.sensor_id;
    context.point_cloud = handle.point_cloud;
    context.is_primary = handle.is_primary;

    std::vector<double> sample_times;
    std::vector<Eigen::Affine3d> poses;
    if (!deskew_policy->ComputeMotionCompensationPoses(context, &sample_times,
                                                       &poses) ||
        sample_times.empty() || poses.empty() ||
        sample_times.size() != poses.size()) {
      if (handle.is_primary) {
        AERROR << "Failed to compute motion compensation poses for main sensor "
               << handle.sensor_id;
        return false;
      }
      AWARN << "Skip auxiliary sensor due to invalid motion compensation data: "
            << handle.sensor_id;
      continue;
    }

    contexts->push_back(std::move(context));
    motion_sample_times->push_back(std::move(sample_times));
    motion_poses->push_back(std::move(poses));
    *required_points += static_cast<size_t>(handle.point_cloud->point_size());
  }

  return !contexts->empty();
}

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
