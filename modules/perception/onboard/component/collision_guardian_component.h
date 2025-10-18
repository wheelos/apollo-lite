#pragma once

#include <memory>
#include <string>

#include <Eigen/Geometry>

#include "modules/common_msgs/perception_msgs/collision_warning.pb.h"
#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"
#include "modules/perception/onboard/proto/collision_guardian_component.pb.h"

#include "cyber/component/component.h"
#include "modules/perception/onboard/transform_wrapper/transform_wrapper.h"

namespace apollo {
namespace perception {
namespace lidar {

using apollo::drivers::PointCloud;

class CollisionGuardianComponent : public cyber::Component<PointCloud> {
 public:
  using TransformWrapper = apollo::perception::onboard::TransformWrapper;

  CollisionGuardianComponent() = default;
  ~CollisionGuardianComponent() = default;

  bool Init() override;
  bool Proc(const std::shared_ptr<PointCloud>& message) override;

 private:
  // The core logic function that performs filtering and point counting on a
  // single frame.
  bool CheckCollisionRisk(const std::shared_ptr<PointCloud>& message,
                          Eigen::Affine3d& sensor2vehicle_transform);

  // Member variables loaded from the config proto.
  std::string vehicle_frame_id_;

  // Ego Vehicle Dimensions
  float ego_box_forward_ = 0.0f;
  float ego_box_backward_ = 0.0f;
  float ego_box_side_ = 0.0f;

  float roi_forward_distance_ = 0.0f;
  float roi_backward_distance_ = 0.0f;
  float roi_side_distance_ = 0.0f;
  float height_min_threshold_ = 0.0f;
  float height_max_threshold_ = 0.0f;
  uint32_t min_points_in_roi_to_trigger_ = 0;
  uint32_t min_consecutive_frames_to_trigger_ = 0;

  // Temporal consistency counter.
  uint32_t consecutive_hit_counter_ = 0;

  // Cyber writer for publishing the result.
  std::shared_ptr<cyber::Writer<apollo::perception::CollisionWarning>> writer_ =
      nullptr;

  // TF listener for getting the transform from sensor to vehicle frame.
  std::unique_ptr<TransformWrapper> transform_wrapper_ = nullptr;
};

CYBER_REGISTER_COMPONENT(CollisionGuardianComponent);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
