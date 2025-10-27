#include "modules/perception/onboard/component/collision_guardian_component.h"

#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/adapters/adapter_gflags.h"

namespace apollo {
namespace perception {
namespace lidar {

using apollo::cyber::Clock;

bool CollisionGuardianComponent::Init() {
  CollisionGuardianComponentConfig comp_config;
  if (!GetProtoConfig(&comp_config)) {
    AERROR << "Failed to get CollisionGuardianComponentConfig.";
    return false;
  }
  AINFO << "Collision Guardian Component Configs: "
        << comp_config.DebugString();

  writer_ = node_->CreateWriter<apollo::perception::CollisionWarning>(
      FLAGS_collision_warning_topic);

  vehicle_frame_id_ = comp_config.vehicle_frame_id();

  // Load Ego Vehicle Dimensions
  ego_box_forward_ = static_cast<float>(comp_config.ego_box_forward());
  ego_box_backward_ = -static_cast<float>(comp_config.ego_box_backward());
  ego_box_side_ = static_cast<float>(comp_config.ego_box_side());

  // Load ROI Dimensions
  roi_forward_distance_ =
      static_cast<float>(comp_config.roi_forward_distance());
  roi_backward_distance_ =
      -static_cast<float>(comp_config.roi_backward_distance());
  roi_side_distance_ = static_cast<float>(comp_config.roi_side_distance());

  // Load Filter and Trigger Logic Params
  height_min_threshold_ =
      static_cast<float>(comp_config.height_min_threshold());
  height_max_threshold_ =
      static_cast<float>(comp_config.height_max_threshold());
  min_points_in_roi_to_trigger_ = comp_config.min_points_in_roi_to_trigger();
  min_consecutive_frames_to_trigger_ =
      comp_config.min_consecutive_frames_to_trigger();

  // Initialize TransformWrapper
  // According to your provided class, we can just instantiate it.
  // The `GetTrans` method will handle the lookup.
  transform_wrapper_ = std::make_unique<TransformWrapper>();

  AINFO << "CollisionGuardianComponent Init SUCCESS";
  return true;
}

bool CollisionGuardianComponent::Proc(
    const std::shared_ptr<PointCloud>& message) {
  const double start_time = Clock::NowInSeconds();

  // --- Get sensor to vehicle body transform using TransformWrapper ---
  Eigen::Affine3d sensor2vehicle_transform;
  // Using the generic GetTrans method as it's the most flexible.
  // This transform will take points from the sensor frame to the vehicle frame.
  if (!transform_wrapper_->GetTrans(
          message->header().timestamp_sec(), &sensor2vehicle_transform,
          vehicle_frame_id_, message->header().frame_id())) {
    AERROR << "Failed to get transform from " << message->header().frame_id()
           << " to " << vehicle_frame_id_;
    return false;
  }

  bool risk_in_current_frame =
      CheckCollisionRisk(message, sensor2vehicle_transform);

  if (risk_in_current_frame) {
    consecutive_hit_counter_++;
  } else {
    consecutive_hit_counter_ = 0;
  }

  auto out_message = std::make_shared<apollo::perception::CollisionWarning>();
  out_message->mutable_header()->set_timestamp_sec(start_time);

  if (consecutive_hit_counter_ >= min_consecutive_frames_to_trigger_) {
    out_message->set_is_collision(true);
    AWARN << "[CollisionGuardian] PERSISTENT COLLISION RISK DETECTED! "
             "Publishing TRUE.";
  } else {
    out_message->set_is_collision(false);
  }

  writer_->Write(out_message);

  return true;
}

bool CollisionGuardianComponent::CheckCollisionRisk(
    const std::shared_ptr<PointCloud>& message,
    Eigen::Affine3d& sensor2vehicle_transform) {
  uint32_t points_in_roi_count = 0;

  for (const auto& pt : message->point()) {
    // Layer 1: Filter invalid points
    if (std::isnan(pt.x()) || std::isnan(pt.y()) || std::isnan(pt.z())) {
      continue;
    }

    // Transform point to the vehicle coordinate frame for consistent filtering
    Eigen::Vector3d point_in_vehicle_frame =
        sensor2vehicle_transform * Eigen::Vector3d(pt.x(), pt.y(), pt.z());

    // Remove any point that falls within the precise bounding box of our own
    // vehicle.
    if (point_in_vehicle_frame.y() < ego_box_forward_ &&
        point_in_vehicle_frame.y() > ego_box_backward_ &&
        std::abs(point_in_vehicle_frame.x()) < ego_box_side_) {
      continue;  // This point is on our car, ignore it.
    }

    // Layer 3: Height Filter
    if (point_in_vehicle_frame.z() < height_min_threshold_ ||
        point_in_vehicle_frame.z() > height_max_threshold_) {
      continue;
    }

    // Layer 4: ROI (Protective Field) Filter
    if (point_in_vehicle_frame.y() > roi_forward_distance_ ||
        point_in_vehicle_frame.y() < roi_backward_distance_ ||
        std::abs(point_in_vehicle_frame.x()) > roi_side_distance_) {
      continue;
    }

    points_in_roi_count++;

    if (points_in_roi_count >= min_points_in_roi_to_trigger_) {
      return true;
    }
  }

  return false;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
