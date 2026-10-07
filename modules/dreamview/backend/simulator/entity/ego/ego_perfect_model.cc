#include "modules/dreamview/backend/simulator/entity/ego/ego_perfect_model.h"

#include "modules/common/math/linear_interpolation.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/util/message_util.h"

namespace apollo {
namespace dreamview {

EgoPerfectModel::EgoPerfectModel(std::shared_ptr<cyber::Node> node,
                                 MapService* map_service)
    : EgoModelBase(node, map_service) {
  planning_reader_ = node_->CreateReader<apollo::planning::ADCTrajectory>(
      "/apollo/planning",
      [this](
          const std::shared_ptr<apollo::planning::ADCTrajectory>& trajectory) {
        this->OnPlanning(trajectory);
      });
}

void EgoPerfectModel::SetPose(double x, double y, double heading,
                              double velocity) {
  std::lock_guard<std::mutex> lock(mutex_);
  current_x_ = x;
  current_y_ = y;
  current_heading_ = heading;
  current_velocity_ = velocity;
  current_acceleration_ = 0.0;
  current_trajectory_ = nullptr;  // Clear previous trajectory
}

void EgoPerfectModel::RequestRouting(
    const std::vector<apollo::common::math::Vec2d>& waypoints) {
  auto routing_request = std::make_shared<apollo::routing::RoutingRequest>();
  apollo::common::util::FillHeader("EgoPerfectModel", routing_request.get());

  for (const auto& pt : waypoints) {
    auto* waypoint = routing_request->add_waypoint();
    waypoint->mutable_pose()->set_x(pt.x());
    waypoint->mutable_pose()->set_y(pt.y());
  }
  routing_writer_->Write(routing_request);
}

void EgoPerfectModel::Reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  current_trajectory_ = nullptr;
  current_velocity_ = 0.0;
  current_acceleration_ = 0.0;
}

void EgoPerfectModel::OnPlanning(
    const std::shared_ptr<apollo::planning::ADCTrajectory>& trajectory) {
  std::lock_guard<std::mutex> lock(mutex_);
  current_trajectory_ = trajectory;
}

bool EgoPerfectModel::Init() { return true; }

void EgoPerfectModel::Step(double dt) {
  (void)dt;
  double current_time_sec;
  std::shared_ptr<apollo::planning::ADCTrajectory> trajectory;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    trajectory = current_trajectory_;
    current_time_sec = last_sim_time_sec_;
  }

  apollo::common::TrajectoryPoint target_point;

  if (!trajectory || trajectory->trajectory_point_size() == 0) {
    // No trajectory yet, but maintain the transform and publish localization
    target_point.mutable_path_point()->set_x(current_x_);
    target_point.mutable_path_point()->set_y(current_y_);
    target_point.mutable_path_point()->set_theta(current_heading_);
    target_point.set_v(0.0);
    target_point.set_a(0.0);
  } else {
    // Find matched point based on time (relative to trajectory header)
    double relative_time = current_time_sec - trajectory->header().timestamp_sec();

    if (relative_time < trajectory->trajectory_point(0).relative_time()) {
      target_point = trajectory->trajectory_point(0);
    } else if (relative_time >=
               trajectory
                   ->trajectory_point(trajectory->trajectory_point_size() - 1)
                   .relative_time()) {
      target_point =
          trajectory->trajectory_point(trajectory->trajectory_point_size() - 1);
    } else {
      // Linear interpolation
      for (int i = 0; i < trajectory->trajectory_point_size() - 1; ++i) {
        if (trajectory->trajectory_point(i).relative_time() <= relative_time &&
            trajectory->trajectory_point(i + 1).relative_time() >
                relative_time) {
          target_point =
              apollo::common::math::InterpolateUsingLinearApproximation(
                  trajectory->trajectory_point(i),
                  trajectory->trajectory_point(i + 1), relative_time);
          break;
        }
      }
    }
    // Update internal state
    current_x_ = target_point.path_point().x();
    current_y_ = target_point.path_point().y();
    current_heading_ = target_point.path_point().theta();
    current_velocity_ = target_point.v();
    current_acceleration_ = target_point.a();
  }

  current_time_sec_ = current_time_sec;
  target_point_ = target_point;
}

void EgoPerfectModel::Publish() {
  PublishLocalization(target_point_, current_time_sec_);
  PublishChassis(target_point_, current_time_sec_);
}

void EgoPerfectModel::PublishLocalization(
    const apollo::common::TrajectoryPoint& point, double absolute_time) {
  auto localization =
      std::make_shared<apollo::localization::LocalizationEstimate>();
  apollo::common::util::FillHeader("EgoPerfectModel", localization.get());
  localization->mutable_header()->set_timestamp_sec(absolute_time);

  auto* pose = localization->mutable_pose();
  pose->mutable_position()->set_x(point.path_point().x());
  pose->mutable_position()->set_y(point.path_point().y());
  pose->mutable_position()->set_z(0.0);
  pose->set_heading(point.path_point().theta());

  pose->mutable_linear_velocity()->set_x(std::cos(point.path_point().theta()) *
                                         point.v());
  pose->mutable_linear_velocity()->set_y(std::sin(point.path_point().theta()) *
                                         point.v());
  pose->mutable_linear_acceleration()->set_x(
      std::cos(point.path_point().theta()) * point.a());
  pose->mutable_linear_acceleration()->set_y(
      std::sin(point.path_point().theta()) * point.a());

  apollo::common::math::Quaternion quaternion =
      apollo::common::math::HeadingToQuaternion(point.path_point().theta());
  pose->mutable_orientation()->set_qw(quaternion.w());
  pose->mutable_orientation()->set_qx(quaternion.x());
  pose->mutable_orientation()->set_qy(quaternion.y());
  pose->mutable_orientation()->set_qz(quaternion.z());

  localization_writer_->Write(localization);
}

void EgoPerfectModel::PublishChassis(
    const apollo::common::TrajectoryPoint& point, double absolute_time) {
  auto chassis = std::make_shared<apollo::canbus::Chassis>();
  apollo::common::util::FillHeader("EgoPerfectModel", chassis.get());
  chassis->mutable_header()->set_timestamp_sec(absolute_time);

  chassis->set_engine_started(true);
  chassis->set_driving_mode(apollo::canbus::Chassis::COMPLETE_AUTO_DRIVE);
  chassis->set_gear_location(apollo::canbus::Chassis::GEAR_DRIVE);
  chassis->set_speed_mps(point.v());

  if (point.path_point().kappa() > 0.0) {
    chassis->set_steering_percentage(point.path_point().kappa() * 100.0);
  }

  chassis_writer_->Write(chassis);
}

void EgoPerfectModel::SetSimTime(double sim_time_sec) {
  std::lock_guard<std::mutex> lock(mutex_);
  last_sim_time_sec_ = sim_time_sec;
}

EgoState EgoPerfectModel::GetState() const {
  std::lock_guard<std::mutex> lock(mutex_);
  EgoState s;
  s.x = current_x_;
  s.y = current_y_;
  s.z = current_z_;
  s.heading = current_heading_;
  s.v = current_velocity_;
  s.a = current_acceleration_;
  s.kappa = target_point_.path_point().kappa();
  s.steering_percentage = (target_point_.path_point().kappa() > 0.0)
                             ? target_point_.path_point().kappa() * 100.0
                             : 0.0;
  return s;
}

}  // namespace dreamview
}  // namespace apollo
