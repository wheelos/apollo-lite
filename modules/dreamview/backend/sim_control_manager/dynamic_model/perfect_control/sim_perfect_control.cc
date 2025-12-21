/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/dreamview/backend/sim_control_manager/dynamic_model/perfect_control/sim_perfect_control.h"

#include <memory>

#include "cyber/common/file.h"
#include "cyber/time/clock.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/util/message_util.h"
#include "modules/common/util/util.h"

namespace apollo {
namespace dreamview {

using apollo::canbus::Chassis;
using apollo::common::Header;
using apollo::common::Point3D;
using apollo::common::Quaternion;
using apollo::common::TrajectoryPoint;
using apollo::common::math::HeadingToQuaternion;
using apollo::common::math::InterpolateUsingLinearApproximation;
using apollo::common::math::InverseQuaternionRotate;
using apollo::common::util::FillHeader;
using apollo::cyber::Clock;
using apollo::localization::LocalizationEstimate;
using apollo::planning::ADCTrajectory;
using apollo::prediction::PredictionObstacles;
using apollo::relative_map::NavigationInfo;
using apollo::routing::RoutingRequest;
using apollo::routing::RoutingResponse;
using Json = nlohmann::json;

namespace {

static constexpr double kMaxDistance = 5.0;

void TransformToVRF(const Point3D &point_mrf, const Quaternion &orientation,
                    Point3D *point_vrf) {
  Eigen::Vector3d v_mrf(point_mrf.x(), point_mrf.y(), point_mrf.z());
  auto v_vrf = InverseQuaternionRotate(orientation, v_mrf);
  point_vrf->set_x(v_vrf.x());
  point_vrf->set_y(v_vrf.y());
  point_vrf->set_z(v_vrf.z());
}

bool IsSameHeader(const Header &lhs, const Header &rhs) {
  return lhs.sequence_num() == rhs.sequence_num() &&
         lhs.timestamp_sec() == rhs.timestamp_sec();
}

}  // namespace

SimPerfectControl::SimPerfectControl(const MapService *map_service)
    : SimControlBase(),
      map_service_(map_service),
      node_(cyber::CreateNode("sim_perfect_control")),
      current_trajectory_(std::make_shared<ADCTrajectory>()),
      need_calc_origin_(true),
      origin_(Eigen::Isometry3d::Identity()) {
  InitTimerAndIO();
  // Meters (m), Radians (rad)
  InitRandomGenerators(0.1, 0.1, 0.0, 0.0174533);
}

void SimPerfectControl::InitRandomGenerators(double pos_std_dev_x,
                                             double pos_std_dev_y,
                                             double pos_std_dev_z,
                                             double heading_std_dev) {
  random_engine_ = std::mt19937(std::random_device()());

  pos_noise_x_ = std::normal_distribution<double>(0.0, pos_std_dev_x);
  pos_noise_y_ = std::normal_distribution<double>(0.0, pos_std_dev_y);
  pos_noise_z_ = std::normal_distribution<double>(0.0, pos_std_dev_z);
  heading_noise_ = std::normal_distribution<double>(0.0, heading_std_dev);
}

void SimPerfectControl::InitTimerAndIO() {
  localization_reader_ =
      node_->CreateReader<LocalizationEstimate>(FLAGS_localization_topic);
  planning_reader_ = node_->CreateReader<ADCTrajectory>(
      FLAGS_planning_trajectory_topic,
      [this](const std::shared_ptr<ADCTrajectory> &trajectory) {
        this->OnPlanning(trajectory);
      });
  routing_response_reader_ = node_->CreateReader<RoutingResponse>(
      FLAGS_routing_response_topic,
      [this](const std::shared_ptr<RoutingResponse> &routing) {
        this->OnRoutingResponse(routing);
      });
  routing_request_reader_ = node_->CreateReader<RoutingRequest>(
      FLAGS_routing_request_topic,
      [this](const std::shared_ptr<RoutingRequest> &routing_request) {
        this->OnRoutingRequest(routing_request);
      });
  navigation_reader_ = node_->CreateReader<NavigationInfo>(
      FLAGS_navigation_topic,
      [this](const std::shared_ptr<NavigationInfo> &navigation_info) {
        this->OnReceiveNavigationInfo(navigation_info);
      });
  prediction_reader_ = node_->CreateReader<PredictionObstacles>(
      FLAGS_prediction_topic,
      [this](const std::shared_ptr<PredictionObstacles> &obstacles) {
        this->OnPredictionObstacles(obstacles);
      });

  localization_writer_ =
      node_->CreateWriter<LocalizationEstimate>(FLAGS_localization_topic);
  chassis_writer_ = node_->CreateWriter<Chassis>(FLAGS_chassis_topic);
  prediction_writer_ =
      node_->CreateWriter<PredictionObstacles>(FLAGS_prediction_topic);

  // Start timer to publish localization and chassis messages.
  sim_control_timer_.reset(new cyber::Timer(
      kSimControlIntervalMs, [this]() { this->RunOnce(); }, false));
  sim_prediction_timer_.reset(new cyber::Timer(
      kSimPredictionIntervalMs, [this]() { this->PublishDummyPrediction(); },
      false));
}

void SimPerfectControl::Init(bool set_start_point,
                             nlohmann::json start_point_attr,
                             bool use_start_point_position) {
  if (set_start_point && !FLAGS_use_navigation_mode) {
    InitStartPoint(start_point_attr["start_velocity"],
                   start_point_attr["start_acceleration"]);
  }
}

void SimPerfectControl::InitStartPoint(double x, double y,
                                       double start_velocity,
                                       double start_acceleration) {
  TrajectoryPoint point;
  // Use the scenario start point as start point,
  start_point_from_localization_ = false;
  point.mutable_path_point()->set_x(x);
  point.mutable_path_point()->set_y(y);
  // z use default 0
  point.mutable_path_point()->set_z(0);
  double theta = 0.0;
  double s = 0.0;
  map_service_->GetPoseWithRegardToLane(x, y, &theta, &s);
  point.mutable_path_point()->set_theta(theta);
  point.set_v(start_velocity);
  point.set_a(start_acceleration);
  SetStartPoint(point);
}

void SimPerfectControl::InitStartPoint(double start_velocity,
                                       double start_acceleration) {
  TrajectoryPoint point;
  // Use the latest localization position as start point,
  // fall back to a dummy point from map

  localization_reader_->Observe();
  start_point_from_localization_ = false;
  if (!localization_reader_->Empty()) {
    const auto &localization = localization_reader_->GetLatestObserved();
    const auto &pose = localization->pose();
    if (map_service_->PointIsValid(pose.position().x(), pose.position().y())) {
      point.mutable_path_point()->set_x(pose.position().x());
      point.mutable_path_point()->set_y(pose.position().y());
      point.mutable_path_point()->set_z(pose.position().z());
      point.mutable_path_point()->set_theta(pose.heading());
      point.set_v(
          std::hypot(pose.linear_velocity().x(), pose.linear_velocity().y()));
      // Calculates the dot product of acceleration and velocity. The sign
      // of this projection indicates whether this is acceleration or
      // deceleration.
      double projection =
          pose.linear_acceleration().x() * pose.linear_velocity().x() +
          pose.linear_acceleration().y() * pose.linear_velocity().y();

      // Calculates the magnitude of the acceleration. Negate the value if
      // it is indeed a deceleration.
      double magnitude = std::hypot(pose.linear_acceleration().x(),
                                    pose.linear_acceleration().y());
      point.set_a(std::signbit(projection) ? -magnitude : magnitude);
      start_point_from_localization_ = true;
    }
  }
  if (!start_point_from_localization_) {
    apollo::common::PointENU start_point;
    if (!map_service_->GetStartPoint(&start_point)) {
      AWARN << "Failed to get a dummy start point from map!";
      return;
    }
    point.mutable_path_point()->set_x(start_point.x());
    point.mutable_path_point()->set_y(start_point.y());
    point.mutable_path_point()->set_z(start_point.z());
    double theta = 0.0;
    double s = 0.0;
    map_service_->GetPoseWithRegardToLane(start_point.x(), start_point.y(),
                                          &theta, &s);
    point.mutable_path_point()->set_theta(theta);
    point.set_v(start_velocity);
    point.set_a(start_acceleration);
  }
  SetStartPoint(point);
}

void SimPerfectControl::SetStartPoint(const TrajectoryPoint &start_point) {
  next_point_ = start_point;
  prev_point_index_ = next_point_index_ = 0;
  received_planning_ = false;
}

void SimPerfectControl::Reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  InternalReset();
}

void SimPerfectControl::Stop() {
  std::lock_guard<std::mutex> lock(mutex_);

  if (enabled_) {
    sim_control_timer_->Stop();
    sim_prediction_timer_->Stop();
    enabled_ = false;
  }
}

void SimPerfectControl::InternalReset() {
  current_routing_header_.Clear();
  re_routing_triggered_ = false;
  send_dummy_prediction_ = true;
  ClearPlanning();
}

void SimPerfectControl::ClearPlanning() {
  current_trajectory_->Clear();
  received_planning_ = false;
}

void SimPerfectControl::OnReceiveNavigationInfo(
    const std::shared_ptr<NavigationInfo> &navigation_info) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (navigation_info->navigation_path_size() > 0) {
    const auto &path = navigation_info->navigation_path(0).path();
    if (path.path_point_size() > 0) {
      adc_position_ = path.path_point(0);
    }
  }
}

void SimPerfectControl::OnRoutingResponse(
    const std::shared_ptr<RoutingResponse> &routing) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled_) {
    return;
  }

  CHECK_GE(routing->routing_request().waypoint_size(), 2)
      << "routing should have at least two waypoints";

  current_routing_header_ = routing->header();

  // // If this is from a planning re-routing request, or the start point has
  // // been
  // // initialized by an actual localization pose, don't reset the start point.
  // re_routing_triggered_ =
  //     routing->routing_request().header().module_name() == "planning";
  // if (!re_routing_triggered_ && !start_point_from_localization_) {
  //   ClearPlanning();
  //   TrajectoryPoint point;
  //   point.mutable_path_point()->set_x(start_pose.x());
  //   point.mutable_path_point()->set_y(start_pose.y());
  //   point.set_a(next_point_.has_a() ? next_point_.a() : 0.0);
  //   point.set_v(next_point_.has_v() ? next_point_.v() : 0.0);
  //   double theta = 0.0;
  //   double s = 0.0;
  //   map_service_->GetPoseWithRegardToLane(start_pose.x(), start_pose.y(),
  //                                         &theta, &s);
  //   point.mutable_path_point()->set_theta(theta);
  //   SetStartPoint(point);
  // }
}

void SimPerfectControl::OnRoutingRequest(
    const std::shared_ptr<RoutingRequest> &routing_request) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled_) {
    return;
  }

  CHECK_GE(routing_request->waypoint_size(), 2)
      << "routing should have at least two waypoints";

  // Set parking info for PublishRelativePose
  parking_info_ = routing_request->mutable_parking_info();

  const auto &start_pose = routing_request->waypoint(0).pose();

  ClearPlanning();
  TrajectoryPoint point;
  point.mutable_path_point()->set_x(start_pose.x());
  point.mutable_path_point()->set_y(start_pose.y());
  point.set_a(next_point_.has_a() ? next_point_.a() : 0.0);
  point.set_v(next_point_.has_v() ? next_point_.v() : 0.0);
  double theta = 0.0;
  double s = 0.0;
  const auto &start_way_point = routing_request->waypoint().Get(0);
  // If the lane id has been set, set theta as the lane heading.
  if (start_way_point.has_id()) {
    auto &hdmap = hdmap::HDMapUtil::BaseMap();
    hdmap::Id lane_id = hdmap::MakeMapId(start_way_point.id());
    auto lane = hdmap.GetLaneById(lane_id);
    if (nullptr != lane) {
      theta = lane->Heading(start_way_point.s());
    } else {
      map_service_->GetPoseWithRegardToLane(start_pose.x(), start_pose.y(),
                                            &theta, &s);
    }
  } else {
    // Find the lane nearest to the start pose and get its heading as theta.
    map_service_->GetPoseWithRegardToLane(start_pose.x(), start_pose.y(),
                                          &theta, &s);
  }

  point.mutable_path_point()->set_theta(theta);
  SetStartPoint(point);
}

void SimPerfectControl::OnPredictionObstacles(
    const std::shared_ptr<PredictionObstacles> &obstacles) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!enabled_) {
    return;
  }

  send_dummy_prediction_ = obstacles->header().module_name() == "SimPrediction";
}

void SimPerfectControl::Start() {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!enabled_) {
    // When there is no localization yet, Init(true) will use a
    // dummy point from the current map as an arbitrary start.
    // When localization is already available, we do not need to
    // reset/override the start point.
    localization_reader_->Observe();
    Json start_point_attr({});
    start_point_attr["start_velocity"] =
        next_point_.has_v() ? next_point_.v() : 0.0;
    start_point_attr["start_acceleration"] =
        next_point_.has_a() ? next_point_.a() : 0.0;
    Init(true, start_point_attr);
    InternalReset();
    sim_control_timer_->Start();
    sim_prediction_timer_->Start();
    enabled_ = true;
  }
}

void SimPerfectControl::Start(double x, double y) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!enabled_) {
    // Do not use localization info. use scenario start point to init start
    // point.
    InitStartPoint(x, y, 0, 0);
    InternalReset();
    sim_control_timer_->Start();
    sim_prediction_timer_->Start();
    enabled_ = true;
  }
}

void SimPerfectControl::OnPlanning(
    const std::shared_ptr<ADCTrajectory> &trajectory) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!enabled_) {
    return;
  }

  // Reset current trajectory and the indices upon receiving a new trajectory.
  // The routing SimPerfectControl owns must match with the one Planning has.
  if (re_routing_triggered_ ||
      IsSameHeader(trajectory->routing_header(), current_routing_header_)) {
    current_trajectory_ = trajectory;
    prev_point_index_ = 0;
    next_point_index_ = 0;
    received_planning_ = true;
  } else {
    ClearPlanning();
  }
}

void SimPerfectControl::Freeze() {
  next_point_.set_v(0.0);
  next_point_.set_a(0.0);
  prev_point_ = next_point_;
}

void SimPerfectControl::RunOnce() {
  std::lock_guard<std::mutex> lock(mutex_);

  TrajectoryPoint trajectory_point;
  Chassis::GearPosition gear_position;
  if (!PerfectControlModel(&trajectory_point, &gear_position)) {
    AERROR << "Failed to calculate next point with perfect control model";
    return;
  }

  PublishChassis(trajectory_point.v(), gear_position);
  auto ideal_local_ptr = std::make_shared<LocalizationEstimate>();
  PublishLocalization(trajectory_point, ideal_local_ptr);
}

bool SimPerfectControl::PerfectControlModel(
    TrajectoryPoint *point, Chassis::GearPosition *gear_position) {
  // Result of the interpolation.
  auto current_time = Clock::NowInSeconds();
  const auto &trajectory = current_trajectory_->trajectory_point();
  *gear_position = current_trajectory_->gear();

  if (!received_planning_) {
    prev_point_ = next_point_;
  } else {
    if (current_trajectory_->estop().is_estop() ||
        next_point_index_ >= trajectory.size()) {
      // Freeze the car when there's an estop or the current trajectory has
      // been exhausted.
      Freeze();
    } else {
      // Determine the status of the car based on received planning message.
      while (next_point_index_ < trajectory.size() &&
             current_time > trajectory.Get(next_point_index_).relative_time() +
                                current_trajectory_->header().timestamp_sec()) {
        ++next_point_index_;
      }

      if (next_point_index_ >= trajectory.size()) {
        next_point_index_ = trajectory.size() - 1;
      }

      if (next_point_index_ == 0) {
        AERROR << "First trajectory point is a future point!";
        return false;
      }

      prev_point_index_ = next_point_index_ - 1;

      next_point_ = trajectory.Get(next_point_index_);
      prev_point_ = trajectory.Get(prev_point_index_);
    }
  }

  if (current_time > next_point_.relative_time() +
                         current_trajectory_->header().timestamp_sec()) {
    // Don't try to extrapolate if relative_time passes last point
    *point = next_point_;
  } else {
    *point = InterpolateUsingLinearApproximation(
        prev_point_, next_point_,
        current_time - current_trajectory_->header().timestamp_sec());
  }
  return true;
}

void SimPerfectControl::PublishChassis(double cur_speed,
                                       Chassis::GearPosition gear_position) {
  auto chassis = std::make_shared<Chassis>();
  FillHeader("SimPerfectControl", chassis.get());

  chassis->set_engine_started(true);
  chassis->set_driving_mode(Chassis::COMPLETE_AUTO_DRIVE);
  chassis->set_gear_location(gear_position);

  chassis->set_speed_mps(static_cast<float>(cur_speed));
  // if (gear_position == canbus::Chassis::GEAR_REVERSE) {
  //   chassis->set_speed_mps(-chassis->speed_mps());
  // }

  chassis->set_throttle_percentage(0.0);
  chassis->set_brake_percentage(0.0);

  chassis_writer_->Write(chassis);
}

void SimPerfectControl::PublishLocalization(
    const TrajectoryPoint &point,
    std::shared_ptr<LocalizationEstimate> ideal_local_ptr) {
  // 1. Ideal Pose Construction
  const double ideal_x = point.path_point().x();
  const double ideal_y = point.path_point().y();
  const double ideal_z = point.path_point().z();
  const double ideal_theta = point.path_point().theta();

  auto *ideal_pose = ideal_local_ptr->mutable_pose();
  ideal_pose->mutable_position()->set_x(ideal_x);
  ideal_pose->mutable_position()->set_y(ideal_y);
  ideal_pose->mutable_position()->set_z(ideal_z);
  ideal_pose->set_heading(ideal_theta);

  // 2. Noise Injection
  // Generate a noisy pose based on the ideal pose.
  const double noisy_x = ideal_x + pos_noise_x_(random_engine_);
  const double noisy_y = ideal_y + pos_noise_y_(random_engine_);
  const double noisy_z = ideal_z + pos_noise_z_(random_engine_);
  const double noisy_theta = ideal_theta + heading_noise_(random_engine_);

  double final_x, final_y, final_z, final_theta;
  auto final_local_ptr = std::make_shared<LocalizationEstimate>();
  FillHeader("SimPerfectControl", final_local_ptr.get());

  // If relative localization is not used, directly use the noisy absolute
  // pose as the final pose.
  need_calc_origin_ = true;  // Reset the origin calculation flag when
                             // switching back to absolute localization.
  if (FLAGS_sim_perfect_control_enable_noise) {
    final_x = noisy_x;
    final_y = noisy_y;
    final_z = noisy_z;
    final_theta = noisy_theta;
  } else {
    final_x = ideal_x;
    final_y = ideal_y;
    final_z = ideal_z;
    final_theta = ideal_theta;
  }

  // 5. Apply Navigation Mode Correction
  if (FLAGS_use_navigation_mode) {
    double flu_x = point.path_point().x();
    double flu_y = point.path_point().y();

    Eigen::Vector2d enu_coordinate =
        common::math::RotateVector2d({flu_x, flu_y}, final_theta);

    enu_coordinate.x() += adc_position_.x();
    enu_coordinate.y() += adc_position_.y();

    final_x = enu_coordinate.x();
    final_y = enu_coordinate.y();
    // final_z remains unchanged, or has other logic based on navigation mode.
  }

  // 6. Fill and Publish Localization Data
  FillCommonLocalizationData(point, final_local_ptr, final_x, final_y, final_z,
                             final_theta);
  localization_writer_->Write(final_local_ptr);

  // Update adc_position_ for the next navigation mode calculation.
  adc_position_.set_x(final_x);
  adc_position_.set_y(final_y);
  adc_position_.set_z(final_z);
}

void SimPerfectControl::FillCommonLocalizationData(
    const TrajectoryPoint &point,
    std::shared_ptr<LocalizationEstimate> final_local_ptr, double x, double y,
    double z, double theta) {
  auto *pose = final_local_ptr->mutable_pose();

  pose->mutable_position()->set_x(x);
  pose->mutable_position()->set_y(y);
  pose->mutable_position()->set_z(z);
  pose->set_heading(theta);

  const auto q = HeadingToQuaternion<double>(theta);
  pose->mutable_orientation()->set_qw(q.w());
  pose->mutable_orientation()->set_qx(q.x());
  pose->mutable_orientation()->set_qy(q.y());
  pose->mutable_orientation()->set_qz(q.z());

  // Set linear_velocity
  pose->mutable_linear_velocity()->set_x(std::cos(theta) * point.v());
  pose->mutable_linear_velocity()->set_y(std::sin(theta) * point.v());
  pose->mutable_linear_velocity()->set_z(0);

  // Set angular_velocity in both map reference frame and vehicle reference
  // frame
  pose->mutable_angular_velocity()->set_x(0);
  pose->mutable_angular_velocity()->set_y(0);
  pose->mutable_angular_velocity()->set_z(point.v() *
                                          point.path_point().kappa());

  TransformToVRF(pose->angular_velocity(), pose->orientation(),
                 pose->mutable_angular_velocity_vrf());

  // Set linear_acceleration in both map reference frame and vehicle reference
  // frame
  auto *linear_acceleration = pose->mutable_linear_acceleration();
  linear_acceleration->set_x(std::cos(theta) * point.a());
  linear_acceleration->set_y(std::sin(theta) * point.a());
  linear_acceleration->set_z(0);

  TransformToVRF(pose->linear_acceleration(), pose->orientation(),
                 pose->mutable_linear_acceleration_vrf());
}

void SimPerfectControl::PublishDummyPrediction() {
  auto prediction = std::make_shared<PredictionObstacles>();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!send_dummy_prediction_) {
      return;
    }
    FillHeader("SimPrediction", prediction.get());
  }
  prediction_writer_->Write(prediction);
}

}  // namespace dreamview
}  // namespace apollo
