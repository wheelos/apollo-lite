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

#include "modules/control/controller/mpc_controller.h"

#include <cmath>
#include <memory>
#include <string>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "gflags/gflags.h"

#include "modules/control/proto/control_conf.pb.h"
#include "wheelos_msgs/planning_msgs/planning.pb.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/control/common/control_gflags.h"
#include "modules/control/controller/controller_agent.h"

namespace apollo {
namespace control {

using apollo::cyber::Clock;
using PlanningTrajectoryPb = planning::ADCTrajectory;
using LocalizationPb = localization::LocalizationEstimate;
using ChassisPb = canbus::Chassis;

class MPCControllerTest : public ::testing::Test, MPCController {
 public:
  void SetUp() override {
    FLAGS_enable_map_reference_unify = false;
    FLAGS_enable_gain_scheduler = false;
    timestamp_ = Clock::NowInSeconds();
  }

  void ComputeLateralErrors(const double x, const double y, const double theta,
                            const double linear_v, const double angular_v,
                            const double linear_a,
                            const TrajectoryAnalyzer &trajectory_analyzer,
                            SimpleMPCDebug *debug) {
    MPCController::ComputeLateralErrors(x, y, theta, linear_v, angular_v,
                                        linear_a, trajectory_analyzer, debug);
  }

 protected:
  LocalizationPb LoadLocalizaionPb(const std::string &filename) {
    LocalizationPb localization_pb;
    ACHECK(cyber::common::GetProtoFromFile(filename, &localization_pb))
        << "Failed to open file " << filename;
    localization_pb.mutable_header()->set_timestamp_sec(timestamp_);
    return localization_pb;
  }

  ChassisPb LoadChassisPb(const std::string &filename) {
    ChassisPb chassis_pb;
    ACHECK(cyber::common::GetProtoFromFile(filename, &chassis_pb))
        << "Failed to open file " << filename;
    chassis_pb.mutable_header()->set_timestamp_sec(timestamp_);
    return chassis_pb;
  }

  PlanningTrajectoryPb LoadPlanningTrajectoryPb(const std::string &filename) {
    PlanningTrajectoryPb planning_trajectory_pb;
    ACHECK(cyber::common::GetProtoFromFile(filename, &planning_trajectory_pb))
        << "Failed to open file " << filename;
    planning_trajectory_pb.mutable_header()->set_timestamp_sec(timestamp_);
    return planning_trajectory_pb;
  }

  void PrepareStraightCommandCase() {
    common::VehicleConfig vehicle_config;
    ASSERT_TRUE(cyber::common::GetProtoFromFile(
        "modules/global_config/vehicle_param.pb.txt", &vehicle_config));
    common::VehicleConfigHelper::Init(vehicle_config);
    ASSERT_TRUE(cyber::common::GetProtoFromFile(
        "modules/control/conf/control_conf.pb.txt", &command_conf_));
    command_conf_.clear_active_controllers();
    command_conf_.add_active_controllers(ControlConf::MPC_CONTROLLER);
    auto *pose = localization_.mutable_pose();
    pose->mutable_position()->set_x(0.0);
    pose->mutable_position()->set_y(0.0);
    pose->mutable_position()->set_z(0.0);
    pose->set_heading(0.0);
    pose->mutable_orientation()->set_qz(-std::sqrt(0.5));
    pose->mutable_orientation()->set_qw(std::sqrt(0.5));
    pose->mutable_angular_velocity()->set_z(0.0);
    pose->mutable_linear_acceleration()->set_y(0.0);
    chassis_.set_speed_mps(2.0);
    chassis_.set_gear_location(ChassisPb::GEAR_DRIVE);
    chassis_.set_driving_mode(ChassisPb::COMPLETE_AUTO_DRIVE);
    injector_ = std::make_shared<DependencyInjector>();
    ASSERT_TRUE(injector_->UpdateVehicleState(localization_, chassis_).ok());

    trajectory_.mutable_header()->set_timestamp_sec(timestamp_);
    trajectory_.set_gear(ChassisPb::GEAR_DRIVE);
    trajectory_.set_trajectory_type(PlanningTrajectoryPb::NORMAL);
    for (int i = 0; i <= 20; ++i) {
      auto *point = trajectory_.add_trajectory_point();
      point->mutable_path_point()->set_x(0.3 * i);
      point->mutable_path_point()->set_y(0.0);
      point->mutable_path_point()->set_s(0.3 * i);
      point->mutable_path_point()->set_theta(0.0);
      point->set_v(3.0);
      point->set_a(0.0);
      point->set_relative_time(0.1 * i);
    }
  }

  gflags::FlagSaver flag_saver_;
  ControlConf command_conf_;
  LocalizationPb localization_;
  ChassisPb chassis_;
  PlanningTrajectoryPb trajectory_;
  std::shared_ptr<DependencyInjector> injector_;
  double timestamp_ = 0.0;
};

TEST_F(MPCControllerTest, ComputesAccelerationWithUnboundedStates) {
  ASSERT_NO_FATAL_FAILURE(PrepareStraightCommandCase());
  ControllerAgent agent;
  ASSERT_TRUE(agent.Init(injector_, &command_conf_).ok());
  ControlCommand command;
  const auto status = agent.ComputeControlCommand(&localization_, &chassis_,
                                                  &trajectory_, &command);
  ASSERT_TRUE(status.ok()) << status.ToString();
  const double feedback =
      command.debug().simple_mpc_debug().acceleration_cmd_closeloop();
  EXPECT_TRUE(std::isfinite(feedback));
  EXPECT_GT(feedback, 0.0);
  EXPECT_LE(feedback, common::VehicleConfigHelper::GetConfig()
                          .vehicle_param()
                          .max_acceleration());
  EXPECT_NEAR(command.steering_target(), 0.0, 1e-6);
}

TEST_F(MPCControllerTest, PropagatesSolverFailureWithoutPublishingCommand) {
  ASSERT_NO_FATAL_FAILURE(PrepareStraightCommandCase());
  command_conf_.mutable_mpc_controller_conf()->set_max_iteration(1);
  ControllerAgent agent;
  ASSERT_TRUE(agent.Init(injector_, &command_conf_).ok());
  ControlCommand command;
  command.set_acceleration(0.25);
  command.set_steering_target(7.0);
  const auto original = command.SerializeAsString();
  const auto status = agent.ComputeControlCommand(&localization_, &chassis_,
                                                  &trajectory_, &command);
  EXPECT_EQ(status.code(), common::ErrorCode::CONTROL_COMPUTE_ERROR);
  EXPECT_EQ(status.error_message(), "MPC OSQP solver failed");
  EXPECT_EQ(command.SerializeAsString(), original);
}

TEST_F(MPCControllerTest, ComputeLateralErrors) {
  auto localization_pb = LoadLocalizaionPb(
      "/apollo/modules//control/testdata/mpc_controller_test/"
      "1_localization.pb.txt");
  auto chassis_pb = LoadChassisPb(
      "/apollo/modules/control/testdata/mpc_controller_test/1_chassis.pb.txt");
  FLAGS_enable_map_reference_unify = false;
  auto injector = std::make_shared<DependencyInjector>();
  ASSERT_TRUE(injector->UpdateVehicleState(localization_pb, chassis_pb).ok());
  const auto &vehicle_state = injector->vehicle_state();

  auto planning_trajectory_pb = LoadPlanningTrajectoryPb(
      "/apollo/modules//control/testdata/mpc_controller_test/"
      "1_planning.pb.txt");
  TrajectoryAnalyzer trajectory_analyzer(&planning_trajectory_pb);

  ControlCommand cmd;
  SimpleMPCDebug *debug = cmd.mutable_debug()->mutable_simple_mpc_debug();

  ComputeLateralErrors(
      vehicle_state.x(), vehicle_state.y(), vehicle_state.heading(),
      vehicle_state.linear_velocity(), vehicle_state.angular_velocity(),
      vehicle_state.linear_acceleration(), trajectory_analyzer, debug);

  double theta_error_expected = -0.03549;
  double theta_error_dot_expected = 0.0044552856731;
  double d_error_expected = 1.30917375441;
  double d_error_dot_expected = 0.0;
  double matched_theta_expected = -1.81266;
  double matched_kappa_expected = -0.00237307;

  EXPECT_NEAR(debug->heading_error(), theta_error_expected, 0.001);
  EXPECT_NEAR(debug->heading_error_rate(), theta_error_dot_expected, 0.001);
  EXPECT_NEAR(debug->lateral_error(), d_error_expected, 0.001);
  EXPECT_NEAR(debug->lateral_error_rate(), d_error_dot_expected, 0.001);
  EXPECT_NEAR(debug->ref_heading(), matched_theta_expected, 0.001);
  EXPECT_NEAR(debug->curvature(), matched_kappa_expected, 0.001);
}

}  // namespace control
}  // namespace apollo
