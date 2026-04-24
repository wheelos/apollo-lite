/******************************************************************************
 * Copyright 2026 WheelOS. All Rights Reserved.
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

#include "gtest/gtest.h"

#include <cstdlib>
#include <cstdio>
#include <memory>

#include "cyber/cyber.h"
#include "cyber/common/file.h"
#include "modules/common/status/status.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/util.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common_msgs/chassis_msgs/chassis.pb.h"
#include "modules/common_msgs/basic_msgs/error_code.pb.h"
#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/planning_msgs/planning.pb.h"
#include "modules/control/controller/controller_agent.h"
#include "modules/control/common/dependency_injector.h"
#include "modules/control/common/control_gflags.h"
#include "modules/control/proto/control_conf.pb.h"
#include "modules/control/proto/local_view.pb.h"
#include "modules/control/safety/safety_manager.h"

namespace apollo {
namespace control {

using apollo::canbus::Chassis;
using apollo::localization::LocalizationEstimate;
using apollo::planning::ADCTrajectory;

class ControlLoopHarness {
 public:
  bool Init(const std::string& control_conf_path) {
    if (!cyber::common::GetProtoFromFile(control_conf_path, &control_conf_)) {
      return false;
    }
    injector_ = std::make_shared<DependencyInjector>();
    if (!controller_agent_.Init(injector_, &control_conf_).ok()) {
      return false;
    }
    controller_agent_.Reset();
    safety_manager_ = std::make_unique<SafetyManager>();
    return safety_manager_->Init(control_conf_);
  }

  common::Status ProduceControlCommand(ControlCommand* control_command) {
    injector_->vehicle_state()->Update(local_view_.localization(),
                                       local_view_.chassis());

    SafetyResult input_res = safety_manager_->PreCheck(local_view_);
    common::Status status = common::Status::OK();
    bool use_previous_cmd = false;

    if (!input_res.must_bypass) {
      status = controller_agent_.ComputeControlCommand(
          &local_view_.localization(), &local_view_.chassis(),
          &local_view_.trajectory(), control_command);

      if (status.ok()) {
        SafetyResult output_res =
            safety_manager_->PostCheck(*control_command, previous_cmd_);
        if (output_res.need_freeze) {
          use_previous_cmd = true;
          status = common::Status(common::ErrorCode::CONTROL_COMPUTE_ERROR,
                                  "Output Limits Violated (Freeze)");
        }
      } else {
        use_previous_cmd = true;
        status = common::Status(common::ErrorCode::CONTROL_COMPUTE_ERROR,
                                "Controller Agent Compute failed");
      }
    } else {
      use_previous_cmd = true;
      status = common::Status(common::ErrorCode::CONTROL_COMPUTE_ERROR,
                              "Input Physics Missing (Bypass)");
    }

    if (use_previous_cmd) {
      *control_command = previous_cmd_;
      controller_agent_.Reset();
    }

    safety_manager_->ApplySafetyPolicy(control_command);
    previous_cmd_ = *control_command;
    return status;
  }

  ControlConf control_conf_;
  ControllerAgent controller_agent_;
  std::shared_ptr<DependencyInjector> injector_;
  std::unique_ptr<SafetyManager> safety_manager_;
  LocalView local_view_;
  ControlCommand previous_cmd_;
};

class PlanningFaultIntegrationTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    CHECK_EQ(setenv("CYBER_PATH", "/home/wfh/01code/apollo-lite-01/cyber", 1),
             0);
    apollo::cyber::Init("planning_fault_test");
  }

  static void TearDownTestSuite() {}

  void SetUp() override {
    constexpr char kVehicleConfigPath[] =
        "/home/wfh/01code/apollo-lite-01/modules/common/data/"
        "vehicle_param.pb.txt";
    FLAGS_vehicle_config_path =
        kVehicleConfigPath;
    FLAGS_control_conf_file =
        "/home/wfh/01code/apollo-lite-01/modules/control/testdata/conf/"
        "control_conf.pb.txt";
    FLAGS_is_control_test_mode = true;
    FLAGS_use_navigation_mode = false;

    apollo::common::VehicleConfig vehicle_config;
    ASSERT_TRUE(
        cyber::common::GetProtoFromFile(kVehicleConfigPath, &vehicle_config));
    vehicle_config.mutable_vehicle_param()->set_max_acceleration(100.0);
    apollo::common::VehicleConfigHelper::Init(vehicle_config);

    ASSERT_TRUE(harness_.Init(FLAGS_control_conf_file));

    ASSERT_TRUE(
        cyber::common::GetProtoFromFile(DataPath("1_pad.pb.txt"), &pad_));
    ASSERT_TRUE(cyber::common::GetProtoFromFile(DataPath("1_chassis.pb.txt"),
                                                &chassis_));
    ASSERT_TRUE(cyber::common::GetProtoFromFile(
        DataPath("1_localization.pb.txt"), &localization_));
    ASSERT_TRUE(cyber::common::GetProtoFromFile(DataPath("1_planning.pb.txt"),
                                                &planning_));
    *harness_.local_view_.mutable_chassis() = chassis_;
    *harness_.local_view_.mutable_localization() = localization_;
    *harness_.local_view_.mutable_trajectory() = planning_;
    *harness_.local_view_.mutable_pad_msg() = pad_;
  }

  std::string DataPath(const std::string& name) const {
    return "/home/wfh/01code/apollo-lite-01/modules/control/testdata/"
           "simple_control_test/" +
           name;
  }

  common::Status RunOneCycle(ControlCommand* cmd) {
    return harness_.ProduceControlCommand(cmd);
  }

  void AdvanceVehicleInputs(double delta_t_sec, uint32_t step) {
    auto* chassis = harness_.local_view_.mutable_chassis();
    chassis->mutable_header()->set_timestamp_sec(chassis_.header().timestamp_sec() +
                                                 delta_t_sec * step);
    chassis->mutable_header()->set_sequence_num(chassis_.header().sequence_num() +
                                                step);

    auto* localization = harness_.local_view_.mutable_localization();
    localization->mutable_header()->set_timestamp_sec(
        localization_.header().timestamp_sec() + delta_t_sec * step);
    localization->mutable_header()->set_sequence_num(
        localization_.header().sequence_num() + step);
  }

  ControlLoopHarness harness_;
  PadMessage pad_;
  Chassis chassis_;
  LocalizationEstimate localization_;
  ADCTrajectory planning_;
};

TEST_F(PlanningFaultIntegrationTest, EmptyPlanningFreezesThenSoftStops) {
  ControlCommand baseline_cmd;
  auto baseline_status = RunOneCycle(&baseline_cmd);
  ASSERT_TRUE(baseline_status.ok()) << baseline_status.ToString();
  EXPECT_EQ(harness_.safety_manager_->GetState(), SafetyState::kNormal);

  auto empty_planning = planning_;
  empty_planning.clear_trajectory_point();
  empty_planning.mutable_header()->set_sequence_num(
      planning_.header().sequence_num() + 1);
  *harness_.local_view_.mutable_trajectory() = empty_planning;

  ControlCommand bypass_cmd;
  auto bypass_status = RunOneCycle(&bypass_cmd);
  EXPECT_FALSE(bypass_status.ok());
  EXPECT_TRUE(common::util::IsProtoEqual(baseline_cmd, bypass_cmd));
  EXPECT_EQ(harness_.safety_manager_->GetState(), SafetyState::kNormal);

  ControlCommand soft_stop_cmd;
  for (int i = 1; i < 30; ++i) {
    auto status = RunOneCycle(&soft_stop_cmd);
    EXPECT_FALSE(status.ok());
  }

  EXPECT_EQ(harness_.safety_manager_->GetState(), SafetyState::kSoftStop);
  EXPECT_DOUBLE_EQ(soft_stop_cmd.throttle(), 0.0);
  EXPECT_DOUBLE_EQ(soft_stop_cmd.brake(),
                   harness_.control_conf_.soft_estop_brake());
  EXPECT_EQ(soft_stop_cmd.gear_location(), canbus::Chassis::GEAR_DRIVE);
}

TEST_F(PlanningFaultIntegrationTest, MissingPlanningRefreshReusesLastTrajectory) {
  ControlCommand initial_cmd;
  auto initial_status = RunOneCycle(&initial_cmd);
  ASSERT_TRUE(initial_status.ok()) << initial_status.ToString();
  const auto planning_seq = harness_.local_view_.trajectory().header().sequence_num();
  const auto planning_points =
      harness_.local_view_.trajectory().trajectory_point_size();
  ASSERT_GT(planning_points, 0);

  for (uint32_t i = 1; i <= 50; ++i) {
    AdvanceVehicleInputs(0.01, i);

    ControlCommand cmd;
    auto status = RunOneCycle(&cmd);
    EXPECT_TRUE(status.ok());
    EXPECT_EQ(harness_.local_view_.trajectory().header().sequence_num(),
              planning_seq);
    EXPECT_EQ(harness_.local_view_.trajectory().trajectory_point_size(),
              planning_points);
    EXPECT_EQ(harness_.safety_manager_->GetState(), SafetyState::kNormal);
  }
}

}  // namespace control
}  // namespace apollo

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  std::fflush(nullptr);
  _Exit(result);
}
