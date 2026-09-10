// Copyright 2026 WheelOS. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//  Created Date: 2026-09-10
//  Author: daohu527

#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "gtest/gtest.h"

#include "wheelos_msgs/chassis_msgs/chassis.pb.h"
#include "wheelos_msgs/control_msgs/control_cmd.pb.h"
#include "wheelos_msgs/localization_msgs/localization.pb.h"
#include "wheelos_msgs/planning_msgs/planning.pb.h"

#include "cyber/cyber.h"
#include "cyber/time/clock.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/control/common/control_gflags.h"
#include "modules/control/control_component.h"
#include "modules/simulation/common/simulation_gflags.h"
#include "modules/simulation/simulation_component.h"

namespace apollo {
namespace simulation {
namespace {

std::string RunfilePath(const std::string& relative_path) {
  const char* test_srcdir = std::getenv("TEST_SRCDIR");
  const char* test_workspace = std::getenv("TEST_WORKSPACE");
  if (test_srcdir != nullptr && test_workspace != nullptr) {
    return std::string(test_srcdir) + "/" + test_workspace + "/" +
           relative_path;
  }
  return relative_path;
}

}  // namespace

class SimulationComponentIntegrationTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    cyber::Init("simulation_component_integration_test");
    cyber::Clock::SetMode(cyber::proto::MODE_CYBER);
  }

  static void TearDownTestSuite() { cyber::Clear(); }

  void SetUp() override {
    static std::atomic<int> next_id{0};
    const std::string suffix =
        std::to_string(getpid()) + "_" + std::to_string(next_id.fetch_add(1));
    FLAGS_sim_backend_type = "kinematic";
    FLAGS_sim_model_path.clear();
    FLAGS_sim_physics_dt = 0.002;
    FLAGS_sim_control_dt = 0.02;
    FLAGS_sim_command_timeout = 2.0;
    FLAGS_control_conf_file =
        RunfilePath("modules/control/testdata/conf/control_conf.pb.txt");
    FLAGS_is_control_test_mode = false;
    FLAGS_enable_map_reference_unify = false;

    FLAGS_control_command_topic = "/apollo/test/simulation/control/" + suffix;
    FLAGS_chassis_topic = "/apollo/test/simulation/chassis/" + suffix;
    FLAGS_localization_topic = "/apollo/test/simulation/localization/" + suffix;

    node_ = cyber::CreateNode("simulation_driver_" + suffix);
    ASSERT_NE(node_, nullptr);
    command_writer_ = node_->CreateWriter<apollo::control::ControlCommand>(
        FLAGS_control_command_topic);
    ASSERT_NE(command_writer_, nullptr);
    chassis_reader_ = node_->CreateReader<apollo::canbus::Chassis>(
        FLAGS_chassis_topic,
        [this](const std::shared_ptr<apollo::canbus::Chassis>& message) {
          std::lock_guard<std::mutex> lock(mutex_);
          chassis_ = *message;
          chassis_received_ = true;
          condition_.notify_all();
        });
    localization_reader_ =
        node_->CreateReader<apollo::localization::LocalizationEstimate>(
            FLAGS_localization_topic,
            [this](const std::shared_ptr<
                   apollo::localization::LocalizationEstimate>& message) {
              std::lock_guard<std::mutex> lock(mutex_);
              localization_ = *message;
              localization_received_ = true;
              condition_.notify_all();
            });
    ASSERT_NE(chassis_reader_, nullptr);
    ASSERT_NE(localization_reader_, nullptr);

    component_ = std::make_shared<SimulationComponent>();
    cyber::TimerComponentConfig config;
    config.set_name("simulation_component_test_" + suffix);
    config.set_interval(1000);
    component_->Initialize(config);
  }

  void TearDown() override {
    if (component_) {
      component_->Shutdown();
    }
    if (control_component_) {
      control_component_->Shutdown();
    }
    control_component_.reset();
    planning_writer_.reset();
    component_.reset();
    localization_reader_.reset();
    chassis_reader_.reset();
    command_writer_.reset();
    node_.reset();
  }

  apollo::planning::ADCTrajectory BuildStraightTrajectory() {
    apollo::planning::ADCTrajectory trajectory;
    trajectory.mutable_header()->set_timestamp_sec(0.0);
    trajectory.mutable_header()->set_sequence_num(1);
    trajectory.set_gear(apollo::canbus::Chassis::GEAR_DRIVE);
    trajectory.set_total_path_length(15.0);
    trajectory.set_total_path_time(5.0);
    for (int i = 0; i <= 50; ++i) {
      const double t = i * 0.1;
      auto* point = trajectory.add_trajectory_point();
      point->mutable_path_point()->set_x(3.0 * t);
      point->mutable_path_point()->set_y(0.0);
      point->mutable_path_point()->set_theta(0.0);
      point->mutable_path_point()->set_s(3.0 * t);
      point->set_v(std::min(3.0, 3.0 * t));
      point->set_a(t < 1.0 ? 3.0 : 0.0);
      point->set_relative_time(t);
    }
    return trajectory;
  }

  void SendCommand(double throttle, double brake, double steering_percentage) {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
    while (!command_writer_->HasReader() &&
           std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    ASSERT_TRUE(command_writer_->HasReader());
    auto command = std::make_shared<apollo::control::ControlCommand>();
    command->mutable_header()->set_timestamp_sec(0.0);
    command->set_throttle(throttle);
    command->set_brake(brake);
    command->set_steering_target(steering_percentage);
    command->set_steering_rate(0.0);
    command->set_gear_location(apollo::canbus::Chassis::GEAR_DRIVE);
    ASSERT_TRUE(command_writer_->Write(command));
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  void Step(int count) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      chassis_received_ = false;
      localization_received_ = false;
    }
    for (int i = 0; i < count; ++i) {
      ASSERT_TRUE(component_->Proc());
    }
  }

  bool WaitForFeedback() {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, std::chrono::milliseconds(500), [this]() {
      return chassis_received_ && localization_received_;
    });
  }

  std::shared_ptr<cyber::Node> node_;
  std::shared_ptr<cyber::Writer<apollo::control::ControlCommand>>
      command_writer_;
  std::shared_ptr<cyber::Writer<apollo::planning::ADCTrajectory>>
      planning_writer_;
  std::shared_ptr<cyber::Reader<apollo::canbus::Chassis>> chassis_reader_;
  std::shared_ptr<cyber::Reader<apollo::localization::LocalizationEstimate>>
      localization_reader_;
  std::shared_ptr<SimulationComponent> component_;
  std::shared_ptr<apollo::control::ControlComponent> control_component_;

  std::mutex mutex_;
  std::condition_variable condition_;
  apollo::canbus::Chassis chassis_;
  apollo::localization::LocalizationEstimate localization_;
  bool chassis_received_{false};
  bool localization_received_{false};
};

TEST_F(SimulationComponentIntegrationTest,
       StraightTurnAndBrakeThroughCyberTopics) {
  SendCommand(0.8, 0.0, 0.0);
  Step(25);
  ASSERT_TRUE(WaitForFeedback());
  const double straight_x = localization_.pose().position().x();
  const double straight_speed = chassis_.speed_mps();
  EXPECT_GT(straight_x, 0.0);
  EXPECT_GT(straight_speed, 0.0);

  SendCommand(0.5, 0.0, 30.0);
  Step(50);
  ASSERT_TRUE(WaitForFeedback());
  EXPECT_GT(localization_.pose().position().y(), 0.0);
  EXPECT_GT(localization_.pose().heading(), 0.0);

  SendCommand(0.0, 1.0, 0.0);
  Step(75);
  ASSERT_TRUE(WaitForFeedback());
  EXPECT_NEAR(chassis_.speed_mps(), 0.0, 0.2);
}

TEST_F(SimulationComponentIntegrationTest, MissingCommandAppliesTimeoutBrake) {
  SendCommand(0.8, 0.0, 0.0);
  Step(1);
  ASSERT_TRUE(WaitForFeedback());
  Step(110);
  ASSERT_TRUE(WaitForFeedback());
  EXPECT_DOUBLE_EQ(chassis_.throttle_percentage(), 0.0);
  EXPECT_DOUBLE_EQ(chassis_.brake_percentage(), 50.0);
}

TEST_F(SimulationComponentIntegrationTest,
       InvalidCommandAppliesEmergencyBrake) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
  while (!command_writer_->HasReader() &&
         std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  ASSERT_TRUE(command_writer_->HasReader());

  auto command = std::make_shared<apollo::control::ControlCommand>();
  command->set_throttle(std::numeric_limits<double>::quiet_NaN());
  command->set_brake(0.0);
  command->set_steering_target(0.0);
  command->set_steering_rate(0.0);
  command->set_gear_location(apollo::canbus::Chassis::GEAR_DRIVE);
  ASSERT_TRUE(command_writer_->Write(command));
  std::this_thread::sleep_for(std::chrono::milliseconds(2));

  Step(1);
  ASSERT_TRUE(WaitForFeedback());
  EXPECT_DOUBLE_EQ(chassis_.brake_percentage(), 100.0);
}

TEST_F(SimulationComponentIntegrationTest, ControlComponentTopicClosedLoop) {
  planning_writer_ = node_->CreateWriter<apollo::planning::ADCTrajectory>(
      FLAGS_planning_trajectory_topic);
  ASSERT_NE(planning_writer_, nullptr);
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
  while (!planning_writer_->HasReader() &&
         std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  control_component_ = std::make_shared<apollo::control::ControlComponent>();
  cyber::TimerComponentConfig config;
  config.set_name("control_component_test_" + std::to_string(getpid()));
  config.set_interval(1000);
  control_component_->Initialize(config);
  ASSERT_TRUE(planning_writer_->HasReader());
  ASSERT_TRUE(planning_writer_->Write(BuildStraightTrajectory()));

  double max_speed = 0.0;
  for (int i = 0; i < 120; ++i) {
    ASSERT_TRUE(component_->Proc());
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    ASSERT_TRUE(control_component_->Proc());
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    max_speed = std::max(max_speed, static_cast<double>(chassis_.speed_mps()));
  }

  ASSERT_TRUE(WaitForFeedback());
  EXPECT_GT(localization_.pose().position().x(), 0.1);
  EXPECT_GT(max_speed, 0.3);
}

}  // namespace simulation
}  // namespace apollo
