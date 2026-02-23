// Copyright 2025 WheelOS All Rights Reserved.
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

//  Created Date: 2025-12-13
//  Author: daohu527

#include <memory>
#include <thread>

#include "gtest/gtest.h"

#include "behaviortree_cpp/bt_factory.h"

#include "modules/common_msgs/chassis_msgs/chassis.pb.h"
#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/mission/common/bt_type_converters.h"
#include "modules/mission/common/mission_context.h"
#include "modules/mission/nodes/action/charge_node.h"
#include "modules/mission/nodes/action/move_to_node.h"
#include "modules/mission/nodes/action/station_wait_node.h"
#include "modules/mission/nodes/condition/check_battery.h"

namespace apollo {
namespace mission {

// ---------------------------------------------------------------------------
// Helper: build a BT node under test via a minimal BehaviorTreeFactory
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// bt_type_converters tests
// ---------------------------------------------------------------------------

TEST(BtTypeConvertersTest, ConvertPointENU_XY) {
  auto p = BT::convertFromString<apollo::common::PointENU>("1.0,2.0");
  EXPECT_DOUBLE_EQ(p.x(), 1.0);
  EXPECT_DOUBLE_EQ(p.y(), 2.0);
  EXPECT_DOUBLE_EQ(p.z(), 0.0);
}

TEST(BtTypeConvertersTest, ConvertPointENU_XYZ) {
  auto p = BT::convertFromString<apollo::common::PointENU>("3.5,4.5,5.5");
  EXPECT_DOUBLE_EQ(p.x(), 3.5);
  EXPECT_DOUBLE_EQ(p.y(), 4.5);
  EXPECT_DOUBLE_EQ(p.z(), 5.5);
}

TEST(BtTypeConvertersTest, ConvertPointENU_InvalidThrows) {
  EXPECT_THROW(BT::convertFromString<apollo::common::PointENU>("only_one"),
               BT::RuntimeError);
}

// ---------------------------------------------------------------------------
// CheckBatteryNode tests
// ---------------------------------------------------------------------------

class CheckBatteryNodeTest : public ::testing::Test {
 protected:
  void SetUp() override {
    factory_.registerNodeType<CheckBatteryNode>("CheckBattery");
  }

  BT::BehaviorTreeFactory factory_;
};

TEST_F(CheckBatteryNodeTest, ReturnFailureWhenNoChassis) {
  // No chassis data injected → should return FAILURE
  MissionContext::Instance()->UpdateChassis(nullptr);

  auto blackboard = BT::Blackboard::create();
  blackboard->set("min_percentage", 20.0);

  std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="TestBattery">
        <Condition ID="CheckBattery" min_percentage="20.0" />
      </BehaviorTree>
    </root>
  )";
  factory_.registerBehaviorTreeFromText(xml);
  auto tree = factory_.createTree("TestBattery", blackboard);

  BT::NodeStatus status = tree.tickOnce();
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(CheckBatteryNodeTest, ReturnSuccessWhenBatteryAboveThreshold) {
  auto chassis = std::make_shared<apollo::canbus::Chassis>();
  chassis->set_battery_soc_percentage(80.0);
  MissionContext::Instance()->UpdateChassis(chassis);

  auto blackboard = BT::Blackboard::create();

  std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="TestBattery2">
        <Condition ID="CheckBattery" min_percentage="20.0" />
      </BehaviorTree>
    </root>
  )";
  factory_.registerBehaviorTreeFromText(xml);
  auto tree = factory_.createTree("TestBattery2", blackboard);

  BT::NodeStatus status = tree.tickOnce();
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST_F(CheckBatteryNodeTest, ReturnFailureWhenBatteryBelowThreshold) {
  auto chassis = std::make_shared<apollo::canbus::Chassis>();
  chassis->set_battery_soc_percentage(10.0);
  MissionContext::Instance()->UpdateChassis(chassis);

  auto blackboard = BT::Blackboard::create();

  std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="TestBattery3">
        <Condition ID="CheckBattery" min_percentage="20.0" />
      </BehaviorTree>
    </root>
  )";
  factory_.registerBehaviorTreeFromText(xml);
  auto tree = factory_.createTree("TestBattery3", blackboard);

  BT::NodeStatus status = tree.tickOnce();
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

// ---------------------------------------------------------------------------
// StationWaitNode tests
// ---------------------------------------------------------------------------

class StationWaitNodeTest : public ::testing::Test {
 protected:
  void SetUp() override {
    factory_.registerNodeType<StationWaitNode>("StationWait");
  }

  BT::BehaviorTreeFactory factory_;
};

TEST_F(StationWaitNodeTest, ReturnsRunningImmediately) {
  auto blackboard = BT::Blackboard::create();
  std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="TestWait1">
        <Action ID="StationWait" seconds="100.0" />
      </BehaviorTree>
    </root>
  )";
  factory_.registerBehaviorTreeFromText(xml);
  auto tree = factory_.createTree("TestWait1", blackboard);

  // First tick starts the timer and returns RUNNING
  BT::NodeStatus status = tree.tickOnce();
  EXPECT_EQ(status, BT::NodeStatus::RUNNING);
}

TEST_F(StationWaitNodeTest, ReturnsSuccessAfterElapsed) {
  auto blackboard = BT::Blackboard::create();
  std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="TestWait2">
        <Action ID="StationWait" seconds="0.01" />
      </BehaviorTree>
    </root>
  )";
  factory_.registerBehaviorTreeFromText(xml);
  auto tree = factory_.createTree("TestWait2", blackboard);

  tree.tickOnce();  // starts RUNNING
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  BT::NodeStatus status = tree.tickOnce();
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

// ---------------------------------------------------------------------------
// MoveToNode tests
// ---------------------------------------------------------------------------

class MoveToNodeTest : public ::testing::Test {
 protected:
  void SetUp() override {
    factory_.registerNodeType<MoveToNode>("MoveTo");
  }

  BT::BehaviorTreeFactory factory_;
};

TEST_F(MoveToNodeTest, ReturnsRunningWhenNoLocalization) {
  // No localization data → onRunning should keep returning RUNNING
  MissionContext::Instance()->UpdateLocalization(nullptr);

  auto blackboard = BT::Blackboard::create();
  apollo::common::PointENU goal;
  goal.set_x(100.0);
  goal.set_y(200.0);
  blackboard->set("target_goal", goal);

  std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="TestMoveTo1">
        <Action ID="MoveTo" goal="{target_goal}" />
      </BehaviorTree>
    </root>
  )";
  factory_.registerBehaviorTreeFromText(xml);
  auto tree = factory_.createTree("TestMoveTo1", blackboard);

  // onStart → sends routing (no-op since writer is null), returns RUNNING
  BT::NodeStatus status = tree.tickOnce();
  EXPECT_EQ(status, BT::NodeStatus::RUNNING);

  // onRunning → no localization, returns RUNNING
  status = tree.tickOnce();
  EXPECT_EQ(status, BT::NodeStatus::RUNNING);
}

TEST_F(MoveToNodeTest, ReturnsSuccessWhenNearGoal) {
  // Set localization very close to the goal
  auto loc = std::make_shared<apollo::localization::LocalizationEstimate>();
  loc->mutable_pose()->mutable_position()->set_x(10.0);
  loc->mutable_pose()->mutable_position()->set_y(10.0);
  MissionContext::Instance()->UpdateLocalization(loc);

  auto blackboard = BT::Blackboard::create();
  apollo::common::PointENU goal;
  goal.set_x(10.5);  // distance < 3.0 m → SUCCESS
  goal.set_y(10.5);
  blackboard->set("target_goal2", goal);

  std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="TestMoveTo2">
        <Action ID="MoveTo" goal="{target_goal2}" />
      </BehaviorTree>
    </root>
  )";
  factory_.registerBehaviorTreeFromText(xml);
  auto tree = factory_.createTree("TestMoveTo2", blackboard);

  tree.tickOnce();  // onStart → RUNNING
  BT::NodeStatus status = tree.tickOnce();  // onRunning → near goal → SUCCESS
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST_F(MoveToNodeTest, ReturnsRunningWhenFarFromGoal) {
  auto loc = std::make_shared<apollo::localization::LocalizationEstimate>();
  loc->mutable_pose()->mutable_position()->set_x(0.0);
  loc->mutable_pose()->mutable_position()->set_y(0.0);
  MissionContext::Instance()->UpdateLocalization(loc);

  auto blackboard = BT::Blackboard::create();
  apollo::common::PointENU goal;
  goal.set_x(100.0);
  goal.set_y(100.0);
  blackboard->set("target_goal3", goal);

  std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="TestMoveTo3">
        <Action ID="MoveTo" goal="{target_goal3}" />
      </BehaviorTree>
    </root>
  )";
  factory_.registerBehaviorTreeFromText(xml);
  auto tree = factory_.createTree("TestMoveTo3", blackboard);

  tree.tickOnce();  // onStart
  BT::NodeStatus status = tree.tickOnce();  // onRunning → far → RUNNING
  EXPECT_EQ(status, BT::NodeStatus::RUNNING);
}

// ---------------------------------------------------------------------------
// ChargeNode tests
// ---------------------------------------------------------------------------

class ChargeNodeTest : public ::testing::Test {
 protected:
  void SetUp() override {
    factory_.registerNodeType<ChargeNode>("Charge");
  }

  BT::BehaviorTreeFactory factory_;
};

TEST_F(ChargeNodeTest, ReturnsRunningWhenNoChassis) {
  MissionContext::Instance()->UpdateChassis(nullptr);

  auto blackboard = BT::Blackboard::create();
  std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="TestCharge1">
        <Action ID="Charge" target_soc="80.0" />
      </BehaviorTree>
    </root>
  )";
  factory_.registerBehaviorTreeFromText(xml);
  auto tree = factory_.createTree("TestCharge1", blackboard);

  tree.tickOnce();  // onStart → RUNNING
  BT::NodeStatus status = tree.tickOnce();  // onRunning, no chassis → RUNNING
  EXPECT_EQ(status, BT::NodeStatus::RUNNING);
}

TEST_F(ChargeNodeTest, ReturnsSuccessWhenTargetSocReached) {
  auto chassis = std::make_shared<apollo::canbus::Chassis>();
  chassis->set_battery_soc_percentage(90.0);
  MissionContext::Instance()->UpdateChassis(chassis);

  auto blackboard = BT::Blackboard::create();
  std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="TestCharge2">
        <Action ID="Charge" target_soc="80.0" />
      </BehaviorTree>
    </root>
  )";
  factory_.registerBehaviorTreeFromText(xml);
  auto tree = factory_.createTree("TestCharge2", blackboard);

  tree.tickOnce();  // onStart → RUNNING
  BT::NodeStatus status = tree.tickOnce();  // onRunning → SOC 90 >= 80 → SUCCESS
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

// ---------------------------------------------------------------------------
// BT factory integration test: load XML tree and tick through nodes
// ---------------------------------------------------------------------------

class BtFactoryIntegrationTest : public ::testing::Test {
 protected:
  void SetUp() override {
    factory_.registerNodeType<MoveToNode>("MoveTo");
    factory_.registerNodeType<StationWaitNode>("StationWait");
    factory_.registerNodeType<CheckBatteryNode>("CheckBattery");
    factory_.registerNodeType<ChargeNode>("Charge");
  }

  BT::BehaviorTreeFactory factory_;
};

TEST_F(BtFactoryIntegrationTest, LoadAndTickPatrolTask) {
  // Set sufficient battery so CheckBattery passes
  auto chassis = std::make_shared<apollo::canbus::Chassis>();
  chassis->set_battery_soc_percentage(80.0);
  MissionContext::Instance()->UpdateChassis(chassis);

  // Set localization far from stations (keeps MoveTo RUNNING)
  auto loc = std::make_shared<apollo::localization::LocalizationEstimate>();
  loc->mutable_pose()->mutable_position()->set_x(0.0);
  loc->mutable_pose()->mutable_position()->set_y(0.0);
  MissionContext::Instance()->UpdateLocalization(loc);

  apollo::common::PointENU station_a;
  station_a.set_x(100.0);
  station_a.set_y(200.0);

  apollo::common::PointENU station_b;
  station_b.set_x(300.0);
  station_b.set_y(400.0);

  std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="PatrolTask">
        <Sequence>
          <Action ID="MoveTo" goal="{Station_A}" />
          <Action ID="StationWait" seconds="100.0" />
          <Action ID="MoveTo" goal="{Station_B}" />
          <Action ID="StationWait" seconds="100.0" />
        </Sequence>
      </BehaviorTree>
    </root>
  )";
  factory_.registerBehaviorTreeFromText(xml);

  auto blackboard = BT::Blackboard::create();
  blackboard->set("Station_A", station_a);
  blackboard->set("Station_B", station_b);

  auto tree = factory_.createTree("PatrolTask", blackboard);

  // First tick: MoveTo onStart → RUNNING
  BT::NodeStatus status = tree.tickOnce();
  EXPECT_EQ(status, BT::NodeStatus::RUNNING);

  // Second tick: MoveTo onRunning → still RUNNING (loc far from goal)
  status = tree.tickOnce();
  EXPECT_EQ(status, BT::NodeStatus::RUNNING);
}

TEST_F(BtFactoryIntegrationTest, ShuttleBusTreeWithLowBatteryFails) {
  // Low battery → CheckBattery returns FAILURE → whole sequence fails
  auto chassis = std::make_shared<apollo::canbus::Chassis>();
  chassis->set_battery_soc_percentage(5.0);
  MissionContext::Instance()->UpdateChassis(chassis);

  apollo::common::PointENU station_a;
  station_a.set_x(100.0);
  station_a.set_y(200.0);

  apollo::common::PointENU station_b;
  station_b.set_x(300.0);
  station_b.set_y(400.0);

  std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="ShuttleBusTest">
        <Sequence name="RouteSequence">
          <Sequence name="StationA_Ops">
            <Condition ID="CheckBattery" min_percentage="20.0" />
            <Action ID="MoveTo" goal="{Station_A}" />
            <Action ID="StationWait" seconds="30.0" />
          </Sequence>
          <Sequence name="StationB_Ops">
            <Condition ID="CheckBattery" min_percentage="20.0" />
            <Action ID="MoveTo" goal="{Station_B}" />
            <Action ID="StationWait" seconds="60.0" />
          </Sequence>
        </Sequence>
      </BehaviorTree>
    </root>
  )";
  factory_.registerBehaviorTreeFromText(xml);

  auto blackboard = BT::Blackboard::create();
  blackboard->set("Station_A", station_a);
  blackboard->set("Station_B", station_b);

  auto tree = factory_.createTree("ShuttleBusTest", blackboard);

  // CheckBattery fails → whole sequence FAILURE
  BT::NodeStatus status = tree.tickOnce();
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

}  // namespace mission
}  // namespace apollo
