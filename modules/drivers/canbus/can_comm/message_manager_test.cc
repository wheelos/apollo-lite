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

#include "modules/drivers/canbus/can_comm/message_manager.h"

#include <memory>
#include <set>

#include "gtest/gtest.h"

#include "wheelos_msgs/chassis_msgs/chassis_detail.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace drivers {
namespace canbus {

using apollo::common::ErrorCode;

class MockProtocolData : public ProtocolData<::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID = 0x111;

  void Parse(const uint8_t* bytes, int32_t length,
             ::apollo::canbus::ChassisDetail* sensor_data) const override {
    if (length > 0 && sensor_data != nullptr) {
      sensor_data->mutable_vehicle_spd()->set_vehicle_spd(bytes[0]);
    }
  }
};

class MockCheckResponseProtocolData
    : public ProtocolData<::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID = 0x112;

  void Parse(const uint8_t* bytes, int32_t length,
             ::apollo::canbus::ChassisDetail* sensor_data) const override {
    if (length > 0 && sensor_data != nullptr) {
      sensor_data->mutable_check_response()->set_is_vcu_online(bytes[0] != 0);
    }
  }
};

class MockMessageManager
    : public MessageManager<::apollo::canbus::ChassisDetail> {
 public:
  MockMessageManager() {
    AddRecvProtocolData<MockProtocolData, true>();
    AddRecvProtocolData<MockCheckResponseProtocolData, true>();
    AddSendProtocolData<MockProtocolData, true>();
  }
};

TEST(MessageManagerTest, GetMutableProtocolDataById) {
  uint8_t mock_data = 1;
  MockMessageManager manager;
  manager.Parse(MockProtocolData::ID, &mock_data, 8);
  manager.ResetSendMessages();
  EXPECT_NE(manager.GetMutableProtocolDataById(MockProtocolData::ID), nullptr);

  ::apollo::canbus::ChassisDetail chassis_detail;
  chassis_detail.set_car_type(::apollo::canbus::ChassisDetail::QIRUI_EQ_15);
  EXPECT_EQ(manager.GetSensorData(&chassis_detail), ErrorCode::OK);
  EXPECT_EQ(manager.GetSensorData(nullptr), ErrorCode::CANBUS_ERROR);
}

TEST(MessageManagerTest, GetAndClearSensorDataSeparatesEventSnapshots) {
  MockMessageManager manager;
  const uint8_t speed = 42;
  manager.Parse(MockProtocolData::ID, &speed, 1);

  ::apollo::canbus::ChassisDetail first_snapshot;
  ASSERT_EQ(manager.GetAndClearSensorData(&first_snapshot), ErrorCode::OK);
  ASSERT_TRUE(first_snapshot.has_vehicle_spd());
  EXPECT_EQ(42, first_snapshot.vehicle_spd().vehicle_spd());

  const uint8_t online = 1;
  manager.Parse(MockCheckResponseProtocolData::ID, &online, 1);
  ::apollo::canbus::ChassisDetail second_snapshot;
  ASSERT_EQ(manager.GetAndClearSensorData(&second_snapshot), ErrorCode::OK);
  EXPECT_FALSE(second_snapshot.has_vehicle_spd());
  ASSERT_TRUE(second_snapshot.has_check_response());
  EXPECT_TRUE(second_snapshot.check_response().is_vcu_online());
}

}  // namespace canbus
}  // namespace drivers
}  // namespace apollo
