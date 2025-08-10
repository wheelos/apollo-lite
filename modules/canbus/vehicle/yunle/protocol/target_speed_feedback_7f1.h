/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#pragma once

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/common_msgs/chassis_msgs/chassis_detail.pb.h"

namespace apollo {
namespace canbus {
namespace yunle {

class Targetspeedfeedback7f1 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Targetspeedfeedback7f1();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'bit': 48, 'description': '实际平均轮速', 'is_signed_var': True, 'len': 16, 'name': 'Target_speed_rpm', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': 'rpm', 'precision': 0.1, 'type': 'double'}
  double target_speed_rpm(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 32, 'description': '目标速度', 'is_signed_var': True, 'len': 16, 'name': 'Target_speed', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': 'km/h', 'precision': 0.1, 'type': 'double'}
  double target_speed(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 16, 'description': '自动驾驶模式下发目标速度', 'is_signed_var': True, 'len': 16, 'name': 'SCU_speed', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': 'km/h', 'precision': 0.1, 'type': 'double'}
  double scu_speed(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 0, 'description': '硬件目标速度', 'is_signed_var': True, 'len': 16, 'name': 'Hdw_speed', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': 'km/h', 'precision': 0.1, 'type': 'double'}
  double hdw_speed(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace yunle
}  // namespace canbus
}  // namespace apollo


