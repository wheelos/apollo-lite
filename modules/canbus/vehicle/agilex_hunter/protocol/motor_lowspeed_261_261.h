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
namespace agilex_hunter {

class Motorlowspeed261261 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Motorlowspeed261261();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'bit': 15, 'is_signed_var': False, 'len': 16, 'name': 'Driver_Voltage', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|100]', 'physical_unit': 'V', 'precision': 0.1, 'type': 'double'}
  double driver_voltage(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 31, 'is_signed_var': True, 'len': 16, 'name': 'Driver_Temp', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-40|125]', 'physical_unit': 'C', 'precision': 1.0, 'type': 'int'}
  int driver_temp(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 39, 'is_signed_var': True, 'len': 8, 'name': 'Motor_Temp', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-40|125]', 'physical_unit': 'C', 'precision': 1.0, 'type': 'int'}
  int motor_temp(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 47, 'is_signed_var': False, 'len': 8, 'name': 'Driver_Status', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int driver_status(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace agilex_hunter
}  // namespace canbus
}  // namespace apollo


