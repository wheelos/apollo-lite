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

class Motionfeedback221 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Motionfeedback221();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'bit': 15, 'is_signed_var': True, 'len': 16, 'name': 'Vehicle_Speed', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-5|5]', 'physical_unit': 'm/s', 'precision': 0.001, 'type': 'double'}
  double vehicle_speed(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 63, 'is_signed_var': True, 'len': 16, 'name': 'Steering_Angle', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-1|1]', 'physical_unit': 'rad', 'precision': 0.001, 'type': 'double'}
  double steering_angle(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace agilex_hunter
}  // namespace canbus
}  // namespace apollo


