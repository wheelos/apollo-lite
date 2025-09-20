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
namespace apple {

class Wheelfrcontrol2b6 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Wheelfrcontrol2b6();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'bit': 0, 'is_signed_var': True, 'len': 32, 'name': 'Speed_fr_Ctrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1000]', 'physical_unit': 'RPM', 'precision': 1.0, 'type': 'int'}
  int speed_fr_ctrl(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 32, 'enum': {0: 'RUNNINGSTATE_FR_STOPPED', 2: 'RUNNINGSTATE_FR_RUNNING'}, 'is_signed_var': False, 'len': 8, 'name': 'RunningState_fr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|2]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  Wheel_fr_control_2b6::Runningstate_frType runningstate_fr(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 40, 'enum': {0: 'DIRECTIONSTATE_FR_REVERSE', 1: 'DIRECTIONSTATE_FR_FORWARD'}, 'is_signed_var': False, 'len': 8, 'name': 'DirectionState_fr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  Wheel_fr_control_2b6::Directionstate_frType directionstate_fr(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace apple
}  // namespace canbus
}  // namespace apollo


