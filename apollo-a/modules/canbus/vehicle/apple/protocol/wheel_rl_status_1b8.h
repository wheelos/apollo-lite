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

class Wheelrlstatus1b8 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Wheelrlstatus1b8();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'bit': 0, 'enum': {0: 'ERRORCODE_RL_NOERROR', 1: 'ERRORCODE_RL_OVERVOLTAGE', 2: 'ERRORCODE_RL_UNDERVOLTAGE', 4: 'ERRORCODE_RL_OVERCURRENT', 5: 'ERRORCODE_RL_CONTROLLEROVERTEMP', 6: 'ERRORCODE_RL_MOTOROVERTEMP', 9: 'ERRORCODE_RL_MCUUNDERVOLTAGE', 11: 'ERRORCODE_RL_ENCODERSPIERROR', 12: 'ERRORCODE_RL_ENCODERAMPLITUDELOW', 13: 'ERRORCODE_RL_ENCODERAMPLITUDEHIGH', 14: 'ERRORCODE_RL_MEMORYSTORAGEERROR', 18: 'ERRORCODE_RL_THREEPHASEIMBALANCE', 23: 'ERRORCODE_RL_APPCONFIGSTORAGEERROR', 24: 'ERRORCODE_RL_CONTROLCONFIGSTORAGEERROR', 28: 'ERRORCODE_RL_ENCODERERROR', 30: 'ERRORCODE_RL_COMMUNICATIONLOST'}, 'is_signed_var': False, 'len': 8, 'name': 'ErrorCode_rl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|30]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  Wheel_rl_status_1b8::Errorcode_rlType errorcode_rl(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 8, 'is_signed_var': False, 'len': 16, 'name': 'StatusWord_rl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|65535]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int statusword_rl(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 24, 'is_signed_var': True, 'len': 16, 'name': 'Torque_rl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1000]', 'physical_unit': 'Percentage', 'precision': 0.001, 'type': 'double'}
  double torque_rl(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 40, 'is_signed_var': True, 'len': 16, 'name': 'Speed_rl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|25]', 'physical_unit': 'm/s', 'precision': 0.01, 'type': 'double'}
  double speed_rl(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace apple
}  // namespace canbus
}  // namespace apollo


