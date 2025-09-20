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

#include "modules/canbus/vehicle/apple/protocol/wheel_rl_status_1b8.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace apple {

using ::apollo::drivers::canbus::Byte;

Wheelrlstatus1b8::Wheelrlstatus1b8() {}
const int32_t Wheelrlstatus1b8::ID = 0x1B8;

void Wheelrlstatus1b8::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_apple()->mutable_wheel_rl_status_1b8()->set_errorcode_rl(errorcode_rl(bytes, length));
  chassis->mutable_apple()->mutable_wheel_rl_status_1b8()->set_statusword_rl(statusword_rl(bytes, length));
  chassis->mutable_apple()->mutable_wheel_rl_status_1b8()->set_torque_rl(torque_rl(bytes, length));
  chassis->mutable_apple()->mutable_wheel_rl_status_1b8()->set_speed_rl(speed_rl(bytes, length));
}

// config detail: {'bit': 0, 'enum': {0: 'ERRORCODE_RL_NOERROR', 1: 'ERRORCODE_RL_OVERVOLTAGE', 2: 'ERRORCODE_RL_UNDERVOLTAGE', 4: 'ERRORCODE_RL_OVERCURRENT', 5: 'ERRORCODE_RL_CONTROLLEROVERTEMP', 6: 'ERRORCODE_RL_MOTOROVERTEMP', 9: 'ERRORCODE_RL_MCUUNDERVOLTAGE', 11: 'ERRORCODE_RL_ENCODERSPIERROR', 12: 'ERRORCODE_RL_ENCODERAMPLITUDELOW', 13: 'ERRORCODE_RL_ENCODERAMPLITUDEHIGH', 14: 'ERRORCODE_RL_MEMORYSTORAGEERROR', 18: 'ERRORCODE_RL_THREEPHASEIMBALANCE', 23: 'ERRORCODE_RL_APPCONFIGSTORAGEERROR', 24: 'ERRORCODE_RL_CONTROLCONFIGSTORAGEERROR', 28: 'ERRORCODE_RL_ENCODERERROR', 30: 'ERRORCODE_RL_COMMUNICATIONLOST'}, 'is_signed_var': False, 'len': 8, 'name': 'errorcode_rl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|30]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
Wheel_rl_status_1b8::Errorcode_rlType Wheelrlstatus1b8::errorcode_rl(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Wheel_rl_status_1b8::Errorcode_rlType ret =  static_cast<Wheel_rl_status_1b8::Errorcode_rlType>(x);
  return ret;
}

// config detail: {'bit': 8, 'is_signed_var': False, 'len': 16, 'name': 'statusword_rl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|65535]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Wheelrlstatus1b8::statusword_rl(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'bit': 24, 'is_signed_var': True, 'len': 16, 'name': 'torque_rl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1000]', 'physical_unit': 'Percentage', 'precision': 0.001, 'type': 'double'}
double Wheelrlstatus1b8::torque_rl(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.001000;
  return ret;
}

// config detail: {'bit': 40, 'is_signed_var': True, 'len': 16, 'name': 'speed_rl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|25]', 'physical_unit': 'm/s', 'precision': 0.01, 'type': 'double'}
double Wheelrlstatus1b8::speed_rl(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.010000;
  return ret;
}
}  // namespace apple
}  // namespace canbus
}  // namespace apollo
