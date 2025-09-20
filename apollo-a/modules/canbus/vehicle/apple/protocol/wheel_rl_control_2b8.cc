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

#include "modules/canbus/vehicle/apple/protocol/wheel_rl_control_2b8.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace apple {

using ::apollo::drivers::canbus::Byte;

Wheelrlcontrol2b8::Wheelrlcontrol2b8() {}
const int32_t Wheelrlcontrol2b8::ID = 0x2B8;

void Wheelrlcontrol2b8::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_apple()->mutable_wheel_rl_control_2b8()->set_speed_rl_ctrl(speed_rl_ctrl(bytes, length));
  chassis->mutable_apple()->mutable_wheel_rl_control_2b8()->set_runningstate_rl(runningstate_rl(bytes, length));
  chassis->mutable_apple()->mutable_wheel_rl_control_2b8()->set_directionstate_rl(directionstate_rl(bytes, length));
}

// config detail: {'bit': 0, 'is_signed_var': True, 'len': 32, 'name': 'speed_rl_ctrl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1000]', 'physical_unit': 'RPM', 'precision': 1.0, 'type': 'int'}
int Wheelrlcontrol2b8::speed_rl_ctrl(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t2(bytes + 1);
  t = t2.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t3(bytes + 0);
  t = t3.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 0;
  x >>= 0;

  int ret = x;
  return ret;
}

// config detail: {'bit': 32, 'enum': {0: 'RUNNINGSTATE_RL_STOPPED', 2: 'RUNNINGSTATE_RL_RUNNING'}, 'is_signed_var': False, 'len': 8, 'name': 'runningstate_rl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|2]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
Wheel_rl_control_2b8::Runningstate_rlType Wheelrlcontrol2b8::runningstate_rl(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Wheel_rl_control_2b8::Runningstate_rlType ret =  static_cast<Wheel_rl_control_2b8::Runningstate_rlType>(x);
  return ret;
}

// config detail: {'bit': 40, 'enum': {0: 'DIRECTIONSTATE_RL_REVERSE', 1: 'DIRECTIONSTATE_RL_FORWARD'}, 'is_signed_var': False, 'len': 8, 'name': 'directionstate_rl', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
Wheel_rl_control_2b8::Directionstate_rlType Wheelrlcontrol2b8::directionstate_rl(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  Wheel_rl_control_2b8::Directionstate_rlType ret =  static_cast<Wheel_rl_control_2b8::Directionstate_rlType>(x);
  return ret;
}
}  // namespace apple
}  // namespace canbus
}  // namespace apollo
