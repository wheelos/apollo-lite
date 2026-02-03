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

#include "modules/canbus/vehicle/agilex_hunter/protocol/motor_highspeed_253_253.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace agilex_hunter {

using ::apollo::drivers::canbus::Byte;

Motorhighspeed253253::Motorhighspeed253253() {}
const int32_t Motorhighspeed253253::ID = 0x253;

void Motorhighspeed253253::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_agilex_hunter()->mutable_motor_highspeed_253_253()->set_motor_speed(motor_speed(bytes, length));
  chassis->mutable_agilex_hunter()->mutable_motor_highspeed_253_253()->set_motor_current(motor_current(bytes, length));
}

// config detail: {'bit': 15, 'is_signed_var': True, 'len': 16, 'name': 'motor_speed', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-5000|5000]', 'physical_unit': 'rpm', 'precision': 1.0, 'type': 'int'}
int Motorhighspeed253253::motor_speed(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  int ret = x;
  return ret;
}

// config detail: {'bit': 31, 'is_signed_var': True, 'len': 16, 'name': 'motor_current', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-300|300]', 'physical_unit': 'A', 'precision': 0.1, 'type': 'double'}
double Motorhighspeed253253::motor_current(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 4);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.100000;
  return ret;
}
}  // namespace agilex_hunter
}  // namespace canbus
}  // namespace apollo
