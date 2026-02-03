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

#include "modules/canbus/vehicle/agilex_hunter/protocol/motor_lowspeed_263_263.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace agilex_hunter {

using ::apollo::drivers::canbus::Byte;

Motorlowspeed263263::Motorlowspeed263263() {}
const int32_t Motorlowspeed263263::ID = 0x263;

void Motorlowspeed263263::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_agilex_hunter()->mutable_motor_lowspeed_263_263()->set_driver_voltage(driver_voltage(bytes, length));
  chassis->mutable_agilex_hunter()->mutable_motor_lowspeed_263_263()->set_driver_temp(driver_temp(bytes, length));
  chassis->mutable_agilex_hunter()->mutable_motor_lowspeed_263_263()->set_motor_temp(motor_temp(bytes, length));
  chassis->mutable_agilex_hunter()->mutable_motor_lowspeed_263_263()->set_driver_status(driver_status(bytes, length));
}

// config detail: {'bit': 15, 'is_signed_var': False, 'len': 16, 'name': 'driver_voltage', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|100]', 'physical_unit': 'V', 'precision': 0.1, 'type': 'double'}
double Motorlowspeed263263::driver_voltage(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 31, 'is_signed_var': True, 'len': 16, 'name': 'driver_temp', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-40|125]', 'physical_unit': 'C', 'precision': 1.0, 'type': 'int'}
int Motorlowspeed263263::driver_temp(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 4);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  int ret = x;
  return ret;
}

// config detail: {'bit': 39, 'is_signed_var': True, 'len': 8, 'name': 'motor_temp', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-40|125]', 'physical_unit': 'C', 'precision': 1.0, 'type': 'int'}
int Motorlowspeed263263::motor_temp(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  int ret = x;
  return ret;
}

// config detail: {'bit': 47, 'is_signed_var': False, 'len': 8, 'name': 'driver_status', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Motorlowspeed263263::driver_status(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}
}  // namespace agilex_hunter
}  // namespace canbus
}  // namespace apollo
