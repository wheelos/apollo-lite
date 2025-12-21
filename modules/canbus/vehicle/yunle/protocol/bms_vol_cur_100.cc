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

#include "modules/canbus/vehicle/yunle/protocol/bms_vol_cur_100.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace yunle {

using ::apollo::drivers::canbus::Byte;

Bmsvolcur100::Bmsvolcur100() {}
const int32_t Bmsvolcur100::ID = 0x100;

void Bmsvolcur100::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_yunle()->mutable_bms_vol_cur_100()->set_total_voltage(total_voltage(bytes, length));
  chassis->mutable_yunle()->mutable_bms_vol_cur_100()->set_soc_current(soc_current(bytes, length));
}

// config detail: {'bit': 7, 'is_signed_var': False, 'len': 16, 'name': 'total_voltage', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|100]', 'physical_unit': '', 'precision': 0.01, 'type': 'double'}
double Bmsvolcur100::total_voltage(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.010000;
  return ret;
}

// config detail: {'bit': 23, 'is_signed_var': True, 'len': 16, 'name': 'soc_current', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-100|100]', 'physical_unit': 'A', 'precision': 0.01, 'type': 'double'}
double Bmsvolcur100::soc_current(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.010000;
  return ret;
}
}  // namespace yunle
}  // namespace canbus
}  // namespace apollo
