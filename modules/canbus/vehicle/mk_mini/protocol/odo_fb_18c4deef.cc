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

#include "modules/canbus/vehicle/mk_mini/protocol/odo_fb_18c4deef.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace mk_mini {

using ::apollo::drivers::canbus::Byte;

Odofb18c4deef::Odofb18c4deef() {}
const int32_t Odofb18c4deef::ID = 0x98c4deef;

void Odofb18c4deef::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_mk_mini()->mutable_odo_fb_18c4deef()->set_odo_fb_accumulative_angular(odo_fb_accumulative_angular(bytes, length));
  chassis->mutable_mk_mini()->mutable_odo_fb_18c4deef()->set_odo_fb_accumulative_mileage(odo_fb_accumulative_mileage(bytes, length));
}

// config detail: {'bit': 32, 'is_signed_var': True, 'len': 32, 'name': 'odo_fb_accumulative_angular', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 0.001, 'type': 'double'}
double Odofb18c4deef::odo_fb_accumulative_angular(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 6);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t2(bytes + 5);
  t = t2.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t3(bytes + 4);
  t = t3.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 0;
  x >>= 0;

  double ret = x * 0.001000;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': True, 'len': 32, 'name': 'odo_fb_accumulative_mileage', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 0.001, 'type': 'double'}
double Odofb18c4deef::odo_fb_accumulative_mileage(const std::uint8_t* bytes, int32_t length) const {
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

  double ret = x * 0.001000;
  return ret;
}
}  // namespace mk_mini
}  // namespace canbus
}  // namespace apollo
