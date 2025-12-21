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

#include "modules/canbus/vehicle/mk_mini/protocol/rr_wheel_fb_18c4d8ef.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace mk_mini {

using ::apollo::drivers::canbus::Byte;

Rrwheelfb18c4d8ef::Rrwheelfb18c4d8ef() {}
const int32_t Rrwheelfb18c4d8ef::ID = 0x98c4d8ef;

void Rrwheelfb18c4d8ef::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_mk_mini()->mutable_rr_wheel_fb_18c4d8ef()->set_rr_wheel_fb_velocity(rr_wheel_fb_velocity(bytes, length));
  chassis->mutable_mk_mini()->mutable_rr_wheel_fb_18c4d8ef()->set_rr_wheel_fb_pulse(rr_wheel_fb_pulse(bytes, length));
  chassis->mutable_mk_mini()->mutable_rr_wheel_fb_18c4d8ef()->set_rr_wheel_fb_check_bcc(rr_wheel_fb_check_bcc(bytes, length));
  chassis->mutable_mk_mini()->mutable_rr_wheel_fb_18c4d8ef()->set_rr_wheel_fb_alive_cnt(rr_wheel_fb_alive_cnt(bytes, length));
}

// config detail: {'bit': 0, 'is_signed_var': True, 'len': 16, 'name': 'rr_wheel_fb_velocity', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 0.001, 'type': 'double'}
double Rrwheelfb18c4d8ef::rr_wheel_fb_velocity(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 0);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.001000;
  return ret;
}

// config detail: {'bit': 16, 'is_signed_var': True, 'len': 32, 'name': 'rr_wheel_fb_pulse', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Rrwheelfb18c4d8ef::rr_wheel_fb_pulse(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 4);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t2(bytes + 3);
  t = t2.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t3(bytes + 2);
  t = t3.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 0;
  x >>= 0;

  int ret = x;
  return ret;
}

// config detail: {'bit': 56, 'is_signed_var': False, 'len': 8, 'name': 'rr_wheel_fb_check_bcc', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Rrwheelfb18c4d8ef::rr_wheel_fb_check_bcc(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 52, 'is_signed_var': False, 'len': 4, 'name': 'rr_wheel_fb_alive_cnt', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Rrwheelfb18c4d8ef::rr_wheel_fb_alive_cnt(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(4, 4);

  int ret = x;
  return ret;
}
}  // namespace mk_mini
}  // namespace canbus
}  // namespace apollo
