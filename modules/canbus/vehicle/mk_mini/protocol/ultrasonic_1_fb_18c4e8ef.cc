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

#include "modules/canbus/vehicle/mk_mini/protocol/ultrasonic_1_fb_18c4e8ef.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace mk_mini {

using ::apollo::drivers::canbus::Byte;

Ultrasonic1fb18c4e8ef::Ultrasonic1fb18c4e8ef() {}
const int32_t Ultrasonic1fb18c4e8ef::ID = 0x98c4e8ef;

void Ultrasonic1fb18c4e8ef::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_mk_mini()->mutable_ultrasonic_1_fb_18c4e8ef()->set_ultrasonic_fb_01_check_bcc(ultrasonic_fb_01_check_bcc(bytes, length));
  chassis->mutable_mk_mini()->mutable_ultrasonic_1_fb_18c4e8ef()->set_ultrasonic_fb_01_alive_cnt(ultrasonic_fb_01_alive_cnt(bytes, length));
  chassis->mutable_mk_mini()->mutable_ultrasonic_1_fb_18c4e8ef()->set_ultrasonic_fb_04(ultrasonic_fb_04(bytes, length));
  chassis->mutable_mk_mini()->mutable_ultrasonic_1_fb_18c4e8ef()->set_ultrasonic_fb_03(ultrasonic_fb_03(bytes, length));
  chassis->mutable_mk_mini()->mutable_ultrasonic_1_fb_18c4e8ef()->set_ultrasonic_fb_02(ultrasonic_fb_02(bytes, length));
  chassis->mutable_mk_mini()->mutable_ultrasonic_1_fb_18c4e8ef()->set_ultrasonic_fb_01(ultrasonic_fb_01(bytes, length));
}

// config detail: {'bit': 56, 'is_signed_var': False, 'len': 8, 'name': 'ultrasonic_fb_01_check_bcc', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Ultrasonic1fb18c4e8ef::ultrasonic_fb_01_check_bcc(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 52, 'is_signed_var': False, 'len': 4, 'name': 'ultrasonic_fb_01_alive_cnt', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Ultrasonic1fb18c4e8ef::ultrasonic_fb_01_alive_cnt(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(4, 4);

  int ret = x;
  return ret;
}

// config detail: {'bit': 36, 'is_signed_var': False, 'len': 12, 'name': 'ultrasonic_fb_04', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': 'mm', 'precision': 1.0, 'type': 'int'}
int Ultrasonic1fb18c4e8ef::ultrasonic_fb_04(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 4);
  int32_t t = t1.get_byte(4, 4);
  x <<= 4;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'bit': 24, 'is_signed_var': False, 'len': 12, 'name': 'ultrasonic_fb_03', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': 'mm', 'precision': 1.0, 'type': 'int'}
int Ultrasonic1fb18c4e8ef::ultrasonic_fb_03(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 4);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'bit': 12, 'is_signed_var': False, 'len': 12, 'name': 'ultrasonic_fb_02', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': 'mm', 'precision': 1.0, 'type': 'int'}
int Ultrasonic1fb18c4e8ef::ultrasonic_fb_02(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(4, 4);
  x <<= 4;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 12, 'name': 'ultrasonic_fb_01', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': 'mm', 'precision': 1.0, 'type': 'int'}
int Ultrasonic1fb18c4e8ef::ultrasonic_fb_01(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 4);

  Byte t1(bytes + 0);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}
}  // namespace mk_mini
}  // namespace canbus
}  // namespace apollo
