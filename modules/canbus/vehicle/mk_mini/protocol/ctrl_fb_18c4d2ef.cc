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

#include "modules/canbus/vehicle/mk_mini/protocol/ctrl_fb_18c4d2ef.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace mk_mini {

using ::apollo::drivers::canbus::Byte;

Ctrlfb18c4d2ef::Ctrlfb18c4d2ef() {}
const int32_t Ctrlfb18c4d2ef::ID = 0x98c4d2ef;

uint32_t Ctrlfb18c4d2ef::GetPeriod() const {
  static const uint32_t PERIOD = 10000;  // 10ms
  return PERIOD;
}

void Ctrlfb18c4d2ef::Parse(const std::uint8_t* bytes, int32_t length,
                           ChassisDetail* chassis) const {
  chassis->mutable_mk_mini()->mutable_ctrl_fb_18c4d2ef()->set_ctrl_fb_remotest(
      ctrl_fb_remotest(bytes, length));
  chassis->mutable_mk_mini()->mutable_ctrl_fb_18c4d2ef()->set_ctrl_fb_check_bcc(
      ctrl_fb_check_bcc(bytes, length));
  chassis->mutable_mk_mini()->mutable_ctrl_fb_18c4d2ef()->set_ctrl_fb_alive_cnt(
      ctrl_fb_alive_cnt(bytes, length));
  chassis->mutable_mk_mini()->mutable_ctrl_fb_18c4d2ef()->set_ctrl_fb_mode(
      ctrl_fb_mode(bytes, length));
  chassis->mutable_mk_mini()->mutable_ctrl_fb_18c4d2ef()->set_ctrl_fb_brake(
      ctrl_fb_brake(bytes, length));
  chassis->mutable_mk_mini()->mutable_ctrl_fb_18c4d2ef()->set_ctrl_fb_steering(
      ctrl_fb_steering(bytes, length));
  chassis->mutable_mk_mini()->mutable_ctrl_fb_18c4d2ef()->set_ctrl_fb_velocity(
      ctrl_fb_velocity(bytes, length));
  chassis->mutable_mk_mini()->mutable_ctrl_fb_18c4d2ef()->set_ctrl_fb_gear(
      ctrl_fb_gear(bytes, length));
}

// config detail: {'bit': 47, 'is_signed_var': False, 'len': 1, 'name':
// 'ctrl_fb_remotest', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Ctrlfb18c4d2ef::ctrl_fb_remotest(const std::uint8_t* bytes,
                                      int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(7, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 56, 'is_signed_var': False, 'len': 8, 'name':
// 'ctrl_fb_check_bcc', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Ctrlfb18c4d2ef::ctrl_fb_check_bcc(const std::uint8_t* bytes,
                                      int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 52, 'is_signed_var': False, 'len': 4, 'name':
// 'ctrl_fb_alive_cnt', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Ctrlfb18c4d2ef::ctrl_fb_alive_cnt(const std::uint8_t* bytes,
                                      int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(4, 4);

  int ret = x;
  return ret;
}

// config detail: {'bit': 44, 'is_signed_var': False, 'len': 2, 'name':
// 'ctrl_fb_mode', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Ctrlfb18c4d2ef::ctrl_fb_mode(const std::uint8_t* bytes,
                                 int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(4, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 36, 'is_signed_var': False, 'len': 8, 'name':
// 'ctrl_fb_brake', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Ctrlfb18c4d2ef::ctrl_fb_brake(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 4);

  Byte t1(bytes + 4);
  int32_t t = t1.get_byte(4, 4);
  x <<= 4;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'bit': 20, 'is_signed_var': True, 'len': 16, 'name':
// 'ctrl_fb_steering', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 0.01, 'type': 'double'}
double Ctrlfb18c4d2ef::ctrl_fb_steering(const std::uint8_t* bytes,
                                        int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 4);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t2(bytes + 2);
  t = t2.get_byte(4, 4);
  x <<= 4;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.010000;
  return ret;
}

// config detail: {'bit': 4, 'is_signed_var': False, 'len': 16, 'name':
// 'ctrl_fb_velocity', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 0.001, 'type': 'double'}
double Ctrlfb18c4d2ef::ctrl_fb_velocity(const std::uint8_t* bytes,
                                        int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 4);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t2(bytes + 0);
  t = t2.get_byte(4, 4);
  x <<= 4;
  x |= t;

  double ret = x * 0.001000;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 4, 'name':
// 'ctrl_fb_gear', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Ctrlfb18c4d2ef::ctrl_fb_gear(const std::uint8_t* bytes,
                                 int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

}  // namespace mk_mini
}  // namespace canbus
}  // namespace apollo
