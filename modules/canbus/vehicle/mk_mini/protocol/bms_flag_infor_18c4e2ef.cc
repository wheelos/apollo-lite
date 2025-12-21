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

#include "modules/canbus/vehicle/mk_mini/protocol/bms_flag_infor_18c4e2ef.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace mk_mini {

using ::apollo::drivers::canbus::Byte;

Bmsflaginfor18c4e2ef::Bmsflaginfor18c4e2ef() {}
const int32_t Bmsflaginfor18c4e2ef::ID = 0x98c4e2ef;

void Bmsflaginfor18c4e2ef::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_mk_mini()->mutable_bms_flag_infor_18c4e2ef()->set_bms_flag_infor_soclowprotection(bms_flag_infor_soclowprotection(bytes, length));
  chassis->mutable_mk_mini()->mutable_bms_flag_infor_18c4e2ef()->set_bms_flag_infor_check_bcc(bms_flag_infor_check_bcc(bytes, length));
  chassis->mutable_mk_mini()->mutable_bms_flag_infor_18c4e2ef()->set_bms_flag_infor_alive_cnt(bms_flag_infor_alive_cnt(bytes, length));
  chassis->mutable_mk_mini()->mutable_bms_flag_infor_18c4e2ef()->set_bms_flag_infor_socwarning(bms_flag_infor_socwarning(bytes, length));
  chassis->mutable_mk_mini()->mutable_bms_flag_infor_18c4e2ef()->set_bms_flag_infor_low_temperature(bms_flag_infor_low_temperature(bytes, length));
  chassis->mutable_mk_mini()->mutable_bms_flag_infor_18c4e2ef()->set_bms_flag_infor_hight_temperature(bms_flag_infor_hight_temperature(bytes, length));
  chassis->mutable_mk_mini()->mutable_bms_flag_infor_18c4e2ef()->set_bms_flag_infor_charge_st(bms_flag_infor_charge_st(bytes, length));
  chassis->mutable_mk_mini()->mutable_bms_flag_infor_18c4e2ef()->set_bms_flag_infor_lock_mos(bms_flag_infor_lock_mos(bytes, length));
  chassis->mutable_mk_mini()->mutable_bms_flag_infor_18c4e2ef()->set_bms_flag_infor_ic_error(bms_flag_infor_ic_error(bytes, length));
  chassis->mutable_mk_mini()->mutable_bms_flag_infor_18c4e2ef()->set_bms_flag_infor_short(bms_flag_infor_short(bytes, length));
  chassis->mutable_mk_mini()->mutable_bms_flag_infor_18c4e2ef()->set_bms_flag_infor_discharge_oc(bms_flag_infor_discharge_oc(bytes, length));
  chassis->mutable_mk_mini()->mutable_bms_flag_infor_18c4e2ef()->set_bms_flag_infor_charge_oc(bms_flag_infor_charge_oc(bytes, length));
  chassis->mutable_mk_mini()->mutable_bms_flag_infor_18c4e2ef()->set_bms_flag_infor_discharge_ut(bms_flag_infor_discharge_ut(bytes, length));
  chassis->mutable_mk_mini()->mutable_bms_flag_infor_18c4e2ef()->set_bms_flag_infor_discharge_ot(bms_flag_infor_discharge_ot(bytes, length));
  chassis->mutable_mk_mini()->mutable_bms_flag_infor_18c4e2ef()->set_bms_flag_infor_charge_ut(bms_flag_infor_charge_ut(bytes, length));
  chassis->mutable_mk_mini()->mutable_bms_flag_infor_18c4e2ef()->set_bms_flag_infor_charge_ot(bms_flag_infor_charge_ot(bytes, length));
  chassis->mutable_mk_mini()->mutable_bms_flag_infor_18c4e2ef()->set_bms_flag_infor_uv(bms_flag_infor_uv(bytes, length));
  chassis->mutable_mk_mini()->mutable_bms_flag_infor_18c4e2ef()->set_bms_flag_infor_ov(bms_flag_infor_ov(bytes, length));
  chassis->mutable_mk_mini()->mutable_bms_flag_infor_18c4e2ef()->set_bms_flag_infor_single_uv(bms_flag_infor_single_uv(bytes, length));
  chassis->mutable_mk_mini()->mutable_bms_flag_infor_18c4e2ef()->set_bms_flag_infor_single_ov(bms_flag_infor_single_ov(bytes, length));
  chassis->mutable_mk_mini()->mutable_bms_flag_infor_18c4e2ef()->set_bms_flag_infor_soc(bms_flag_infor_soc(bytes, length));
}

// config detail: {'bit': 24, 'is_signed_var': False, 'len': 1, 'name': 'bms_flag_infor_soclowprotection', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Bmsflaginfor18c4e2ef::bms_flag_infor_soclowprotection(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 56, 'is_signed_var': False, 'len': 8, 'name': 'bms_flag_infor_check_bcc', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Bmsflaginfor18c4e2ef::bms_flag_infor_check_bcc(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 52, 'is_signed_var': False, 'len': 4, 'name': 'bms_flag_infor_alive_cnt', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Bmsflaginfor18c4e2ef::bms_flag_infor_alive_cnt(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(4, 4);

  int ret = x;
  return ret;
}

// config detail: {'bit': 23, 'is_signed_var': False, 'len': 1, 'name': 'bms_flag_infor_socwarning', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Bmsflaginfor18c4e2ef::bms_flag_infor_socwarning(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(7, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 40, 'is_signed_var': True, 'len': 12, 'name': 'bms_flag_infor_low_temperature', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 0.1, 'type': 'double'}
double Bmsflaginfor18c4e2ef::bms_flag_infor_low_temperature(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 20;
  x >>= 20;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 28, 'is_signed_var': True, 'len': 12, 'name': 'bms_flag_infor_hight_temperature', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 0.1, 'type': 'double'}
double Bmsflaginfor18c4e2ef::bms_flag_infor_hight_temperature(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(4, 4);
  x <<= 4;
  x |= t;

  x <<= 20;
  x >>= 20;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 21, 'is_signed_var': False, 'len': 2, 'name': 'bms_flag_infor_charge_st', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Bmsflaginfor18c4e2ef::bms_flag_infor_charge_st(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(5, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 20, 'is_signed_var': False, 'len': 1, 'name': 'bms_flag_infor_lock_mos', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Bmsflaginfor18c4e2ef::bms_flag_infor_lock_mos(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 19, 'is_signed_var': False, 'len': 1, 'name': 'bms_flag_infor_ic_error', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Bmsflaginfor18c4e2ef::bms_flag_infor_ic_error(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 18, 'is_signed_var': False, 'len': 1, 'name': 'bms_flag_infor_short', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Bmsflaginfor18c4e2ef::bms_flag_infor_short(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 17, 'is_signed_var': False, 'len': 1, 'name': 'bms_flag_infor_discharge_oc', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Bmsflaginfor18c4e2ef::bms_flag_infor_discharge_oc(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 16, 'is_signed_var': False, 'len': 1, 'name': 'bms_flag_infor_charge_oc', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Bmsflaginfor18c4e2ef::bms_flag_infor_charge_oc(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 15, 'is_signed_var': False, 'len': 1, 'name': 'bms_flag_infor_discharge_ut', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Bmsflaginfor18c4e2ef::bms_flag_infor_discharge_ut(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(7, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 14, 'is_signed_var': False, 'len': 1, 'name': 'bms_flag_infor_discharge_ot', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Bmsflaginfor18c4e2ef::bms_flag_infor_discharge_ot(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(6, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 13, 'is_signed_var': False, 'len': 1, 'name': 'bms_flag_infor_charge_ut', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Bmsflaginfor18c4e2ef::bms_flag_infor_charge_ut(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 12, 'is_signed_var': False, 'len': 1, 'name': 'bms_flag_infor_charge_ot', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Bmsflaginfor18c4e2ef::bms_flag_infor_charge_ot(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 11, 'is_signed_var': False, 'len': 1, 'name': 'bms_flag_infor_uv', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Bmsflaginfor18c4e2ef::bms_flag_infor_uv(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 10, 'is_signed_var': False, 'len': 1, 'name': 'bms_flag_infor_ov', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Bmsflaginfor18c4e2ef::bms_flag_infor_ov(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 9, 'is_signed_var': False, 'len': 1, 'name': 'bms_flag_infor_single_uv', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Bmsflaginfor18c4e2ef::bms_flag_infor_single_uv(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 8, 'is_signed_var': False, 'len': 1, 'name': 'bms_flag_infor_single_ov', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Bmsflaginfor18c4e2ef::bms_flag_infor_single_ov(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 8, 'name': 'bms_flag_infor_soc', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Bmsflaginfor18c4e2ef::bms_flag_infor_soc(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}
}  // namespace mk_mini
}  // namespace canbus
}  // namespace apollo
