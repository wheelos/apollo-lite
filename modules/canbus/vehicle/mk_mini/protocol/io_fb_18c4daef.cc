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

#include "modules/canbus/vehicle/mk_mini/protocol/io_fb_18c4daef.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace mk_mini {

using ::apollo::drivers::canbus::Byte;

Iofb18c4daef::Iofb18c4daef() {}
const int32_t Iofb18c4daef::ID = 0x98c4daef;

uint32_t Iofb18c4daef::GetPeriod() const {
  static const uint32_t PERIOD = 50000;  // 50ms
  return PERIOD;
}

void Iofb18c4daef::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_mk_mini()->mutable_io_fb_18c4daef()->set_io_fb_scramst(
      io_fb_scramst(bytes, length));
  chassis->mutable_mk_mini()->mutable_io_fb_18c4daef()->set_io_fb_chargeen(
      io_fb_chargeen(bytes, length));
  chassis->mutable_mk_mini()->mutable_io_fb_18c4daef()->set_io_fb_dischargeflg(
      io_fb_dischargeflg(bytes, length));
  chassis->mutable_mk_mini()->mutable_io_fb_18c4daef()->set_io_fb_check_bcc(
      io_fb_check_bcc(bytes, length));
  chassis->mutable_mk_mini()->mutable_io_fb_18c4daef()->set_io_fb_alive_cnt(
      io_fb_alive_cnt(bytes, length));
  chassis->mutable_mk_mini()
      ->mutable_io_fb_18c4daef()
      ->set_io_fb_rr_drop_sensor(io_fb_rr_drop_sensor(bytes, length));
  chassis->mutable_mk_mini()
      ->mutable_io_fb_18c4daef()
      ->set_io_fb_rm_drop_sensor(io_fb_rm_drop_sensor(bytes, length));
  chassis->mutable_mk_mini()
      ->mutable_io_fb_18c4daef()
      ->set_io_fb_rl_drop_sensor(io_fb_rl_drop_sensor(bytes, length));
  chassis->mutable_mk_mini()
      ->mutable_io_fb_18c4daef()
      ->set_io_fb_fr_drop_sensor(io_fb_fr_drop_sensor(bytes, length));
  chassis->mutable_mk_mini()
      ->mutable_io_fb_18c4daef()
      ->set_io_fb_fm_drop_sensor(io_fb_fm_drop_sensor(bytes, length));
  chassis->mutable_mk_mini()
      ->mutable_io_fb_18c4daef()
      ->set_io_fb_fl_drop_sensor(io_fb_fl_drop_sensor(bytes, length));
  chassis->mutable_mk_mini()
      ->mutable_io_fb_18c4daef()
      ->set_io_fb_rr_impact_sensor(io_fb_rr_impact_sensor(bytes, length));
  chassis->mutable_mk_mini()
      ->mutable_io_fb_18c4daef()
      ->set_io_fb_rm_impact_sensor(io_fb_rm_impact_sensor(bytes, length));
  chassis->mutable_mk_mini()
      ->mutable_io_fb_18c4daef()
      ->set_io_fb_rl_impact_sensor(io_fb_rl_impact_sensor(bytes, length));
  chassis->mutable_mk_mini()
      ->mutable_io_fb_18c4daef()
      ->set_io_fb_fr_impact_sensor(io_fb_fr_impact_sensor(bytes, length));
  chassis->mutable_mk_mini()
      ->mutable_io_fb_18c4daef()
      ->set_io_fb_fm_impact_sensor(io_fb_fm_impact_sensor(bytes, length));
  chassis->mutable_mk_mini()
      ->mutable_io_fb_18c4daef()
      ->set_io_fb_fl_impact_sensor(io_fb_fl_impact_sensor(bytes, length));
  chassis->mutable_mk_mini()->mutable_io_fb_18c4daef()->set_io_fb_speaker(
      io_fb_speaker(bytes, length));
  chassis->mutable_mk_mini()->mutable_io_fb_18c4daef()->set_io_fb_fog_lamp(
      io_fb_fog_lamp(bytes, length));
  chassis->mutable_mk_mini()
      ->mutable_io_fb_18c4daef()
      ->set_io_fb_clearance_lamp(io_fb_clearance_lamp(bytes, length));
  chassis->mutable_mk_mini()->mutable_io_fb_18c4daef()->set_io_fb_braking_lamp(
      io_fb_braking_lamp(bytes, length));
  chassis->mutable_mk_mini()->mutable_io_fb_18c4daef()->set_io_fb_turn_lamp(
      io_fb_turn_lamp(bytes, length));
  chassis->mutable_mk_mini()
      ->mutable_io_fb_18c4daef()
      ->set_io_fb_upper_beam_headlamp(io_fb_upper_beam_headlamp(bytes, length));
  chassis->mutable_mk_mini()
      ->mutable_io_fb_18c4daef()
      ->set_io_fb_lower_beam_headlamp(io_fb_lower_beam_headlamp(bytes, length));
  chassis->mutable_mk_mini()->mutable_io_fb_18c4daef()->set_io_fb_enable(
      io_fb_enable(bytes, length));
}

// config detail: {'bit': 44, 'is_signed_var': False, 'len': 1, 'name':
// 'io_fb_scramst', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Iofb18c4daef::io_fb_scramst(const std::uint8_t* bytes,
                                 int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 41, 'is_signed_var': False, 'len': 1, 'name':
// 'io_fb_chargeen', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Iofb18c4daef::io_fb_chargeen(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 40, 'is_signed_var': False, 'len': 1, 'name':
// 'io_fb_dischargeflg', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Iofb18c4daef::io_fb_dischargeflg(const std::uint8_t* bytes,
                                      int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 56, 'is_signed_var': False, 'len': 8, 'name':
// 'io_fb_check_bcc', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Iofb18c4daef::io_fb_check_bcc(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 52, 'is_signed_var': False, 'len': 4, 'name':
// 'io_fb_alive_cnt', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Iofb18c4daef::io_fb_alive_cnt(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(4, 4);

  int ret = x;
  return ret;
}

// config detail: {'bit': 37, 'is_signed_var': False, 'len': 1, 'name':
// 'io_fb_rr_drop_sensor', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Iofb18c4daef::io_fb_rr_drop_sensor(const std::uint8_t* bytes,
                                        int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 36, 'is_signed_var': False, 'len': 1, 'name':
// 'io_fb_rm_drop_sensor', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Iofb18c4daef::io_fb_rm_drop_sensor(const std::uint8_t* bytes,
                                        int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 35, 'is_signed_var': False, 'len': 1, 'name':
// 'io_fb_rl_drop_sensor', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Iofb18c4daef::io_fb_rl_drop_sensor(const std::uint8_t* bytes,
                                        int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 34, 'is_signed_var': False, 'len': 1, 'name':
// 'io_fb_fr_drop_sensor', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Iofb18c4daef::io_fb_fr_drop_sensor(const std::uint8_t* bytes,
                                        int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 33, 'is_signed_var': False, 'len': 1, 'name':
// 'io_fb_fm_drop_sensor', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Iofb18c4daef::io_fb_fm_drop_sensor(const std::uint8_t* bytes,
                                        int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 32, 'is_signed_var': False, 'len': 1, 'name':
// 'io_fb_fl_drop_sensor', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Iofb18c4daef::io_fb_fl_drop_sensor(const std::uint8_t* bytes,
                                        int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 29, 'is_signed_var': False, 'len': 1, 'name':
// 'io_fb_rr_impact_sensor', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Iofb18c4daef::io_fb_rr_impact_sensor(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 28, 'is_signed_var': False, 'len': 1, 'name':
// 'io_fb_rm_impact_sensor', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Iofb18c4daef::io_fb_rm_impact_sensor(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 27, 'is_signed_var': False, 'len': 1, 'name':
// 'io_fb_rl_impact_sensor', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Iofb18c4daef::io_fb_rl_impact_sensor(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 26, 'is_signed_var': False, 'len': 1, 'name':
// 'io_fb_fr_impact_sensor', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Iofb18c4daef::io_fb_fr_impact_sensor(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 25, 'is_signed_var': False, 'len': 1, 'name':
// 'io_fb_fm_impact_sensor', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Iofb18c4daef::io_fb_fm_impact_sensor(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 24, 'is_signed_var': False, 'len': 1, 'name':
// 'io_fb_fl_impact_sensor', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Iofb18c4daef::io_fb_fl_impact_sensor(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 16, 'is_signed_var': False, 'len': 1, 'name':
// 'io_fb_speaker', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Iofb18c4daef::io_fb_speaker(const std::uint8_t* bytes,
                                 int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 14, 'is_signed_var': False, 'len': 1, 'name':
// 'io_fb_fog_lamp', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Iofb18c4daef::io_fb_fog_lamp(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(6, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 13, 'is_signed_var': False, 'len': 1, 'name':
// 'io_fb_clearance_lamp', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Iofb18c4daef::io_fb_clearance_lamp(const std::uint8_t* bytes,
                                        int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 12, 'is_signed_var': False, 'len': 1, 'name':
// 'io_fb_braking_lamp', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Iofb18c4daef::io_fb_braking_lamp(const std::uint8_t* bytes,
                                      int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 10, 'is_signed_var': False, 'len': 2, 'name':
// 'io_fb_turn_lamp', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Iofb18c4daef::io_fb_turn_lamp(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(2, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 9, 'is_signed_var': False, 'len': 1, 'name':
// 'io_fb_upper_beam_headlamp', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'bool'}
bool Iofb18c4daef::io_fb_upper_beam_headlamp(const std::uint8_t* bytes,
                                             int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 8, 'is_signed_var': False, 'len': 1, 'name':
// 'io_fb_lower_beam_headlamp', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'bool'}
bool Iofb18c4daef::io_fb_lower_beam_headlamp(const std::uint8_t* bytes,
                                             int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 1, 'name':
// 'io_fb_enable', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Iofb18c4daef::io_fb_enable(const std::uint8_t* bytes,
                                int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}
}  // namespace mk_mini
}  // namespace canbus
}  // namespace apollo
