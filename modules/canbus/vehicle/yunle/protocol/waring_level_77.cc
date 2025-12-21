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

#include "modules/canbus/vehicle/yunle/protocol/waring_level_77.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace yunle {

using ::apollo::drivers::canbus::Byte;

Waringlevel77::Waringlevel77() {}
const int32_t Waringlevel77::ID = 0x77;

uint32_t Waringlevel77::GetPeriod() const {
  static const uint32_t PERIOD = 10 * 1000;
  return PERIOD;
}

void Waringlevel77::Parse(const std::uint8_t* bytes, int32_t length,
                          ChassisDetail* chassis) const {
  chassis->mutable_yunle()->mutable_waring_level_77()->set_mcu_vol_warning(
      mcu_vol_warning(bytes, length));
  chassis->mutable_yunle()
      ->mutable_waring_level_77()
      ->set_turn_unstoppable_warning(turn_unstoppable_warning(bytes, length));
  chassis->mutable_yunle()->mutable_waring_level_77()->set_turn_lock_warning(
      turn_lock_warning(bytes, length));
  chassis->mutable_yunle()
      ->mutable_waring_level_77()
      ->set_turn_disconnect_warning(turn_disconnect_warning(bytes, length));
  chassis->mutable_yunle()
      ->mutable_waring_level_77()
      ->set_mcu_temperature_warning(mcu_temperature_warning(bytes, length));
  chassis->mutable_yunle()->mutable_waring_level_77()->set_mcu_speed_warning(
      mcu_speed_warning(bytes, length));
  chassis->mutable_yunle()->mutable_waring_level_77()->set_mcu_motor_warning(
      mcu_motor_warning(bytes, length));
  chassis->mutable_yunle()
      ->mutable_waring_level_77()
      ->set_mcu_disconnect_warning(mcu_disconnect_warning(bytes, length));
  chassis->mutable_yunle()->mutable_waring_level_77()->set_mcu_cur_warning(
      mcu_cur_warning(bytes, length));
  chassis->mutable_yunle()
      ->mutable_waring_level_77()
      ->set_bms_temperature_warning(bms_temperature_warning(bytes, length));
  chassis->mutable_yunle()->mutable_waring_level_77()->set_bms_soc_warning(
      bms_soc_warning(bytes, length));
  chassis->mutable_yunle()
      ->mutable_waring_level_77()
      ->set_bms_dischargecur_warning(bms_dischargecur_warning(bytes, length));
  chassis->mutable_yunle()
      ->mutable_waring_level_77()
      ->set_bms_chargecur_warning(bms_chargecur_warning(bytes, length));
}

// config detail: {'bit': 27, 'is_signed_var': True, 'len': 3, 'name':
// 'mcu_vol_warning', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Waringlevel77::mcu_vol_warning(const std::uint8_t* bytes,
                                   int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(3, 3);

  x <<= 29;
  x >>= 29;

  int ret = x;
  return ret;
}

// config detail: {'bit': 36, 'is_signed_var': True, 'len': 3, 'name':
// 'turn_unstoppable_warning', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'int'}
int Waringlevel77::turn_unstoppable_warning(const std::uint8_t* bytes,
                                            int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(4, 3);

  x <<= 29;
  x >>= 29;

  int ret = x;
  return ret;
}

// config detail: {'bit': 33, 'is_signed_var': True, 'len': 3, 'name':
// 'turn_lock_warning', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Waringlevel77::turn_lock_warning(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(1, 3);

  x <<= 29;
  x >>= 29;

  int ret = x;
  return ret;
}

// config detail: {'bit': 30, 'is_signed_var': True, 'len': 3, 'name':
// 'turn_disconnect_warning', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Waringlevel77::turn_disconnect_warning(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 1);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(6, 2);
  x <<= 2;
  x |= t;

  x <<= 29;
  x >>= 29;

  int ret = x;
  return ret;
}

// config detail: {'bit': 24, 'is_signed_var': True, 'len': 3, 'name':
// 'mcu_temperature_warning', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Waringlevel77::mcu_temperature_warning(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 3);

  x <<= 29;
  x >>= 29;

  int ret = x;
  return ret;
}

// config detail: {'bit': 21, 'is_signed_var': True, 'len': 3, 'name':
// 'mcu_speed_warning', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Waringlevel77::mcu_speed_warning(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(5, 3);

  x <<= 29;
  x >>= 29;

  int ret = x;
  return ret;
}

// config detail: {'bit': 18, 'is_signed_var': True, 'len': 3, 'name':
// 'mcu_motor_warning', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Waringlevel77::mcu_motor_warning(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(2, 3);

  x <<= 29;
  x >>= 29;

  int ret = x;
  return ret;
}

// config detail: {'bit': 15, 'is_signed_var': True, 'len': 3, 'name':
// 'mcu_disconnect_warning', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Waringlevel77::mcu_disconnect_warning(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 2);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(7, 1);
  x <<= 1;
  x |= t;

  x <<= 29;
  x >>= 29;

  int ret = x;
  return ret;
}

// config detail: {'bit': 12, 'is_signed_var': True, 'len': 3, 'name':
// 'mcu_cur_warning', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Waringlevel77::mcu_cur_warning(const std::uint8_t* bytes,
                                   int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(4, 3);

  x <<= 29;
  x >>= 29;

  int ret = x;
  return ret;
}

// config detail: {'bit': 9, 'is_signed_var': True, 'len': 3, 'name':
// 'bms_temperature_warning', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Waringlevel77::bms_temperature_warning(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(1, 3);

  x <<= 29;
  x >>= 29;

  int ret = x;
  return ret;
}

// config detail: {'bit': 6, 'is_signed_var': True, 'len': 3, 'name':
// 'bms_soc_warning', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Waringlevel77::bms_soc_warning(const std::uint8_t* bytes,
                                   int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 1);

  Byte t1(bytes + 0);
  int32_t t = t1.get_byte(6, 2);
  x <<= 2;
  x |= t;

  x <<= 29;
  x >>= 29;

  int ret = x;
  return ret;
}

// config detail: {'bit': 3, 'is_signed_var': True, 'len': 3, 'name':
// 'bms_dischargecur_warning', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'int'}
int Waringlevel77::bms_dischargecur_warning(const std::uint8_t* bytes,
                                            int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(3, 3);

  x <<= 29;
  x >>= 29;

  int ret = x;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': True, 'len': 3, 'name':
// 'bms_chargecur_warning', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Waringlevel77::bms_chargecur_warning(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 3);

  x <<= 29;
  x >>= 29;

  int ret = x;
  return ret;
}
}  // namespace yunle
}  // namespace canbus
}  // namespace apollo
