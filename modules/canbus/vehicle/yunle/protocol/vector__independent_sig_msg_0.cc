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

#include "modules/canbus/vehicle/yunle/protocol/vector__independent_sig_msg_0.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace yunle {

using ::apollo::drivers::canbus::Byte;

Vectorindependentsigmsg0::Vectorindependentsigmsg0() {}
const int32_t Vectorindependentsigmsg0::ID = 0x0;

void Vectorindependentsigmsg0::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_yunle()->mutable_vector__independent_sig_msg_0()->set_rearfog_sts(rearfog_sts(bytes, length));
  chassis->mutable_yunle()->mutable_vector__independent_sig_msg_0()->set_horn_sts(horn_sts(bytes, length));
  chassis->mutable_yunle()->mutable_vector__independent_sig_msg_0()->set_highbeam_sts(highbeam_sts(bytes, length));
  chassis->mutable_yunle()->mutable_vector__independent_sig_msg_0()->set_hazard_light_sts(hazard_light_sts(bytes, length));
  chassis->mutable_yunle()->mutable_vector__independent_sig_msg_0()->set_sas_angle_speed_r(sas_angle_speed_r(bytes, length));
  chassis->mutable_yunle()->mutable_vector__independent_sig_msg_0()->set_sas_angle_speed_f(sas_angle_speed_f(bytes, length));
  chassis->mutable_yunle()->mutable_vector__independent_sig_msg_0()->set_gw_horn_req(gw_horn_req(bytes, length));
  chassis->mutable_yunle()->mutable_vector__independent_sig_msg_0()->set_gw_rearfoglight_req(gw_rearfoglight_req(bytes, length));
  chassis->mutable_yunle()->mutable_vector__independent_sig_msg_0()->set_gw_highbeam_req(gw_highbeam_req(bytes, length));
  chassis->mutable_yunle()->mutable_vector__independent_sig_msg_0()->set_gw_hazard_light_req(gw_hazard_light_req(bytes, length));
  chassis->mutable_yunle()->mutable_vector__independent_sig_msg_0()->set_gw_horn_req(gw_horn_req(bytes, length));
  chassis->mutable_yunle()->mutable_vector__independent_sig_msg_0()->set_gw_rearfoglight_req(gw_rearfoglight_req(bytes, length));
  chassis->mutable_yunle()->mutable_vector__independent_sig_msg_0()->set_gw_highbeam_req(gw_highbeam_req(bytes, length));
  chassis->mutable_yunle()->mutable_vector__independent_sig_msg_0()->set_gw_hazard_light_req(gw_hazard_light_req(bytes, length));
  chassis->mutable_yunle()->mutable_vector__independent_sig_msg_0()->set_gw_horn_req(gw_horn_req(bytes, length));
  chassis->mutable_yunle()->mutable_vector__independent_sig_msg_0()->set_gw_rearfoglight_req(gw_rearfoglight_req(bytes, length));
  chassis->mutable_yunle()->mutable_vector__independent_sig_msg_0()->set_gw_highbeam_req(gw_highbeam_req(bytes, length));
  chassis->mutable_yunle()->mutable_vector__independent_sig_msg_0()->set_gw_hazard_light_req(gw_hazard_light_req(bytes, length));
  chassis->mutable_yunle()->mutable_vector__independent_sig_msg_0()->set_scu_gearntorque_en_jd(scu_gearntorque_en_jd(bytes, length));
  chassis->mutable_yunle()->mutable_vector__independent_sig_msg_0()->set_scu_tq_or_speed(scu_tq_or_speed(bytes, length));
  chassis->mutable_yunle()->mutable_vector__independent_sig_msg_0()->set_scu_gearntorque_en(scu_gearntorque_en(bytes, length));
  chassis->mutable_yunle()->mutable_vector__independent_sig_msg_0()->set_scu_rmotor_spd_rpm(scu_rmotor_spd_rpm(bytes, length));
  chassis->mutable_yunle()->mutable_vector__independent_sig_msg_0()->set_scu_lmotor_spd_rpm(scu_lmotor_spd_rpm(bytes, length));
  chassis->mutable_yunle()->mutable_vector__independent_sig_msg_0()->set_gw_lowbeam_req(gw_lowbeam_req(bytes, length));
  chassis->mutable_yunle()->mutable_vector__independent_sig_msg_0()->set_gw_position_light_req(gw_position_light_req(bytes, length));
  chassis->mutable_yunle()->mutable_vector__independent_sig_msg_0()->set_gw_right_turn_light_req(gw_right_turn_light_req(bytes, length));
  chassis->mutable_yunle()->mutable_vector__independent_sig_msg_0()->set_gw_left_turn_light_req(gw_left_turn_light_req(bytes, length));
  chassis->mutable_yunle()->mutable_vector__independent_sig_msg_0()->set_scu_brk_en(scu_brk_en(bytes, length));
  chassis->mutable_yunle()->mutable_vector__independent_sig_msg_0()->set_scu_shiftlevel_req(scu_shiftlevel_req(bytes, length));
  chassis->mutable_yunle()->mutable_vector__independent_sig_msg_0()->set_scu_drive_mode_req(scu_drive_mode_req(bytes, length));
  chassis->mutable_yunle()->mutable_vector__independent_sig_msg_0()->set_gw_lowbeam_req(gw_lowbeam_req(bytes, length));
  chassis->mutable_yunle()->mutable_vector__independent_sig_msg_0()->set_gw_position_light_req(gw_position_light_req(bytes, length));
  chassis->mutable_yunle()->mutable_vector__independent_sig_msg_0()->set_gw_right_turn_light_req(gw_right_turn_light_req(bytes, length));
  chassis->mutable_yunle()->mutable_vector__independent_sig_msg_0()->set_gw_left_turn_light_req(gw_left_turn_light_req(bytes, length));
  chassis->mutable_yunle()->mutable_vector__independent_sig_msg_0()->set_scu_brk_en(scu_brk_en(bytes, length));
  chassis->mutable_yunle()->mutable_vector__independent_sig_msg_0()->set_scu_target_speed(scu_target_speed(bytes, length));
  chassis->mutable_yunle()->mutable_vector__independent_sig_msg_0()->set_scu_steering_wheel_angle(scu_steering_wheel_angle(bytes, length));
  chassis->mutable_yunle()->mutable_vector__independent_sig_msg_0()->set_scu_shiftlevel_req(scu_shiftlevel_req(bytes, length));
  chassis->mutable_yunle()->mutable_vector__independent_sig_msg_0()->set_scu_drive_mode_req(scu_drive_mode_req(bytes, length));
}

// config detail: {'bit': 0, 'description': '后雾灯状态', 'is_signed_var': False, 'len': 1, 'name': 'rearfog_sts', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vectorindependentsigmsg0::rearfog_sts(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 0, 'description': '喇叭状态', 'is_signed_var': False, 'len': 1, 'name': 'horn_sts', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vectorindependentsigmsg0::horn_sts(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 0, 'description': '远光灯状态', 'is_signed_var': False, 'len': 1, 'name': 'highbeam_sts', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vectorindependentsigmsg0::highbeam_sts(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 0, 'description': '警示灯状态', 'is_signed_var': False, 'len': 1, 'name': 'hazard_light_sts', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vectorindependentsigmsg0::hazard_light_sts(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': True, 'len': 8, 'name': 'sas_angle_speed_r', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-3276.8|3276.7]', 'physical_unit': 'deg/s', 'precision': 0.1, 'type': 'double'}
double Vectorindependentsigmsg0::sas_angle_speed_r(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': True, 'len': 8, 'name': 'sas_angle_speed_f', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-3276.8|3276.7]', 'physical_unit': 'deg/s', 'precision': 0.1, 'type': 'double'}
double Vectorindependentsigmsg0::sas_angle_speed_f(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name': 'gw_horn_req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vectorindependentsigmsg0::gw_horn_req(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name': 'gw_rearfoglight_req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vectorindependentsigmsg0::gw_rearfoglight_req(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name': 'gw_highbeam_req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vectorindependentsigmsg0::gw_highbeam_req(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name': 'gw_hazard_light_req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vectorindependentsigmsg0::gw_hazard_light_req(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name': 'gw_horn_req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vectorindependentsigmsg0::gw_horn_req(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name': 'gw_rearfoglight_req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vectorindependentsigmsg0::gw_rearfoglight_req(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name': 'gw_highbeam_req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vectorindependentsigmsg0::gw_highbeam_req(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name': 'gw_hazard_light_req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vectorindependentsigmsg0::gw_hazard_light_req(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name': 'gw_horn_req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vectorindependentsigmsg0::gw_horn_req(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name': 'gw_rearfoglight_req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vectorindependentsigmsg0::gw_rearfoglight_req(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name': 'gw_highbeam_req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vectorindependentsigmsg0::gw_highbeam_req(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name': 'gw_hazard_light_req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vectorindependentsigmsg0::gw_hazard_light_req(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 1, 'name': 'scu_gearntorque_en_jd', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vectorindependentsigmsg0::scu_gearntorque_en_jd(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': True, 'len': 1, 'name': 'scu_tq_or_speed', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vectorindependentsigmsg0::scu_tq_or_speed(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  x <<= 31;
  x >>= 31;

  bool ret = x;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 1, 'name': 'scu_gearntorque_en', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vectorindependentsigmsg0::scu_gearntorque_en(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': True, 'len': 16, 'name': 'scu_rmotor_spd_rpm', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-400|400]', 'physical_unit': '', 'precision': 0.1, 'type': 'double'}
double Vectorindependentsigmsg0::scu_rmotor_spd_rpm(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 0);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': True, 'len': 16, 'name': 'scu_lmotor_spd_rpm', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-400|400]', 'physical_unit': '', 'precision': 0.1, 'type': 'double'}
double Vectorindependentsigmsg0::scu_lmotor_spd_rpm(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 0);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name': 'gw_lowbeam_req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vectorindependentsigmsg0::gw_lowbeam_req(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name': 'gw_position_light_req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vectorindependentsigmsg0::gw_position_light_req(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name': 'gw_right_turn_light_req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vectorindependentsigmsg0::gw_right_turn_light_req(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name': 'gw_left_turn_light_req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vectorindependentsigmsg0::gw_left_turn_light_req(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 1, 'name': 'scu_brk_en', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vectorindependentsigmsg0::scu_brk_en(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name': 'scu_shiftlevel_req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vectorindependentsigmsg0::scu_shiftlevel_req(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name': 'scu_drive_mode_req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vectorindependentsigmsg0::scu_drive_mode_req(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name': 'gw_lowbeam_req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vectorindependentsigmsg0::gw_lowbeam_req(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name': 'gw_position_light_req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vectorindependentsigmsg0::gw_position_light_req(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name': 'gw_right_turn_light_req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vectorindependentsigmsg0::gw_right_turn_light_req(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name': 'gw_left_turn_light_req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vectorindependentsigmsg0::gw_left_turn_light_req(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 1, 'name': 'scu_brk_en', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vectorindependentsigmsg0::scu_brk_en(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 9, 'name': 'scu_target_speed', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|51]', 'physical_unit': 'km/h', 'precision': 0.1, 'type': 'double'}
double Vectorindependentsigmsg0::scu_target_speed(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 1);

  Byte t1(bytes + 0);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': True, 'len': 16, 'name': 'scu_steering_wheel_angle', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-120|120]', 'physical_unit': '', 'precision': 0.1, 'type': 'double'}
double Vectorindependentsigmsg0::scu_steering_wheel_angle(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 0);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name': 'scu_shiftlevel_req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vectorindependentsigmsg0::scu_shiftlevel_req(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name': 'scu_drive_mode_req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vectorindependentsigmsg0::scu_drive_mode_req(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}
}  // namespace yunle
}  // namespace canbus
}  // namespace apollo
