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

#pragma once

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/common_msgs/chassis_msgs/chassis_detail.pb.h"

namespace apollo {
namespace canbus {
namespace yunle {

class Vectorindependentsigmsg0 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Vectorindependentsigmsg0();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'bit': 0, 'description': '后雾灯状态', 'is_signed_var': False, 'len': 1, 'name': 'RearFog_Sts', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool rearfog_sts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 0, 'description': '喇叭状态', 'is_signed_var': False, 'len': 1, 'name': 'Horn_Sts', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool horn_sts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 0, 'description': '远光灯状态', 'is_signed_var': False, 'len': 1, 'name': 'HighBeam_Sts', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool highbeam_sts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 0, 'description': '警示灯状态', 'is_signed_var': False, 'len': 1, 'name': 'Hazard_Light_Sts', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool hazard_light_sts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 0, 'is_signed_var': True, 'len': 8, 'name': 'SAS_Angle_Speed_R', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-3276.8|3276.7]', 'physical_unit': 'deg/s', 'precision': 0.1, 'type': 'double'}
  double sas_angle_speed_r(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 0, 'is_signed_var': True, 'len': 8, 'name': 'SAS_Angle_Speed_F', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-3276.8|3276.7]', 'physical_unit': 'deg/s', 'precision': 0.1, 'type': 'double'}
  double sas_angle_speed_f(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name': 'GW_Horn_Req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int gw_horn_req(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name': 'GW_RearFogLight_Req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int gw_rearfoglight_req(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name': 'GW_HighBeam_Req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int gw_highbeam_req(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name': 'GW_Hazard_Light_Req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int gw_hazard_light_req(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name': 'GW_Horn_Req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int gw_horn_req(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name': 'GW_RearFogLight_Req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int gw_rearfoglight_req(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name': 'GW_HighBeam_Req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int gw_highbeam_req(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name': 'GW_Hazard_Light_Req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int gw_hazard_light_req(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name': 'GW_Horn_Req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int gw_horn_req(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name': 'GW_RearFogLight_Req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int gw_rearfoglight_req(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name': 'GW_HighBeam_Req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int gw_highbeam_req(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name': 'GW_Hazard_Light_Req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int gw_hazard_light_req(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 1, 'name': 'SCU_GearNTorque_En_JD', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool scu_gearntorque_en_jd(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 0, 'is_signed_var': True, 'len': 1, 'name': 'SCU_Tq_or_Speed', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool scu_tq_or_speed(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 1, 'name': 'SCU_GearNTorque_En', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool scu_gearntorque_en(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 0, 'is_signed_var': True, 'len': 16, 'name': 'SCU_RMotor_Spd_rpm', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-400|400]', 'physical_unit': '', 'precision': 0.1, 'type': 'double'}
  double scu_rmotor_spd_rpm(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 0, 'is_signed_var': True, 'len': 16, 'name': 'SCU_LMotor_Spd_rpm', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-400|400]', 'physical_unit': '', 'precision': 0.1, 'type': 'double'}
  double scu_lmotor_spd_rpm(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name': 'GW_LowBeam_Req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int gw_lowbeam_req(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name': 'GW_Position_Light_Req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int gw_position_light_req(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name': 'GW_Right_Turn_Light_Req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int gw_right_turn_light_req(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name': 'GW_Left_Turn_Light_Req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int gw_left_turn_light_req(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 1, 'name': 'SCU_Brk_En', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool scu_brk_en(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name': 'SCU_ShiftLevel_Req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int scu_shiftlevel_req(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name': 'SCU_Drive_Mode_Req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int scu_drive_mode_req(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name': 'GW_LowBeam_Req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int gw_lowbeam_req(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name': 'GW_Position_Light_Req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int gw_position_light_req(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name': 'GW_Right_Turn_Light_Req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int gw_right_turn_light_req(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name': 'GW_Left_Turn_Light_Req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int gw_left_turn_light_req(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 1, 'name': 'SCU_Brk_En', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool scu_brk_en(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 9, 'name': 'SCU_Target_Speed', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|51]', 'physical_unit': 'km/h', 'precision': 0.1, 'type': 'double'}
  double scu_target_speed(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 0, 'is_signed_var': True, 'len': 16, 'name': 'SCU_Steering_Wheel_Angle', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-120|120]', 'physical_unit': '', 'precision': 0.1, 'type': 'double'}
  double scu_steering_wheel_angle(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name': 'SCU_ShiftLevel_Req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int scu_shiftlevel_req(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name': 'SCU_Drive_Mode_Req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int scu_drive_mode_req(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace yunle
}  // namespace canbus
}  // namespace apollo


