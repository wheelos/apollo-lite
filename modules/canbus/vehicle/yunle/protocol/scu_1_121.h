/******************************************************************************
 * Copyright 2025 The WheelOS Team. All Rights Reserved.
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

#include "modules/common_msgs/chassis_msgs/chassis_detail.pb.h"

#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace yunle {

class Scu1121 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Scu1121();

  uint32_t GetPeriod() const override;

  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'bit': 60, 'description': '只针对个别车型使用，不适用所有车型', 'is_signed_var': True, 'len': 1, 'name': 'angular_speed_flag', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  Scu1121* set_angular_speed_flag(bool angular_speed_flag);

  // config detail: {'bit': 16, 'is_signed_var': False, 'len': 8, 'name': 'SCU_Steering_Wheel_Angle_R', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Scu1121* set_scu_steering_wheel_angle_r(int scu_steering_wheel_angle_r);

  // config detail: {'bit': 8, 'is_signed_var': False, 'len': 8, 'name': 'SCU_Steering_Wheel_Angle_F', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Scu1121* set_scu_steering_wheel_angle_f(int scu_steering_wheel_angle_f);

  // config detail: {'bit': 48, 'is_signed_var': False, 'len': 2, 'name': 'GW_LowBeam_Req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Scu1121* set_gw_lowbeam_req(int gw_lowbeam_req);

  // config detail: {'bit': 46, 'is_signed_var': False, 'len': 2, 'name': 'GW_Position_Light_Req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Scu1121* set_gw_position_light_req(int gw_position_light_req);

  // config detail: {'bit': 42, 'is_signed_var': False, 'len': 2, 'name': 'GW_Right_Turn_Light_Req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Scu1121* set_gw_right_turn_light_req(int gw_right_turn_light_req);

  // config detail: {'bit': 40, 'is_signed_var': False, 'len': 2, 'name': 'GW_Left_Turn_Light_Req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Scu1121* set_gw_left_turn_light_req(int gw_left_turn_light_req);

  // config detail: {'bit': 33, 'is_signed_var': False, 'len': 1, 'name': 'SCU_Brk_En', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  Scu1121* set_scu_brk_en(bool scu_brk_en);

  // config detail: {'bit': 24, 'is_signed_var': False, 'len': 9, 'name': 'SCU_Target_Speed', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|51]', 'physical_unit': 'km/h', 'precision': 0.1, 'type': 'double'}
  Scu1121* set_scu_target_speed(double scu_target_speed);

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name': 'SCU_ShiftLevel_Req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Scu1121* set_scu_shiftlevel_req(int scu_shiftlevel_req);

  // config detail: {'bit': 6, 'is_signed_var': False, 'len': 2, 'name': 'SCU_Drive_Mode_Req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Scu1121* set_scu_drive_mode_req(int scu_drive_mode_req);

 private:

  // config detail: {'bit': 60, 'description': '只针对个别车型使用，不适用所有车型', 'is_signed_var': True, 'len': 1, 'name': 'angular_speed_flag', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_angular_speed_flag(uint8_t* data, bool angular_speed_flag);

  // config detail: {'bit': 16, 'is_signed_var': False, 'len': 8, 'name': 'SCU_Steering_Wheel_Angle_R', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_scu_steering_wheel_angle_r(uint8_t* data, int scu_steering_wheel_angle_r);

  // config detail: {'bit': 8, 'is_signed_var': False, 'len': 8, 'name': 'SCU_Steering_Wheel_Angle_F', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_scu_steering_wheel_angle_f(uint8_t* data, int scu_steering_wheel_angle_f);

  // config detail: {'bit': 48, 'is_signed_var': False, 'len': 2, 'name': 'GW_LowBeam_Req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_gw_lowbeam_req(uint8_t* data, int gw_lowbeam_req);

  // config detail: {'bit': 46, 'is_signed_var': False, 'len': 2, 'name': 'GW_Position_Light_Req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_gw_position_light_req(uint8_t* data, int gw_position_light_req);

  // config detail: {'bit': 42, 'is_signed_var': False, 'len': 2, 'name': 'GW_Right_Turn_Light_Req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_gw_right_turn_light_req(uint8_t* data, int gw_right_turn_light_req);

  // config detail: {'bit': 40, 'is_signed_var': False, 'len': 2, 'name': 'GW_Left_Turn_Light_Req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_gw_left_turn_light_req(uint8_t* data, int gw_left_turn_light_req);

  // config detail: {'bit': 33, 'is_signed_var': False, 'len': 1, 'name': 'SCU_Brk_En', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_scu_brk_en(uint8_t* data, bool scu_brk_en);

  // config detail: {'bit': 24, 'is_signed_var': False, 'len': 9, 'name': 'SCU_Target_Speed', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|51]', 'physical_unit': 'km/h', 'precision': 0.1, 'type': 'double'}
  void set_p_scu_target_speed(uint8_t* data, double scu_target_speed);

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name': 'SCU_ShiftLevel_Req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_scu_shiftlevel_req(uint8_t* data, int scu_shiftlevel_req);

  // config detail: {'bit': 6, 'is_signed_var': False, 'len': 2, 'name': 'SCU_Drive_Mode_Req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_scu_drive_mode_req(uint8_t* data, int scu_drive_mode_req);

  bool angular_speed_flag(const std::uint8_t* bytes, const int32_t length) const;

  int scu_steering_wheel_angle_r(const std::uint8_t* bytes, const int32_t length) const;

  int scu_steering_wheel_angle_f(const std::uint8_t* bytes, const int32_t length) const;

  int gw_lowbeam_req(const std::uint8_t* bytes, const int32_t length) const;

  int gw_position_light_req(const std::uint8_t* bytes, const int32_t length) const;

  int gw_right_turn_light_req(const std::uint8_t* bytes, const int32_t length) const;

  int gw_left_turn_light_req(const std::uint8_t* bytes, const int32_t length) const;

  bool scu_brk_en(const std::uint8_t* bytes, const int32_t length) const;

  double scu_target_speed(const std::uint8_t* bytes, const int32_t length) const;

  int scu_shiftlevel_req(const std::uint8_t* bytes, const int32_t length) const;

  int scu_drive_mode_req(const std::uint8_t* bytes, const int32_t length) const;

 private:
  bool angular_speed_flag_;
  int scu_steering_wheel_angle_r_;
  int scu_steering_wheel_angle_f_;
  int gw_lowbeam_req_;
  int gw_position_light_req_;
  int gw_right_turn_light_req_;
  int gw_left_turn_light_req_;
  bool scu_brk_en_;
  double scu_target_speed_;
  int scu_shiftlevel_req_;
  int scu_drive_mode_req_;
};

}  // namespace yunle
}  // namespace canbus
}  // namespace apollo


