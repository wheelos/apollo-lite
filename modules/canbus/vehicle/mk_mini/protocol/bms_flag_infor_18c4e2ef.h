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
namespace mk_mini {

class Bmsflaginfor18c4e2ef : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Bmsflaginfor18c4e2ef();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'bit': 24, 'is_signed_var': False, 'len': 1, 'name': 'bms_flag_Infor_SOCLowProtection', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bms_flag_infor_soclowprotection(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 56, 'is_signed_var': False, 'len': 8, 'name': 'bms_flag_Infor_check_bcc', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int bms_flag_infor_check_bcc(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 52, 'is_signed_var': False, 'len': 4, 'name': 'bms_flag_Infor_alive_cnt', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int bms_flag_infor_alive_cnt(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 23, 'is_signed_var': False, 'len': 1, 'name': 'bms_flag_Infor_SOCWarning', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bms_flag_infor_socwarning(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 40, 'is_signed_var': True, 'len': 12, 'name': 'bms_flag_Infor_low_temperature', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 0.1, 'type': 'double'}
  double bms_flag_infor_low_temperature(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 28, 'is_signed_var': True, 'len': 12, 'name': 'bms_flag_Infor_hight_temperature', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 0.1, 'type': 'double'}
  double bms_flag_infor_hight_temperature(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 21, 'is_signed_var': False, 'len': 2, 'name': 'bms_flag_Infor_charge_st', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int bms_flag_infor_charge_st(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 20, 'is_signed_var': False, 'len': 1, 'name': 'bms_flag_Infor_lock_mos', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bms_flag_infor_lock_mos(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 19, 'is_signed_var': False, 'len': 1, 'name': 'bms_flag_Infor_ic_error', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bms_flag_infor_ic_error(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 18, 'is_signed_var': False, 'len': 1, 'name': 'bms_flag_Infor_short', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bms_flag_infor_short(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 17, 'is_signed_var': False, 'len': 1, 'name': 'bms_flag_Infor_discharge_oc', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bms_flag_infor_discharge_oc(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 16, 'is_signed_var': False, 'len': 1, 'name': 'bms_flag_Infor_charge_oc', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bms_flag_infor_charge_oc(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 15, 'is_signed_var': False, 'len': 1, 'name': 'bms_flag_Infor_discharge_ut', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bms_flag_infor_discharge_ut(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 14, 'is_signed_var': False, 'len': 1, 'name': 'bms_flag_Infor_discharge_ot', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bms_flag_infor_discharge_ot(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 13, 'is_signed_var': False, 'len': 1, 'name': 'bms_flag_Infor_charge_ut', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bms_flag_infor_charge_ut(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 12, 'is_signed_var': False, 'len': 1, 'name': 'bms_flag_Infor_charge_ot', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bms_flag_infor_charge_ot(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 11, 'is_signed_var': False, 'len': 1, 'name': 'bms_flag_Infor_uv', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bms_flag_infor_uv(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 10, 'is_signed_var': False, 'len': 1, 'name': 'bms_flag_Infor_ov', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bms_flag_infor_ov(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 9, 'is_signed_var': False, 'len': 1, 'name': 'bms_flag_Infor_single_uv', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bms_flag_infor_single_uv(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 8, 'is_signed_var': False, 'len': 1, 'name': 'bms_flag_Infor_single_ov', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bms_flag_infor_single_ov(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 8, 'name': 'bms_flag_Infor_soc', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int bms_flag_infor_soc(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace mk_mini
}  // namespace canbus
}  // namespace apollo


