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

#include "modules/common_msgs/chassis_msgs/chassis_detail.pb.h"

#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace mk_mini {

class Iofb18c4daef : public ::apollo::drivers::canbus::ProtocolData<
                         ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Iofb18c4daef();

  uint32_t GetPeriod() const override;

  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'bit': 44, 'is_signed_var': False, 'len': 1, 'name':
  // 'io_fb_ScramSt', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool io_fb_scramst(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 41, 'is_signed_var': False, 'len': 1, 'name':
  // 'io_fb_ChargeEn', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool io_fb_chargeen(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 40, 'is_signed_var': False, 'len': 1, 'name':
  // 'io_fb_disChargeflg', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool io_fb_dischargeflg(const std::uint8_t* bytes,
                          const int32_t length) const;

  // config detail: {'bit': 56, 'is_signed_var': False, 'len': 8, 'name':
  // 'io_fb_check_bcc', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int io_fb_check_bcc(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 52, 'is_signed_var': False, 'len': 4, 'name':
  // 'io_fb_alive_cnt', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int io_fb_alive_cnt(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 37, 'is_signed_var': False, 'len': 1, 'name':
  // 'io_fb_rr_drop_sensor', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool io_fb_rr_drop_sensor(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'bit': 36, 'is_signed_var': False, 'len': 1, 'name':
  // 'io_fb_rm_drop_sensor', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool io_fb_rm_drop_sensor(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'bit': 35, 'is_signed_var': False, 'len': 1, 'name':
  // 'io_fb_rl_drop_sensor', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool io_fb_rl_drop_sensor(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'bit': 34, 'is_signed_var': False, 'len': 1, 'name':
  // 'io_fb_fr_drop_sensor', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool io_fb_fr_drop_sensor(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'bit': 33, 'is_signed_var': False, 'len': 1, 'name':
  // 'io_fb_fm_drop_sensor', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool io_fb_fm_drop_sensor(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'bit': 32, 'is_signed_var': False, 'len': 1, 'name':
  // 'io_fb_fl_drop_sensor', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool io_fb_fl_drop_sensor(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'bit': 29, 'is_signed_var': False, 'len': 1, 'name':
  // 'io_fb_rr_impact_sensor', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'bool'}
  bool io_fb_rr_impact_sensor(const std::uint8_t* bytes,
                              const int32_t length) const;

  // config detail: {'bit': 28, 'is_signed_var': False, 'len': 1, 'name':
  // 'io_fb_rm_impact_sensor', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'bool'}
  bool io_fb_rm_impact_sensor(const std::uint8_t* bytes,
                              const int32_t length) const;

  // config detail: {'bit': 27, 'is_signed_var': False, 'len': 1, 'name':
  // 'io_fb_rl_impact_sensor', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'bool'}
  bool io_fb_rl_impact_sensor(const std::uint8_t* bytes,
                              const int32_t length) const;

  // config detail: {'bit': 26, 'is_signed_var': False, 'len': 1, 'name':
  // 'io_fb_fr_impact_sensor', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'bool'}
  bool io_fb_fr_impact_sensor(const std::uint8_t* bytes,
                              const int32_t length) const;

  // config detail: {'bit': 25, 'is_signed_var': False, 'len': 1, 'name':
  // 'io_fb_fm_impact_sensor', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'bool'}
  bool io_fb_fm_impact_sensor(const std::uint8_t* bytes,
                              const int32_t length) const;

  // config detail: {'bit': 24, 'is_signed_var': False, 'len': 1, 'name':
  // 'io_fb_fl_impact_sensor', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'bool'}
  bool io_fb_fl_impact_sensor(const std::uint8_t* bytes,
                              const int32_t length) const;

  // config detail: {'bit': 16, 'is_signed_var': False, 'len': 1, 'name':
  // 'io_fb_speaker', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool io_fb_speaker(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 14, 'is_signed_var': False, 'len': 1, 'name':
  // 'io_fb_fog_lamp', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool io_fb_fog_lamp(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 13, 'is_signed_var': False, 'len': 1, 'name':
  // 'io_fb_clearance_lamp', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool io_fb_clearance_lamp(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'bit': 12, 'is_signed_var': False, 'len': 1, 'name':
  // 'io_fb_braking_lamp', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool io_fb_braking_lamp(const std::uint8_t* bytes,
                          const int32_t length) const;

  // config detail: {'bit': 10, 'is_signed_var': False, 'len': 2, 'name':
  // 'io_fb_turn_lamp', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int io_fb_turn_lamp(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 9, 'is_signed_var': False, 'len': 1, 'name':
  // 'io_fb_upper_beam_headlamp', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'bool'}
  bool io_fb_upper_beam_headlamp(const std::uint8_t* bytes,
                                 const int32_t length) const;

  // config detail: {'bit': 8, 'is_signed_var': False, 'len': 1, 'name':
  // 'io_fb_lower_beam_headlamp', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'bool'}
  bool io_fb_lower_beam_headlamp(const std::uint8_t* bytes,
                                 const int32_t length) const;

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 1, 'name':
  // 'io_fb_enable', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool io_fb_enable(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace mk_mini
}  // namespace canbus
}  // namespace apollo
