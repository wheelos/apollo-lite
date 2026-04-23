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

class Ctrlfb18c4d2ef : public ::apollo::drivers::canbus::ProtocolData<
                           ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Ctrlfb18c4d2ef();

  uint32_t GetPeriod() const override;

  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'bit': 47, 'is_signed_var': False, 'len': 1, 'name':
  // 'ctrl_fb_RemoteSt', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool ctrl_fb_remotest(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 56, 'is_signed_var': False, 'len': 8, 'name':
  // 'ctrl_fb_check_bcc', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int ctrl_fb_check_bcc(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 52, 'is_signed_var': False, 'len': 4, 'name':
  // 'ctrl_fb_alive_cnt', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int ctrl_fb_alive_cnt(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 44, 'is_signed_var': False, 'len': 2, 'name':
  // 'ctrl_fb_mode', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int ctrl_fb_mode(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 36, 'is_signed_var': False, 'len': 8, 'name':
  // 'ctrl_fb_Brake', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int ctrl_fb_brake(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 20, 'is_signed_var': True, 'len': 16, 'name':
  // 'ctrl_fb_steering', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 0.01, 'type': 'double'}
  double ctrl_fb_steering(const std::uint8_t* bytes,
                          const int32_t length) const;

  // config detail: {'bit': 4, 'is_signed_var': False, 'len': 16, 'name':
  // 'ctrl_fb_velocity', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 0.001, 'type': 'double'}
  double ctrl_fb_velocity(const std::uint8_t* bytes,
                          const int32_t length) const;

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 4, 'name':
  // 'ctrl_fb_gear', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int ctrl_fb_gear(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace mk_mini
}  // namespace canbus
}  // namespace apollo
