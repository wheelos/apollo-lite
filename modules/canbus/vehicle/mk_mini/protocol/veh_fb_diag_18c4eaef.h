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

class Vehfbdiag18c4eaef : public ::apollo::drivers::canbus::ProtocolData<
                              ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Vehfbdiag18c4eaef();

  uint32_t GetPeriod() const override;

  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'bit': 38, 'is_signed_var': False, 'len': 6, 'name':
  // 'Veh_fb_RDrvMCUFault', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int veh_fb_rdrvmcufault(const std::uint8_t* bytes,
                          const int32_t length) const;

  // config detail: {'bit': 32, 'is_signed_var': False, 'len': 6, 'name':
  // 'Veh_fb_LDrvMCUFault', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int veh_fb_ldrvmcufault(const std::uint8_t* bytes,
                          const int32_t length) const;

  // config detail: {'bit': 30, 'is_signed_var': False, 'len': 1, 'name':
  // 'Veh_fb_EHBOilFault', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool veh_fb_ehboilfault(const std::uint8_t* bytes,
                          const int32_t length) const;

  // config detail: {'bit': 29, 'is_signed_var': False, 'len': 1, 'name':
  // 'Veh_fb_EHBOilPressSensorFault', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'bool'}
  bool veh_fb_ehboilpresssensorfault(const std::uint8_t* bytes,
                                     const int32_t length) const;

  // config detail: {'bit': 28, 'is_signed_var': False, 'len': 1, 'name':
  // 'Veh_fb_EHBMotorFault', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool veh_fb_ehbmotorfault(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'bit': 27, 'is_signed_var': False, 'len': 1, 'name':
  // 'Veh_fb_EHBsensorAbnomal', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'bool'}
  bool veh_fb_ehbsensorabnomal(const std::uint8_t* bytes,
                               const int32_t length) const;

  // config detail: {'bit': 26, 'is_signed_var': False, 'len': 1, 'name':
  // 'Veh_fb_EHBPowerFault', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool veh_fb_ehbpowerfault(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'bit': 25, 'is_signed_var': False, 'len': 1, 'name':
  // 'Veh_fb_EHBOT', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool veh_fb_ehbot(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 24, 'is_signed_var': False, 'len': 1, 'name':
  // 'Veh_fb_EHBAnguleFault', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool veh_fb_ehbangulefault(const std::uint8_t* bytes,
                             const int32_t length) const;

  // config detail: {'bit': 23, 'is_signed_var': False, 'len': 1, 'name':
  // 'Veh_fb_EHBDisEn', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool veh_fb_ehbdisen(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 22, 'is_signed_var': False, 'len': 1, 'name':
  // 'Veh_fb_EHBWorkModelFault', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'bool'}
  bool veh_fb_ehbworkmodelfault(const std::uint8_t* bytes,
                                const int32_t length) const;

  // config detail: {'bit': 13, 'is_signed_var': False, 'len': 1, 'name':
  // 'Veh_fb_EPSOverCurrent', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool veh_fb_epsovercurrent(const std::uint8_t* bytes,
                             const int32_t length) const;

  // config detail: {'bit': 12, 'is_signed_var': False, 'len': 1, 'name':
  // 'Veh_fb_EPSDisWork', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool veh_fb_epsdiswork(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 11, 'is_signed_var': False, 'len': 1, 'name':
  // 'Veh_fb_EPSWarning', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool veh_fb_epswarning(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 47, 'is_signed_var': False, 'len': 1, 'name':
  // 'Veh_fb_AuxRemoteDisOnline', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'bool'}
  bool veh_fb_auxremotedisonline(const std::uint8_t* bytes,
                                 const int32_t length) const;

  // config detail: {'bit': 46, 'is_signed_var': False, 'len': 1, 'name':
  // 'Veh_fb_AuxRemoteClose', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool veh_fb_auxremoteclose(const std::uint8_t* bytes,
                             const int32_t length) const;

  // config detail: {'bit': 44, 'is_signed_var': False, 'len': 1, 'name':
  // 'Veh_fb_AUXBMSDisOnline', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'bool'}
  bool veh_fb_auxbmsdisonline(const std::uint8_t* bytes,
                              const int32_t length) const;

  // config detail: {'bit': 5, 'is_signed_var': False, 'len': 1, 'name':
  // 'Veh_fb_AutoIOCANCmd', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool veh_fb_autoiocancmd(const std::uint8_t* bytes,
                           const int32_t length) const;

  // config detail: {'bit': 4, 'is_signed_var': False, 'len': 1, 'name':
  // 'Veh_fb_AutoCANCtrlCmd', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool veh_fb_autocanctrlcmd(const std::uint8_t* bytes,
                             const int32_t length) const;

  // config detail: {'bit': 56, 'is_signed_var': False, 'len': 8, 'name':
  // 'Veh_fb_Infor_check_bcc', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'int'}
  int veh_fb_infor_check_bcc(const std::uint8_t* bytes,
                             const int32_t length) const;

  // config detail: {'bit': 52, 'is_signed_var': False, 'len': 4, 'name':
  // 'Veh_fb_Infor_alive_cnt', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'int'}
  int veh_fb_infor_alive_cnt(const std::uint8_t* bytes,
                             const int32_t length) const;

  // config detail: {'bit': 45, 'is_signed_var': False, 'len': 1, 'name':
  // 'Veh_fb_AuxScram', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool veh_fb_auxscram(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 21, 'is_signed_var': False, 'len': 1, 'name':
  // 'Veh_fb_EHBDisOnline', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool veh_fb_ehbdisonline(const std::uint8_t* bytes,
                           const int32_t length) const;

  // config detail: {'bit': 20, 'is_signed_var': False, 'len': 1, 'name':
  // 'Veh_fb_EHBecuFault', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool veh_fb_ehbecufault(const std::uint8_t* bytes,
                          const int32_t length) const;

  // config detail: {'bit': 10, 'is_signed_var': False, 'len': 1, 'name':
  // 'Veh_fb_EPSMosfetOT', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool veh_fb_epsmosfetot(const std::uint8_t* bytes,
                          const int32_t length) const;

  // config detail: {'bit': 9, 'is_signed_var': False, 'len': 1, 'name':
  // 'Veh_fb_EPSfault', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool veh_fb_epsfault(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 8, 'is_signed_var': False, 'len': 1, 'name':
  // 'Veh_fb_EPSDisOnline', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool veh_fb_epsdisonline(const std::uint8_t* bytes,
                           const int32_t length) const;

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 4, 'name':
  // 'Veh_fb_FaultLevel', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int veh_fb_faultlevel(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace mk_mini
}  // namespace canbus
}  // namespace apollo
