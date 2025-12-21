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

#include "modules/canbus/vehicle/mk_mini/protocol/veh_fb_diag_18c4eaef.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace mk_mini {

using ::apollo::drivers::canbus::Byte;

Vehfbdiag18c4eaef::Vehfbdiag18c4eaef() {}
const int32_t Vehfbdiag18c4eaef::ID = 0x98c4eaef;

uint32_t Vehfbdiag18c4eaef::GetPeriod() const {
  static const uint32_t PERIOD = 10000;  // 10ms
  return PERIOD;
}

void Vehfbdiag18c4eaef::Parse(const std::uint8_t* bytes, int32_t length,
                              ChassisDetail* chassis) const {
  chassis->mutable_check_response()->set_is_eps_online(
      !veh_fb_epsdiswork(bytes, length));

  chassis->mutable_check_response()->set_is_vcu_online(
      !veh_fb_autocanctrlcmd(bytes, length));

  chassis->mutable_mk_mini()
      ->mutable_veh_fb_diag_18c4eaef()
      ->set_veh_fb_rdrvmcufault(veh_fb_rdrvmcufault(bytes, length));
  chassis->mutable_mk_mini()
      ->mutable_veh_fb_diag_18c4eaef()
      ->set_veh_fb_ldrvmcufault(veh_fb_ldrvmcufault(bytes, length));
  chassis->mutable_mk_mini()
      ->mutable_veh_fb_diag_18c4eaef()
      ->set_veh_fb_ehboilfault(veh_fb_ehboilfault(bytes, length));
  chassis->mutable_mk_mini()
      ->mutable_veh_fb_diag_18c4eaef()
      ->set_veh_fb_ehboilpresssensorfault(
          veh_fb_ehboilpresssensorfault(bytes, length));
  chassis->mutable_mk_mini()
      ->mutable_veh_fb_diag_18c4eaef()
      ->set_veh_fb_ehbmotorfault(veh_fb_ehbmotorfault(bytes, length));
  chassis->mutable_mk_mini()
      ->mutable_veh_fb_diag_18c4eaef()
      ->set_veh_fb_ehbsensorabnomal(veh_fb_ehbsensorabnomal(bytes, length));
  chassis->mutable_mk_mini()
      ->mutable_veh_fb_diag_18c4eaef()
      ->set_veh_fb_ehbpowerfault(veh_fb_ehbpowerfault(bytes, length));
  chassis->mutable_mk_mini()->mutable_veh_fb_diag_18c4eaef()->set_veh_fb_ehbot(
      veh_fb_ehbot(bytes, length));
  chassis->mutable_mk_mini()
      ->mutable_veh_fb_diag_18c4eaef()
      ->set_veh_fb_ehbangulefault(veh_fb_ehbangulefault(bytes, length));
  chassis->mutable_mk_mini()
      ->mutable_veh_fb_diag_18c4eaef()
      ->set_veh_fb_ehbdisen(veh_fb_ehbdisen(bytes, length));
  chassis->mutable_mk_mini()
      ->mutable_veh_fb_diag_18c4eaef()
      ->set_veh_fb_ehbworkmodelfault(veh_fb_ehbworkmodelfault(bytes, length));
  chassis->mutable_mk_mini()
      ->mutable_veh_fb_diag_18c4eaef()
      ->set_veh_fb_epsovercurrent(veh_fb_epsovercurrent(bytes, length));
  chassis->mutable_mk_mini()
      ->mutable_veh_fb_diag_18c4eaef()
      ->set_veh_fb_epsdiswork(veh_fb_epsdiswork(bytes, length));
  chassis->mutable_mk_mini()
      ->mutable_veh_fb_diag_18c4eaef()
      ->set_veh_fb_epswarning(veh_fb_epswarning(bytes, length));
  chassis->mutable_mk_mini()
      ->mutable_veh_fb_diag_18c4eaef()
      ->set_veh_fb_auxremotedisonline(veh_fb_auxremotedisonline(bytes, length));
  chassis->mutable_mk_mini()
      ->mutable_veh_fb_diag_18c4eaef()
      ->set_veh_fb_auxremoteclose(veh_fb_auxremoteclose(bytes, length));
  chassis->mutable_mk_mini()
      ->mutable_veh_fb_diag_18c4eaef()
      ->set_veh_fb_auxbmsdisonline(veh_fb_auxbmsdisonline(bytes, length));
  chassis->mutable_mk_mini()
      ->mutable_veh_fb_diag_18c4eaef()
      ->set_veh_fb_autoiocancmd(veh_fb_autoiocancmd(bytes, length));
  chassis->mutable_mk_mini()
      ->mutable_veh_fb_diag_18c4eaef()
      ->set_veh_fb_autocanctrlcmd(veh_fb_autocanctrlcmd(bytes, length));
  chassis->mutable_mk_mini()
      ->mutable_veh_fb_diag_18c4eaef()
      ->set_veh_fb_infor_check_bcc(veh_fb_infor_check_bcc(bytes, length));
  chassis->mutable_mk_mini()
      ->mutable_veh_fb_diag_18c4eaef()
      ->set_veh_fb_infor_alive_cnt(veh_fb_infor_alive_cnt(bytes, length));
  chassis->mutable_mk_mini()
      ->mutable_veh_fb_diag_18c4eaef()
      ->set_veh_fb_auxscram(veh_fb_auxscram(bytes, length));
  chassis->mutable_mk_mini()
      ->mutable_veh_fb_diag_18c4eaef()
      ->set_veh_fb_ehbdisonline(veh_fb_ehbdisonline(bytes, length));
  chassis->mutable_mk_mini()
      ->mutable_veh_fb_diag_18c4eaef()
      ->set_veh_fb_ehbecufault(veh_fb_ehbecufault(bytes, length));
  chassis->mutable_mk_mini()
      ->mutable_veh_fb_diag_18c4eaef()
      ->set_veh_fb_epsmosfetot(veh_fb_epsmosfetot(bytes, length));
  chassis->mutable_mk_mini()
      ->mutable_veh_fb_diag_18c4eaef()
      ->set_veh_fb_epsfault(veh_fb_epsfault(bytes, length));
  chassis->mutable_mk_mini()
      ->mutable_veh_fb_diag_18c4eaef()
      ->set_veh_fb_epsdisonline(veh_fb_epsdisonline(bytes, length));
  chassis->mutable_mk_mini()
      ->mutable_veh_fb_diag_18c4eaef()
      ->set_veh_fb_faultlevel(veh_fb_faultlevel(bytes, length));
}

// config detail: {'bit': 38, 'is_signed_var': False, 'len': 6, 'name':
// 'veh_fb_rdrvmcufault', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vehfbdiag18c4eaef::veh_fb_rdrvmcufault(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 4);

  Byte t1(bytes + 4);
  int32_t t = t1.get_byte(6, 2);
  x <<= 2;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'bit': 32, 'is_signed_var': False, 'len': 6, 'name':
// 'veh_fb_ldrvmcufault', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vehfbdiag18c4eaef::veh_fb_ldrvmcufault(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 6);

  int ret = x;
  return ret;
}

// config detail: {'bit': 30, 'is_signed_var': False, 'len': 1, 'name':
// 'veh_fb_ehboilfault', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vehfbdiag18c4eaef::veh_fb_ehboilfault(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(6, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 29, 'is_signed_var': False, 'len': 1, 'name':
// 'veh_fb_ehboilpresssensorfault', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'bool'}
bool Vehfbdiag18c4eaef::veh_fb_ehboilpresssensorfault(const std::uint8_t* bytes,
                                                      int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 28, 'is_signed_var': False, 'len': 1, 'name':
// 'veh_fb_ehbmotorfault', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vehfbdiag18c4eaef::veh_fb_ehbmotorfault(const std::uint8_t* bytes,
                                             int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 27, 'is_signed_var': False, 'len': 1, 'name':
// 'veh_fb_ehbsensorabnomal', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vehfbdiag18c4eaef::veh_fb_ehbsensorabnomal(const std::uint8_t* bytes,
                                                int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 26, 'is_signed_var': False, 'len': 1, 'name':
// 'veh_fb_ehbpowerfault', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vehfbdiag18c4eaef::veh_fb_ehbpowerfault(const std::uint8_t* bytes,
                                             int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 25, 'is_signed_var': False, 'len': 1, 'name':
// 'veh_fb_ehbot', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vehfbdiag18c4eaef::veh_fb_ehbot(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 24, 'is_signed_var': False, 'len': 1, 'name':
// 'veh_fb_ehbangulefault', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vehfbdiag18c4eaef::veh_fb_ehbangulefault(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 23, 'is_signed_var': False, 'len': 1, 'name':
// 'veh_fb_ehbdisen', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vehfbdiag18c4eaef::veh_fb_ehbdisen(const std::uint8_t* bytes,
                                        int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(7, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 22, 'is_signed_var': False, 'len': 1, 'name':
// 'veh_fb_ehbworkmodelfault', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'bool'}
bool Vehfbdiag18c4eaef::veh_fb_ehbworkmodelfault(const std::uint8_t* bytes,
                                                 int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(6, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 13, 'is_signed_var': False, 'len': 1, 'name':
// 'veh_fb_epsovercurrent', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vehfbdiag18c4eaef::veh_fb_epsovercurrent(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 12, 'is_signed_var': False, 'len': 1, 'name':
// 'veh_fb_epsdiswork', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vehfbdiag18c4eaef::veh_fb_epsdiswork(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 11, 'is_signed_var': False, 'len': 1, 'name':
// 'veh_fb_epswarning', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vehfbdiag18c4eaef::veh_fb_epswarning(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 47, 'is_signed_var': False, 'len': 1, 'name':
// 'veh_fb_auxremotedisonline', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'bool'}
bool Vehfbdiag18c4eaef::veh_fb_auxremotedisonline(const std::uint8_t* bytes,
                                                  int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(7, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 46, 'is_signed_var': False, 'len': 1, 'name':
// 'veh_fb_auxremoteclose', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vehfbdiag18c4eaef::veh_fb_auxremoteclose(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(6, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 44, 'is_signed_var': False, 'len': 1, 'name':
// 'veh_fb_auxbmsdisonline', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vehfbdiag18c4eaef::veh_fb_auxbmsdisonline(const std::uint8_t* bytes,
                                               int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 5, 'is_signed_var': False, 'len': 1, 'name':
// 'veh_fb_autoiocancmd', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vehfbdiag18c4eaef::veh_fb_autoiocancmd(const std::uint8_t* bytes,
                                            int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 4, 'is_signed_var': False, 'len': 1, 'name':
// 'veh_fb_autocanctrlcmd', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vehfbdiag18c4eaef::veh_fb_autocanctrlcmd(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 56, 'is_signed_var': False, 'len': 8, 'name':
// 'veh_fb_infor_check_bcc', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vehfbdiag18c4eaef::veh_fb_infor_check_bcc(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 52, 'is_signed_var': False, 'len': 4, 'name':
// 'veh_fb_infor_alive_cnt', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vehfbdiag18c4eaef::veh_fb_infor_alive_cnt(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(4, 4);

  int ret = x;
  return ret;
}

// config detail: {'bit': 45, 'is_signed_var': False, 'len': 1, 'name':
// 'veh_fb_auxscram', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vehfbdiag18c4eaef::veh_fb_auxscram(const std::uint8_t* bytes,
                                        int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 21, 'is_signed_var': False, 'len': 1, 'name':
// 'veh_fb_ehbdisonline', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vehfbdiag18c4eaef::veh_fb_ehbdisonline(const std::uint8_t* bytes,
                                            int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 20, 'is_signed_var': False, 'len': 1, 'name':
// 'veh_fb_ehbecufault', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vehfbdiag18c4eaef::veh_fb_ehbecufault(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 10, 'is_signed_var': False, 'len': 1, 'name':
// 'veh_fb_epsmosfetot', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vehfbdiag18c4eaef::veh_fb_epsmosfetot(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 9, 'is_signed_var': False, 'len': 1, 'name':
// 'veh_fb_epsfault', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vehfbdiag18c4eaef::veh_fb_epsfault(const std::uint8_t* bytes,
                                        int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 8, 'is_signed_var': False, 'len': 1, 'name':
// 'veh_fb_epsdisonline', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vehfbdiag18c4eaef::veh_fb_epsdisonline(const std::uint8_t* bytes,
                                            int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 4, 'name':
// 'veh_fb_faultlevel', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vehfbdiag18c4eaef::veh_fb_faultlevel(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}
}  // namespace mk_mini
}  // namespace canbus
}  // namespace apollo
