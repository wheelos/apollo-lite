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

#include "modules/canbus/vehicle/apple/protocol/wheel_rr_status_1b9.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace apple {

using ::apollo::drivers::canbus::Byte;

Wheelrrstatus1b9::Wheelrrstatus1b9() {}
const int32_t Wheelrrstatus1b9::ID = 0x1B9;

void Wheelrrstatus1b9::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_apple()->mutable_wheel_rr_status_1b9()->set_errorcode_rr(errorcode_rr(bytes, length));
  chassis->mutable_apple()->mutable_wheel_rr_status_1b9()->set_statusword_rr(statusword_rr(bytes, length));
  chassis->mutable_apple()->mutable_wheel_rr_status_1b9()->set_torque_rr(torque_rr(bytes, length));
  chassis->mutable_apple()->mutable_wheel_rr_status_1b9()->set_speed_rr(speed_rr(bytes, length));
}

// config detail: {'bit': 0, 'enum': {0: 'ERRORCODE_RR_NOERROR', 1: 'ERRORCODE_RR_OVERVOLTAGE', 2: 'ERRORCODE_RR_UNDERVOLTAGE', 4: 'ERRORCODE_RR_OVERCURRENT', 5: 'ERRORCODE_RR_CONTROLLEROVERTEMP', 6: 'ERRORCODE_RR_MOTOROVERTEMP', 9: 'ERRORCODE_RR_MCUUNDERVOLTAGE', 11: 'ERRORCODE_RR_ENCODERSPIERROR', 12: 'ERRORCODE_RR_ENCODERAMPLITUDELOW', 13: 'ERRORCODE_RR_ENCODERAMPLITUDEHIGH', 14: 'ERRORCODE_RR_MEMORYSTORAGEERROR', 18: 'ERRORCODE_RR_THREEPHASEIMBALANCE', 23: 'ERRORCODE_RR_APPCONFIGSTORAGEERROR', 24: 'ERRORCODE_RR_CONTROLCONFIGSTORAGEERROR', 28: 'ERRORCODE_RR_ENCODERERROR', 30: 'ERRORCODE_RR_COMMUNICATIONLOST'}, 'is_signed_var': False, 'len': 8, 'name': 'errorcode_rr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|30]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
Wheel_rr_status_1b9::Errorcode_rrType Wheelrrstatus1b9::errorcode_rr(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Wheel_rr_status_1b9::Errorcode_rrType ret =  static_cast<Wheel_rr_status_1b9::Errorcode_rrType>(x);
  return ret;
}

// config detail: {'bit': 8, 'is_signed_var': False, 'len': 16, 'name': 'statusword_rr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|65535]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Wheelrrstatus1b9::statusword_rr(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'bit': 24, 'is_signed_var': True, 'len': 16, 'name': 'torque_rr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1000]', 'physical_unit': 'Percentage', 'precision': 0.001, 'type': 'double'}
double Wheelrrstatus1b9::torque_rr(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.001000;
  return ret;
}

// config detail: {'bit': 40, 'is_signed_var': True, 'len': 16, 'name': 'speed_rr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|25]', 'physical_unit': 'm/s', 'precision': 0.01, 'type': 'double'}
double Wheelrrstatus1b9::speed_rr(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.010000;
  return ret;
}
}  // namespace apple
}  // namespace canbus
}  // namespace apollo
