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

#include "modules/canbus/vehicle/agilex_hunter/protocol/chassis_status_report_211.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace agilex_hunter {

using ::apollo::drivers::canbus::Byte;

Chassisstatusreport211::Chassisstatusreport211() {}
const int32_t Chassisstatusreport211::ID = 0x211;

void Chassisstatusreport211::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_agilex_hunter()->mutable_chassis_status_report_211()->set_vehicle_state(vehicle_state(bytes, length));
  chassis->mutable_agilex_hunter()->mutable_chassis_status_report_211()->set_control_mode_state(control_mode_state(bytes, length));
  chassis->mutable_agilex_hunter()->mutable_chassis_status_report_211()->set_battery_voltage(battery_voltage(bytes, length));
}

// config detail: {'bit': 7, 'enum': {}, 'is_signed_var': False, 'len': 8, 'name': 'vehicle_state', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|2]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
Chassis_status_report_211::Vehicle_stateType Chassisstatusreport211::vehicle_state(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Chassis_status_report_211::Vehicle_stateType ret =  static_cast<Chassis_status_report_211::Vehicle_stateType>(x);
  return ret;
}

// config detail: {'bit': 15, 'enum': {}, 'is_signed_var': False, 'len': 8, 'name': 'control_mode_state', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|2]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
Chassis_status_report_211::Control_mode_stateType Chassisstatusreport211::control_mode_state(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Chassis_status_report_211::Control_mode_stateType ret =  static_cast<Chassis_status_report_211::Control_mode_stateType>(x);
  return ret;
}

// config detail: {'bit': 31, 'is_signed_var': False, 'len': 16, 'name': 'battery_voltage', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|300]', 'physical_unit': 'V', 'precision': 0.1, 'type': 'double'}
double Chassisstatusreport211::battery_voltage(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 4);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.100000;
  return ret;
}
}  // namespace agilex_hunter
}  // namespace canbus
}  // namespace apollo
