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

#include "modules/drivers/radar/yg_radar/proto/yg_radar.pb.h"

#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace drivers {
namespace yg_radar {

class Radarobjectlist560 : public ::apollo::drivers::canbus::ProtocolData<
                               ::apollo::drivers::yg_radar::Yg_radar> {
 public:
  static const int32_t ID;
  Radarobjectlist560();

  uint32_t GetPeriod() const override;

  void Parse(const std::uint8_t* bytes, int32_t length,
             ::apollo::drivers::yg_radar::Yg_radar* message) const override;

 private:
  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 8, 'name':
  // 'Object_Number', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|32]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int object_number(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 8, 'is_signed_var': True, 'len': 16, 'name':
  // 'Guardrail_Distance', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[-3276.8|3276.7]', 'physical_unit': 'm', 'precision': 0.1, 'type':
  // 'double'}
  double guardrail_distance(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'bit': 24, 'is_signed_var': True, 'len': 1, 'name':
  // 'Vehicle_Speed', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[-3276.8|3276.7]', 'physical_unit': 'km/h', 'precision': 0.1, 'type':
  // 'bool'}
  bool vehicle_speed(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace yg_radar
}  // namespace drivers
}  // namespace apollo
