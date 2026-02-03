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
namespace agilex_hunter {

class Steeringzeroquery433 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Steeringzeroquery433();

  uint32_t GetPeriod() const override;

  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'bit': 7, 'is_signed_var': False, 'len': 8, 'name': 'Zero_Query', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[170|170]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Steeringzeroquery433* set_zero_query(int zero_query);

 private:

  // config detail: {'bit': 7, 'is_signed_var': False, 'len': 8, 'name': 'Zero_Query', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[170|170]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_zero_query(uint8_t* data, int zero_query);

  int zero_query(const std::uint8_t* bytes, const int32_t length) const;

 private:
  int zero_query_;
};

}  // namespace agilex_hunter
}  // namespace canbus
}  // namespace apollo


