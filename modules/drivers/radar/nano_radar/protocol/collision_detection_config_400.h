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

#include "modules/common_msgs/sensor_msgs/nano_radar.pb.h"
#include "modules/drivers/radar/nano_radar/proto/nano_radar_conf.pb.h"

#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace drivers {
namespace nano_radar {

using apollo::drivers::NanoRadar;

class CollisionDetectionConfig400
    : public apollo::drivers::canbus::ProtocolData<NanoRadar> {
 public:
  static const uint32_t ID;
  CollisionDetectionConfig400();
  ~CollisionDetectionConfig400();
  /**
   * @brief get the data period
   * @return the value of data period
   */
  uint32_t GetPeriod() const override;

  /**
   * @brief update the data
   * @param data a pointer to the data to be updated
   */
  void UpdateData(uint8_t* data) override;

  /**
   * @brief reset the private variables
   */
  void Reset() override;

  CollisionDetectionConfig400* set_warning_reset(bool reset);
  CollisionDetectionConfig400* set_activation(bool activation);
  CollisionDetectionConfig400* set_min_time_valid(bool valid);
  CollisionDetectionConfig400* set_clear_regions(bool clear);
  CollisionDetectionConfig400* set_min_detect_time(double value);
  CollisionDetectionConfig400* set_radar_conf(RadarConf radar_conf);
  RadarConf radar_conf();

  void set_warning_reset_p(uint8_t* data, bool value);
  void set_activation_p(uint8_t* data, bool value);
  void set_min_time_valid_p(uint8_t* data, bool value);
  void set_clear_regions_p(uint8_t* data, bool value);
  void set_min_detect_time_p(uint8_t* data, double value);

 private:
  RadarConf radar_conf_;
};

}  // namespace nano_radar
}  // namespace drivers
}  // namespace apollo
