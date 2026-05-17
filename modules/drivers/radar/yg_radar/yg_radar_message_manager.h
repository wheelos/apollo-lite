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

#include <memory>

#include "modules/common_msgs/sensor_msgs/radar.pb.h"
#include "modules/drivers/radar/yg_radar/proto/yg_radar.pb.h"

#include "cyber/node/writer.h"
#include "modules/drivers/canbus/can_comm/message_manager.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace drivers {
namespace yg_radar {

using ::apollo::drivers::canbus::MessageManager;

class YgRadarMessageManager
    : public MessageManager<::apollo::drivers::yg_radar::Yg_radar> {
 public:
  YgRadarMessageManager();
  virtual ~YgRadarMessageManager();
  // apollo::drivers::canbus::ProtocolData<apollo::drivers::yg_radar::Yg_radar>*
  // GetMutableProtocolDataById(const uint32_t message_id);
  void Parse(const uint32_t message_id, const uint8_t* data,
             int32_t length) override;
  void set_message_id_offset(int32_t offset) { message_id_offset_ = offset; }
  void set_writer(
      const std::shared_ptr<
          apollo::cyber::Writer<apollo::drivers::RadarObstacles>>& writer) {
    writer_ = writer;
  }

 private:
  int32_t message_id_offset_;
  int32_t current_obstacle_size_ = 0;
  bool modified_ = false;
  apollo::drivers::RadarObstacles radar_obstacles_;
  std::shared_ptr<apollo::cyber::Writer<apollo::drivers::RadarObstacles>>
      writer_;
};

}  // namespace yg_radar
}  // namespace drivers
}  // namespace apollo
