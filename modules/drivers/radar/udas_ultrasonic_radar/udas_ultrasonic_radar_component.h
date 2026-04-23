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
#include <vector>

#include "modules/common_msgs/sensor_msgs/ultrasonic_radar.pb.h"
#include "modules/drivers/radar/udas_ultrasonic_radar/proto/ultrasonic_radar_config.pb.h"

#include "cyber/component/timer_component.h"
#include "cyber/cyber.h"
#include "cyber/node/writer.h"
#include "modules/drivers/canbus/can_client/can_client.h"
#include "modules/drivers/canbus/can_comm/can_receiver.h"
#include "modules/drivers/radar/udas_ultrasonic_radar/udas_ultrasonic_radar_message_manager.h"

namespace apollo {
namespace drivers {
namespace ultrasonic_radar {

class UdasUltrasonicRadarComponent : public apollo::cyber::TimerComponent {
 public:
  UdasUltrasonicRadarComponent();
  ~UdasUltrasonicRadarComponent();
  bool Init() override;
  bool Proc() override;

 private:
  UltrasonicRadarConfig config_;
  std::vector<float> ranges_;
  std::shared_ptr<apollo::drivers::canbus::CanClient> can_client_;
  apollo::drivers::canbus::CanReceiver<
      apollo::drivers::udas_ultrasonic_radar::Udas_ultrasonic_radar>
      can_receiver_;
  std::unique_ptr<apollo::drivers::canbus::MessageManager<
      apollo::drivers::udas_ultrasonic_radar::Udas_ultrasonic_radar>>
      message_manager_;
  std::shared_ptr<apollo::cyber::Writer<apollo::drivers::Ultrasonic>> writer_;
};

CYBER_REGISTER_COMPONENT(UdasUltrasonicRadarComponent)

}  // namespace ultrasonic_radar
}  // namespace drivers
}  // namespace apollo
