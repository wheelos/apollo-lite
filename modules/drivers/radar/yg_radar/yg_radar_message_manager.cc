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

#include "modules/drivers/radar/yg_radar/yg_radar_message_manager.h"

#include "modules/common_msgs/sensor_msgs/radar.pb.h"

#include "cyber/time/time.h"
#include "modules/common/util/message_util.h"
#include "modules/drivers/radar/yg_radar/protocol/radar_object_info_540.h"
#include "modules/drivers/radar/yg_radar/protocol/radar_object_list_560.h"

namespace apollo {
namespace drivers {
namespace yg_radar {

YgRadarMessageManager::YgRadarMessageManager() {
  // Control Messages

  // Report Messages
  AddRecvProtocolData<Radarobjectinfo540, true>();
  AddRecvProtocolData<Radarobjectlist560, true>();
}

YgRadarMessageManager::~YgRadarMessageManager() {}

void YgRadarMessageManager::Parse(const uint32_t message_id,
                                  const uint8_t* data, int32_t length) {
  // Parse method triggered by CanReceiver when a new message is received

  if (message_id >
          static_cast<uint32_t>(Radarobjectlist560::ID + message_id_offset_) ||
      message_id <
          static_cast<uint32_t>(Radarobjectinfo540::ID + message_id_offset_)) {
    // ignore messages that not match the message_id_offset_
    return;
  }
  // treat 0x540~0x55F as 0x540
  // treat 0x560 as 0x560
  apollo::drivers::canbus::ProtocolData<apollo::drivers::yg_radar::Yg_radar>*
      protocol_data = GetMutableProtocolDataById(
          (message_id - message_id_offset_) & 0xFFE0);
  if (protocol_data == nullptr) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(sensor_data_mutex_);
    protocol_data->Parse(data, length, &sensor_data_);
    // object list received
    if (((message_id - message_id_offset_) & 0xFFE0) ==
        static_cast<uint32_t>(Radarobjectlist560::ID)) {
      auto& obj_list = sensor_data_.radar_object_list_560();
      current_obstacle_size_ = obj_list.object_number();
      // trigger obstacle message publish
      if (modified_) {
        // if modified_ is true, means received number of obstacles is zero or
        // less than the `object_number`, the count trigger will not work, so
        // publish the message here
        writer_->Write(radar_obstacles_);
        // clear for next frame
        radar_obstacles_.clear_radar_obstacle();
        modified_ = false;
      }
      // fill header when receive the object list message
      common::util::FillHeader("yg_radar", &radar_obstacles_);
      modified_ = true;
    }
    // object info received
    if (((message_id - message_id_offset_) & 0xFFE0) ==
        static_cast<uint32_t>(Radarobjectinfo540::ID)) {
      auto& obj_info = sensor_data_.radar_object_info_540();
      auto& obstacle =
          (*(radar_obstacles_
                 .mutable_radar_obstacle()))[obj_info.object_tracking_id()];
      obstacle.set_id(obj_info.object_tracking_id());
      obstacle.mutable_relative_position()->set_x(
          obj_info.object_distance_lat());
      obstacle.mutable_relative_position()->set_y(
          obj_info.object_distance_lon());
      obstacle.set_theta(obj_info.object_angle());
      // no x axis velocity info from radar of this model
      obstacle.mutable_relative_velocity()->set_y(obj_info.object_speed());
      if (obj_info.object_speed() > 0) {
        obstacle.set_moving_status(drivers::RadarObstacle::AWAYING);
      } else if (obj_info.object_speed() < 0) {
        obstacle.set_moving_status(drivers::RadarObstacle::NEARING);
      } else {
        obstacle.set_moving_status(drivers::RadarObstacle::STATIONARY);
      }
      modified_ = true;
      if (radar_obstacles_.radar_obstacle_size() >= current_obstacle_size_) {
        // publish message if received count match of then number of obstacles
        writer_->Write(radar_obstacles_);
        // clear for next frame
        radar_obstacles_.clear_radar_obstacle();
        modified_ = false;
      }
    }
  }

  received_ids_.insert((message_id - message_id_offset_) & 0xFFE0);
  // check if need to check period
  const auto it = check_ids_.find((message_id - message_id_offset_) & 0xFFE0);
  if (it != check_ids_.end()) {
    const int64_t time = cyber::Time::Now().ToNanosecond() / 1e3;
    it->second.real_period = time - it->second.last_time;
    // if period 1.5 large than base period, inc error_count
    const double period_multiplier = 1.5;
    if (static_cast<double>(it->second.real_period) >
        (static_cast<double>(it->second.period) * period_multiplier)) {
      it->second.error_count += 1;
    } else {
      it->second.error_count = 0;
    }
    it->second.last_time = time;
  }
}

}  // namespace yg_radar
}  // namespace drivers
}  // namespace apollo
