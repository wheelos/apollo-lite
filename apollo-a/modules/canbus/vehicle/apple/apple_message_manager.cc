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

#include "modules/canbus/vehicle/apple/apple_message_manager.h"



#include "modules/canbus/vehicle/apple/protocol/light_control_7ff.h"
#include "modules/canbus/vehicle/apple/protocol/wheel_fl_control_2b7.h"
#include "modules/canbus/vehicle/apple/protocol/wheel_fl_status_1b7.h"
#include "modules/canbus/vehicle/apple/protocol/wheel_fr_control_2b6.h"
#include "modules/canbus/vehicle/apple/protocol/wheel_fr_status_1b6.h"
#include "modules/canbus/vehicle/apple/protocol/wheel_rl_control_2b8.h"
#include "modules/canbus/vehicle/apple/protocol/wheel_rl_status_1b8.h"
#include "modules/canbus/vehicle/apple/protocol/wheel_rr_control_2b9.h"
#include "modules/canbus/vehicle/apple/protocol/wheel_rr_status_1b9.h"

namespace apollo {
namespace canbus {
namespace apple {

AppleMessageManager::AppleMessageManager() {
  // Control Messages


  // Report Messages
  AddRecvProtocolData<Lightcontrol7ff, true>();
  AddRecvProtocolData<Wheelflcontrol2b7, true>();
  AddRecvProtocolData<Wheelflstatus1b7, true>();
  AddRecvProtocolData<Wheelfrcontrol2b6, true>();
  AddRecvProtocolData<Wheelfrstatus1b6, true>();
  AddRecvProtocolData<Wheelrlcontrol2b8, true>();
  AddRecvProtocolData<Wheelrlstatus1b8, true>();
  AddRecvProtocolData<Wheelrrcontrol2b9, true>();
  AddRecvProtocolData<Wheelrrstatus1b9, true>();
}

AppleMessageManager::~AppleMessageManager() {}

}  // namespace apple
}  // namespace canbus
}  // namespace apollo
