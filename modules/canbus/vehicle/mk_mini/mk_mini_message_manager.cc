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

#include "modules/canbus/vehicle/mk_mini/mk_mini_message_manager.h"

#include "modules/canbus/vehicle/mk_mini/protocol/bms_flag_infor_18c4e2ef.h"
#include "modules/canbus/vehicle/mk_mini/protocol/bms_infor_18c4e1ef.h"
#include "modules/canbus/vehicle/mk_mini/protocol/ctrl_cmd_18c4d2d0.h"
#include "modules/canbus/vehicle/mk_mini/protocol/ctrl_fb_18c4d2ef.h"
#include "modules/canbus/vehicle/mk_mini/protocol/io_cmd_18c4d7d0.h"
#include "modules/canbus/vehicle/mk_mini/protocol/io_fb_18c4daef.h"
#include "modules/canbus/vehicle/mk_mini/protocol/lr_wheel_fb_18c4d7ef.h"
#include "modules/canbus/vehicle/mk_mini/protocol/odo_fb_18c4deef.h"
#include "modules/canbus/vehicle/mk_mini/protocol/rr_wheel_fb_18c4d8ef.h"
#include "modules/canbus/vehicle/mk_mini/protocol/sensor_reset_18ffffff.h"
#include "modules/canbus/vehicle/mk_mini/protocol/ultrasonic_1_fb_18c4e8ef.h"
#include "modules/canbus/vehicle/mk_mini/protocol/ultrasonic_2_fb_18c4e9ef.h"
#include "modules/canbus/vehicle/mk_mini/protocol/veh_fb_diag_18c4eaef.h"

namespace apollo {
namespace canbus {
namespace mk_mini {

Mk_miniMessageManager::Mk_miniMessageManager() {
  // Control Messages
  AddSendProtocolData<Ctrlcmd18c4d2d0, true>();
  AddSendProtocolData<Iocmd18c4d7d0, true>();

  // Report Messages
  AddRecvProtocolData<Bmsflaginfor18c4e2ef, true>();
  AddRecvProtocolData<Bmsinfor18c4e1ef, true>();
  AddRecvProtocolData<Ctrlfb18c4d2ef, true>();
  AddRecvProtocolData<Iofb18c4daef, true>();
  AddRecvProtocolData<Lrwheelfb18c4d7ef, true>();
  AddRecvProtocolData<Odofb18c4deef, true>();
  AddRecvProtocolData<Rrwheelfb18c4d8ef, true>();
  AddRecvProtocolData<Ultrasonic1fb18c4e8ef, true>();
  AddRecvProtocolData<Ultrasonic2fb18c4e9ef, true>();
  AddRecvProtocolData<Vehfbdiag18c4eaef, true>();
}

Mk_miniMessageManager::~Mk_miniMessageManager() {}

}  // namespace mk_mini
}  // namespace canbus
}  // namespace apollo
