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

#include "modules/canbus/vehicle/agilex_hunter/agilex_hunter_message_manager.h"

#include "modules/canbus/vehicle/agilex_hunter/protocol/control_mode_command_421.h"
#include "modules/canbus/vehicle/agilex_hunter/protocol/error_clear_command_441.h"
#include "modules/canbus/vehicle/agilex_hunter/protocol/motion_command_111.h"
#include "modules/canbus/vehicle/agilex_hunter/protocol/steering_zero_query_433.h"
#include "modules/canbus/vehicle/agilex_hunter/protocol/steering_zero_set_432.h"

#include "modules/canbus/vehicle/agilex_hunter/protocol/chassis_status_report_211.h"
#include "modules/canbus/vehicle/agilex_hunter/protocol/motion_feedback_221.h"
#include "modules/canbus/vehicle/agilex_hunter/protocol/motor_highspeed_251_251.h"
#include "modules/canbus/vehicle/agilex_hunter/protocol/motor_highspeed_252_252.h"
#include "modules/canbus/vehicle/agilex_hunter/protocol/motor_highspeed_253_253.h"
#include "modules/canbus/vehicle/agilex_hunter/protocol/motor_lowspeed_261_261.h"
#include "modules/canbus/vehicle/agilex_hunter/protocol/motor_lowspeed_262_262.h"
#include "modules/canbus/vehicle/agilex_hunter/protocol/motor_lowspeed_263_263.h"
#include "modules/canbus/vehicle/agilex_hunter/protocol/steering_zero_feedback_43b.h"

namespace apollo {
namespace canbus {
namespace agilex_hunter {

Agilex_hunterMessageManager::Agilex_hunterMessageManager() {
  // Control Messages
  AddSendProtocolData<Controlmodecommand421, true>();
  AddSendProtocolData<Errorclearcommand441, true>();
  AddSendProtocolData<Motioncommand111, true>();
  AddSendProtocolData<Steeringzeroquery433, true>();
  AddSendProtocolData<Steeringzeroset432, true>();

  // Report Messages
  AddRecvProtocolData<Chassisstatusreport211, true>();
  AddRecvProtocolData<Motionfeedback221, true>();
  AddRecvProtocolData<Motorhighspeed251251, true>();
  AddRecvProtocolData<Motorhighspeed252252, true>();
  AddRecvProtocolData<Motorhighspeed253253, true>();
  AddRecvProtocolData<Motorlowspeed261261, true>();
  AddRecvProtocolData<Motorlowspeed262262, true>();
  AddRecvProtocolData<Motorlowspeed263263, true>();
  AddRecvProtocolData<Steeringzerofeedback43b, true>();
}

Agilex_hunterMessageManager::~Agilex_hunterMessageManager() {}

}  // namespace agilex_hunter
}  // namespace canbus
}  // namespace apollo
