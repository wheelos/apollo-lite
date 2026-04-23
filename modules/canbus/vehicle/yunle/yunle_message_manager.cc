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

#include "modules/canbus/vehicle/yunle/yunle_message_manager.h"

#include "modules/canbus/vehicle/yunle/protocol/bms_rt_soc_17904001.h"
#include "modules/canbus/vehicle/yunle/protocol/bms_soc_101.h"
#include "modules/canbus/vehicle/yunle/protocol/bms_vol_cur_100.h"
#include "modules/canbus/vehicle/yunle/protocol/ccu_status_51.h"
#include "modules/canbus/vehicle/yunle/protocol/sas_angle_feedback_e1.h"
#include "modules/canbus/vehicle/yunle/protocol/scu_1_121.h"
#include "modules/canbus/vehicle/yunle/protocol/scu_tq_123.h"
#include "modules/canbus/vehicle/yunle/protocol/target_speed_feedback_7f1.h"
#include "modules/canbus/vehicle/yunle/protocol/waring_level_77.h"
#include "modules/canbus/vehicle/yunle/protocol/wheel_speed_feedback_rpm_168.h"

namespace apollo {
namespace canbus {
namespace yunle {

YunleMessageManager::YunleMessageManager() {
  // Control Messages
  AddSendProtocolData<Scu1121, true>();
  AddSendProtocolData<Scutq123, true>();

  // Report Messages
  AddRecvProtocolData<Bmsrtsoc17904001, true>();
  AddRecvProtocolData<Bmssoc101, true>();
  AddRecvProtocolData<Bmsvolcur100, true>();
  AddRecvProtocolData<Ccustatus51, true>();
  AddRecvProtocolData<Sasanglefeedbacke1, true>();
  AddRecvProtocolData<Targetspeedfeedback7f1, true>();
  AddRecvProtocolData<Waringlevel77, true>();
  AddRecvProtocolData<Wheelspeedfeedbackrpm168, true>();
}

YunleMessageManager::~YunleMessageManager() {}

}  // namespace yunle
}  // namespace canbus
}  // namespace apollo
