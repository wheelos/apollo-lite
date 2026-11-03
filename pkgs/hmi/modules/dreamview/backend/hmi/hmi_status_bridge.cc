/******************************************************************************
 * Copyright 2026 The Apollo Authors. All Rights Reserved.
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

#include "modules/dreamview/backend/hmi/hmi_status_bridge.h"

#include "cyber/common/log.h"
#include "modules/common/util/map_util.h"

namespace apollo {
namespace dreamview {

void HMIStatusBridge::PopulateModeStatus(const HMIMode& mode, HMIStatus* status) {
  CHECK_NOTNULL(status);
  status->clear_modules();
  for (const auto& iter : mode.modules()) {
    status->mutable_modules()->insert({iter.first, false});
  }

  status->clear_monitored_components();
  for (const auto& iter : mode.monitored_components()) {
    status->mutable_monitored_components()->insert({iter.first, {}});
  }

  status->clear_other_components();
  for (const auto& iter : mode.other_components()) {
    status->mutable_other_components()->insert({iter.first, {}});
  }
}

bool HMIStatusBridge::SetModuleRunning(HMIStatus* status,
                                       const std::string& module,
                                       bool running) {
  CHECK_NOTNULL(status);
  auto* module_status =
      apollo::common::util::FindOrNull(*status->mutable_modules(), module);
  if (module_status == nullptr || *module_status == running) {
    return false;
  }
  *module_status = running;
  return true;
}

}  // namespace dreamview
}  // namespace apollo
