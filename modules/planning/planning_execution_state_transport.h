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

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "modules/execution_state_sync/client.h"

namespace apollo {
namespace planning {

class PlanningExecutionStateTransport {
 public:
  execution_state_sync::Result Init(const std::string& path,
                                    const std::string& producer_epoch);
  execution_state_sync::Result Poll(
      std::vector<execution_state_sync::Event>* events);
  execution_state_sync::Result Acknowledge(uint64_t through_sequence);
  std::vector<execution_state_sync::Submission> DrainSubmissions();
  execution_state_sync::Result Submit(
      execution_state_sync::Channel channel, std::string payload,
      std::vector<execution_state_sync::Guard> guards, bool cleanup,
      execution_state_sync::PlanningStatusKind planning_status_kind);

  std::shared_ptr<const execution_state_sync::WorkerView> Latest() const;
  bool Healthy() const;
  bool Ready() const;
  uint64_t cursor() const;

 private:
  std::unique_ptr<execution_state_sync::Client> client_;
};

}  // namespace planning
}  // namespace apollo
