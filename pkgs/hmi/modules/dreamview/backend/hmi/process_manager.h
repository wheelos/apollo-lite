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

#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "modules/dreamview/backend/hmi/local_runner.h"
#include "modules/dreamview/proto/hmi_mode.pb.h"

namespace apollo {
namespace dreamview {

class ProcessManager {
 public:
  enum class State { STOPPED, STARTING, RUNNING, STOPPING };

  ProcessManager() = default;

  bool ValidateMode(const HMIMode& mode) const;
  bool SetMode(const HMIMode& mode);
  bool StartModule(const std::string& module_name);
  bool StopModule(const std::string& module_name);
  std::vector<std::string> GetStartOrder() const;
  std::vector<std::string> GetStopOrder() const;

 private:
  struct Runtime {
    State state = State::STOPPED;
    ProcessHandle handle;
    std::vector<std::string> keywords;
    std::string exclusive_group;
  };

  bool StartModuleLocked(const std::string& module_name,
                         std::vector<std::string>* visiting);
  bool HasRunningDependentLocked(const std::string& module_name) const;
  static std::vector<std::string> BuildStartOrder(
      const google::protobuf::Map<std::string, Module>& modules);

  mutable std::mutex mutex_;
  LocalRunner runner_;
  google::protobuf::Map<std::string, Module> modules_;
  std::map<std::string, Runtime> module_runtime_;
  std::vector<std::string> start_order_;
};

}  // namespace dreamview
}  // namespace apollo
