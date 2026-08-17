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

#include "modules/dreamview/backend/hmi/process_manager.h"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <deque>
#include <map>
#include <string>
#include <vector>

#include "cyber/common/log.h"

#include "modules/dreamview/backend/hmi/readiness_probe.h"

namespace apollo {
namespace dreamview {

bool ProcessManager::ValidateMode(const HMIMode& mode) const {
  return BuildStartOrder(mode.modules()).size() == mode.modules().size();
}

bool ProcessManager::SetMode(const HMIMode& mode) {
  std::lock_guard<std::mutex> lock(mutex_);
  const std::vector<std::string> start_order = BuildStartOrder(mode.modules());
  if (start_order.size() != mode.modules().size()) {
    AERROR << "Invalid module dependency graph.";
    return false;
  }

  module_runtime_.clear();
  modules_ = mode.modules();
  start_order_ = start_order;
  for (const auto& iter : mode.modules()) {
    module_runtime_.emplace(iter.first, Runtime{});
  }
  return true;
}

bool ProcessManager::StartModule(const std::string& module_name) {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto module_iter = modules_.find(module_name);
  if (module_iter == modules_.end()) {
    AERROR << "Cannot start module outside the active mode: " << module_name;
    return false;
  }
  std::vector<std::string> visiting;
  return StartModuleLocked(module_name, &visiting);
}

bool ProcessManager::StartModuleLocked(const std::string& module_name,
                                       std::vector<std::string>* visiting) {
  const auto module_iter = modules_.find(module_name);
  if (module_iter == modules_.end()) {
    AERROR << "Unknown module dependency: " << module_name;
    return false;
  }
  if (std::find(visiting->begin(), visiting->end(), module_name) !=
      visiting->end()) {
    AERROR << "Circular module dependency: " << module_name;
    return false;
  }

  const Module& module = module_iter->second;
  Runtime& runtime = module_runtime_.at(module_name);
  if (runtime.state == State::RUNNING) {
    if (ReadinessProbe::IsRunning(runtime.handle, runtime.keywords)) {
      return true;
    }
    runtime = Runtime{};
  }
  if (runtime.state != State::STOPPED) {
    AERROR << "Module has an unfinished lifecycle transition: " << module_name;
    return false;
  }

  visiting->push_back(module_name);
  for (const auto& dependency : module.depends_on()) {
    if (!StartModuleLocked(dependency, visiting)) {
      visiting->pop_back();
      return false;
    }
  }
  visiting->pop_back();

  if (!module.exclusive_group().empty()) {
    for (auto& iter : module_runtime_) {
      if (iter.first != module_name &&
          iter.second.exclusive_group == module.exclusive_group() &&
          iter.second.state == State::RUNNING) {
        AERROR << "Exclusive module is already running: " << iter.first;
        return false;
      }
    }
  }

  runtime.state = State::STARTING;
  runtime.keywords.assign(module.process_monitor_config().command_keywords().begin(),
                          module.process_monitor_config().command_keywords().end());
  runtime.exclusive_group = module.exclusive_group();
  if (!runner_.Start(module.start_command(), &runtime.handle)) {
    runtime = Runtime{};
    return false;
  }
  const bool ready = ReadinessProbe::WaitUntilRunning(
      runtime.handle, runtime.keywords, std::chrono::milliseconds(5000));
  if (!ready) {
    runner_.Stop(runtime.handle, runtime.keywords);
    runtime = Runtime{};
    return false;
  }
  runtime.state = State::RUNNING;
  return true;
}

bool ProcessManager::StopModule(const std::string& module_name) {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto module_iter = modules_.find(module_name);
  if (module_iter == modules_.end()) {
    AERROR << "Cannot stop module outside the active mode: " << module_name;
    return false;
  }
  Runtime& runtime = module_runtime_.at(module_name);
  if (runtime.state == State::STOPPED) {
    return true;
  }
  if (HasRunningDependentLocked(module_name)) {
    AERROR << "Cannot stop module with running dependents: " << module_name;
    return false;
  }
  runtime.state = State::STOPPING;
  const bool stopped =
      runtime.handle.pid <= 0 || runner_.Stop(runtime.handle, runtime.keywords);
  runtime = Runtime{};
  return stopped;
}

bool ProcessManager::HasRunningDependentLocked(
    const std::string& module_name) const {
  for (const auto& iter : modules_) {
    const Runtime& runtime = module_runtime_.at(iter.first);
    if (runtime.state == State::RUNNING &&
        std::find(iter.second.depends_on().begin(),
                  iter.second.depends_on().end(),
                  module_name) != iter.second.depends_on().end()) {
      return true;
    }
  }
  return false;
}

std::vector<std::string> ProcessManager::GetStartOrder() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return start_order_;
}

std::vector<std::string> ProcessManager::GetStopOrder() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::string> stop_order = start_order_;
  std::reverse(stop_order.begin(), stop_order.end());
  return stop_order;
}

std::vector<std::string> ProcessManager::BuildStartOrder(
    const google::protobuf::Map<std::string, Module>& modules) {
  std::map<std::string, int> indegree;
  std::map<std::string, std::vector<std::string>> edges;
  for (const auto& iter : modules) {
    indegree[iter.first] = 0;
  }
  for (const auto& iter : modules) {
    for (const auto& dep : iter.second.depends_on()) {
      if (modules.find(dep) == modules.end()) {
        AERROR << "Unknown dependency \"" << dep << "\" for module "
               << iter.first;
        return {};
      }
      edges[dep].push_back(iter.first);
      ++indegree[iter.first];
    }
  }

  std::deque<std::string> ready;
  for (const auto& iter : indegree) {
    if (iter.second == 0) {
      ready.push_back(iter.first);
    }
  }

  std::vector<std::string> order;
  while (!ready.empty()) {
    const std::string current = ready.front();
    ready.pop_front();
    order.push_back(current);
    auto& next_modules = edges[current];
    std::sort(next_modules.begin(), next_modules.end());
    for (const auto& next : next_modules) {
      if (--indegree[next] == 0) {
        ready.push_back(next);
      }
    }
  }

  if (order.size() != modules.size()) {
    AERROR << "Dependency graph has a cycle.";
    return {};
  }
  return order;
}

}  // namespace dreamview
}  // namespace apollo
