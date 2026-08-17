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

#include "modules/dreamview/backend/hmi/readiness_probe.h"

#include <algorithm>
#include <cerrno>
#include <csignal>
#include <string>
#include <thread>
#include <vector>

#include "cyber/common/file.h"

namespace apollo {
namespace dreamview {

namespace {

void ReplaceNulWithSpace(std::string* text) {
  for (char& c : *text) {
    if (c == '\0') {
      c = ' ';
    }
  }
}

bool IsProcessAlive(const pid_t pid) {
  if (pid <= 0) {
    return false;
  }
  if (kill(pid, 0) != 0) {
    return errno != ESRCH;
  }
  std::string stat;
  if (!cyber::common::GetContent("/proc/" + std::to_string(pid) + "/stat",
                                 &stat)) {
    return false;
  }
  const size_t closing_paren = stat.rfind(')');
  return closing_paren == std::string::npos ||
         closing_paren + 2 >= stat.size() || stat[closing_paren + 2] != 'Z';
}

bool ProcessCommandMatchesKeywords(const std::string& cmdline,
                                   const std::vector<std::string>& keywords) {
  if (keywords.empty()) {
    return false;
  }
  return std::all_of(keywords.begin(), keywords.end(),
                     [&cmdline](const std::string& keyword) {
                       return cmdline.find(keyword) != std::string::npos;
                     });
}

}  // namespace

bool ReadinessProbe::IsRunning(const ProcessHandle& handle,
                               const std::vector<std::string>& keywords) {
  if (!IsProcessAlive(handle.pid)) {
    return false;
  }
  if (keywords.empty()) {
    return true;
  }
  std::string cmdline;
  if (!cyber::common::GetContent(
          "/proc/" + std::to_string(handle.pid) + "/cmdline", &cmdline)) {
    return false;
  }
  ReplaceNulWithSpace(&cmdline);
  return ProcessCommandMatchesKeywords(cmdline, keywords);
}

bool ReadinessProbe::IsStopped(const ProcessHandle& handle,
                               const std::vector<std::string>& keywords) {
  return !IsRunning(handle, keywords);
}

bool ReadinessProbe::WaitUntilRunning(const ProcessHandle& handle,
                                      const std::vector<std::string>& keywords,
                                      std::chrono::milliseconds timeout) {
  return WaitUntil([&]() { return IsRunning(handle, keywords); }, timeout);
}

bool ReadinessProbe::WaitUntilStopped(const ProcessHandle& handle,
                                      const std::vector<std::string>& keywords,
                                      std::chrono::milliseconds timeout) {
  return WaitUntil([&]() { return IsStopped(handle, keywords); }, timeout);
}

bool ReadinessProbe::WaitUntil(const std::function<bool()>& predicate,
                               std::chrono::milliseconds timeout,
                               std::chrono::milliseconds interval) {
  const auto end_time = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < end_time) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(interval);
  }
  return predicate();
}

}  // namespace dreamview
}  // namespace apollo
