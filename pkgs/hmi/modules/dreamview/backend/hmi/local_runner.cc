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

#include "modules/dreamview/backend/hmi/local_runner.h"

#include <chrono>
#include <cerrno>
#include <csignal>
#include <cstring>
#include <sys/wait.h>
#include <unistd.h>

#include "absl/strings/match.h"
#include "absl/strings/strip.h"

#include "cyber/common/log.h"

#include "modules/dreamview/backend/hmi/readiness_probe.h"

namespace apollo {
namespace dreamview {
namespace {

void ReapChild(const pid_t pid) {
  if (pid <= 0) {
    return;
  }
  int status = 0;
  while (waitpid(pid, &status, WNOHANG) > 0) {
  }
}

}  // namespace

bool LocalRunner::Start(const std::string& raw_command,
                        ProcessHandle* handle) const {
  CHECK_NOTNULL(handle);
  const std::string command = NormalizeStartCommand(raw_command);
  if (command.empty()) {
    AERROR << "Start command is empty after normalization: " << raw_command;
    return false;
  }

  const pid_t pid = fork();
  if (pid < 0) {
    AERROR << "Failed to fork process for command: " << command
           << ", errno: " << std::strerror(errno);
    return false;
  }

  if (pid == 0) {
    if (setsid() < 0) {
      _exit(127);
    }
    execl("/bin/bash", "bash", "-lc", command.c_str(),
          static_cast<char*>(nullptr));
    _exit(127);
  }

  handle->pid = pid;
  handle->pgid = pid;
  return true;
}

bool LocalRunner::Stop(const ProcessHandle& handle,
                       const std::vector<std::string>& keywords) const {
  if (handle.pgid <= 0 && handle.pid <= 0 && keywords.empty()) {
    return true;
  }

  auto stop_with_signal = [&](const int signal,
                              const std::chrono::milliseconds wait_timeout) {
    if (handle.pgid > 0) {
      if (kill(-handle.pgid, signal) != 0 && errno != ESRCH) {
        AERROR << "Failed to send signal " << signal << " to process group "
               << handle.pgid << ", errno: " << std::strerror(errno);
      }
    } else if (handle.pid > 0) {
      if (kill(handle.pid, signal) != 0 && errno != ESRCH) {
        AERROR << "Failed to send signal " << signal << " to process "
               << handle.pid << ", errno: " << std::strerror(errno);
      }
    }
    return ReadinessProbe::WaitUntilStopped(handle, keywords, wait_timeout);
  };

  if (stop_with_signal(SIGINT, std::chrono::milliseconds(1500))) {
    ReapChild(handle.pid);
    return true;
  }
  if (stop_with_signal(SIGTERM, std::chrono::milliseconds(2000))) {
    ReapChild(handle.pid);
    return true;
  }
  const bool stopped =
      stop_with_signal(SIGKILL, std::chrono::milliseconds(1500));
  ReapChild(handle.pid);
  return stopped;
}

std::string LocalRunner::NormalizeStartCommand(const std::string& raw_command) {
  std::string command = std::string(absl::StripAsciiWhitespace(raw_command));
  if (absl::StartsWith(command, "nohup ")) {
    command = std::string(absl::StripAsciiWhitespace(command.substr(6)));
  }
  if (!command.empty() && command.back() == '&') {
    command.pop_back();
    command = std::string(absl::StripAsciiWhitespace(command));
  }
  return command;
}

}  // namespace dreamview
}  // namespace apollo
