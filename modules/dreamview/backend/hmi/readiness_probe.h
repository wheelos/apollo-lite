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

#include <chrono>
#include <functional>
#include <string>
#include <vector>

#include "modules/dreamview/backend/hmi/process_types.h"

namespace apollo {
namespace dreamview {

class ReadinessProbe {
 public:
  static bool IsRunning(const ProcessHandle& handle,
                        const std::vector<std::string>& keywords);
  static bool IsStopped(const ProcessHandle& handle,
                        const std::vector<std::string>& keywords);
  static bool WaitUntilRunning(const ProcessHandle& handle,
                               const std::vector<std::string>& keywords,
                               std::chrono::milliseconds timeout);
  static bool WaitUntilStopped(const ProcessHandle& handle,
                               const std::vector<std::string>& keywords,
                               std::chrono::milliseconds timeout);

 private:
  static bool WaitUntil(const std::function<bool()>& predicate,
                        std::chrono::milliseconds timeout,
                        std::chrono::milliseconds interval =
                            std::chrono::milliseconds(100));
};

}  // namespace dreamview
}  // namespace apollo
