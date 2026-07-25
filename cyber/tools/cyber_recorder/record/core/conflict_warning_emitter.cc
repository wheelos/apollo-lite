// Copyright 2026 WheelOS All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "cyber/tools/cyber_recorder/record/core/conflict_warning_emitter.h"

#include <sstream>

#include "cyber/common/log.h"

namespace apollo {
namespace cyber {
namespace record {

ConflictWarningEmitter::ConflictWarningEmitter(const uint32_t config_version)
    : config_version_(config_version) {}

void ConflictWarningEmitter::WarnSubscriptionConflict(
    const std::string& topic, const std::vector<std::string>& include_matches,
    const std::vector<std::string>& exclude_matches) {
  if (!MarkWarningAndCheckNeedEmit(topic)) {
    return;
  }
  AWARN << "WARN [subscription] topic=" << topic
        << " include=" << Join(include_matches)
        << " exclude=" << Join(exclude_matches);
}

void ConflictWarningEmitter::WarnPolicyConflict(
    const std::string& topic, const std::string& selected,
    const std::vector<std::string>& shadowed) {
  if (!MarkWarningAndCheckNeedEmit(topic)) {
    return;
  }
  AWARN << "WARN [policy] topic=" << topic << " selected=" << selected
        << " shadowed=" << Join(shadowed);
}

bool ConflictWarningEmitter::MarkWarningAndCheckNeedEmit(
    const std::string& topic) {
  std::ostringstream stream;
  stream << config_version_ << ":" << topic;
  const std::string key = stream.str();
  std::lock_guard<std::mutex> lock(mutex_);
  return warned_topics_.insert(key).second;
}

std::string ConflictWarningEmitter::Join(
    const std::vector<std::string>& values) {
  std::ostringstream stream;
  stream << "[";
  for (size_t i = 0; i < values.size(); ++i) {
    if (i > 0) {
      stream << ",";
    }
    stream << values[i];
  }
  stream << "]";
  return stream.str();
}

}  // namespace record
}  // namespace cyber
}  // namespace apollo
