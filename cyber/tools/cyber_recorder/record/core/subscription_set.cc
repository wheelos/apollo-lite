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

#include "cyber/tools/cyber_recorder/record/core/subscription_set.h"

namespace apollo {
namespace cyber {
namespace record {

bool SubscriptionSet::EnsureSubscribed(const std::string& topic,
                                       const ReaderFactory& factory) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (readers_.find(topic) != readers_.end()) {
      return true;
    }
  }

  std::shared_ptr<ReaderBase> reader = factory();
  if (reader == nullptr) {
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  readers_[topic] = reader;
  return true;
}

void SubscriptionSet::Unsubscribe(const std::string& topic) {
  std::lock_guard<std::mutex> lock(mutex_);
  readers_.erase(topic);
}

bool SubscriptionSet::Contains(const std::string& topic) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return readers_.find(topic) != readers_.end();
}

void SubscriptionSet::Clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  readers_.clear();
}

size_t SubscriptionSet::Size() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return readers_.size();
}

}  // namespace record
}  // namespace cyber
}  // namespace apollo
