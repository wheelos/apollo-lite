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

#include "cyber/tools/cyber_recorder/record/selector/topic_matcher.h"

#include <regex>

namespace apollo {
namespace cyber {
namespace record {

bool TopicMatchesRule(const SelectorMatchType match_type,
                      const std::string& match_expression,
                      const std::string& topic) {
  if (match_type == SelectorMatchType::kChannel) {
    return match_expression == topic;
  }
  try {
    return std::regex_match(topic, std::regex(match_expression));
  } catch (const std::regex_error&) {
    return false;
  }
}

}  // namespace record
}  // namespace cyber
}  // namespace apollo
