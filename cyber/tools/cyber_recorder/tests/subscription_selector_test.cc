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

#include "cyber/tools/cyber_recorder/record/selector/subscription_selector.h"

#include "gtest/gtest.h"

namespace apollo {
namespace cyber {
namespace record {
namespace {

TEST(SubscriptionSelectorTest, UsesLastMatchAndReportsConflict) {
  SubscriptionSelectorConfig config;
  config.default_action = SelectorAction::kInclude;
  config.selectors = {
      {"exclude_camera_raw", SelectorMatchType::kRegex,
       "^/camera/.*/image_raw$", SelectorAction::kExclude},
      {"keep_debug_camera", SelectorMatchType::kChannel,
       "/camera/debug/image_raw", SelectorAction::kInclude},
  };
  SubscriptionSelector selector(config);

  const auto debug_decision =
      selector.Evaluate({"/camera/debug/image_raw", "apollo.test.Debug"});
  EXPECT_TRUE(debug_decision.should_subscribe);
  EXPECT_TRUE(debug_decision.has_conflict);
  EXPECT_EQ(1U, debug_decision.include_matches.size());
  EXPECT_EQ(1U, debug_decision.exclude_matches.size());

  const auto normal_decision =
      selector.Evaluate({"/camera/front/image_raw", "apollo.test.Image"});
  EXPECT_FALSE(normal_decision.should_subscribe);
  EXPECT_FALSE(normal_decision.has_conflict);
}

}  // namespace
}  // namespace record
}  // namespace cyber
}  // namespace apollo
