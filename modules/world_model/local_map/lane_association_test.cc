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

#include "modules/world_model/local_map/lane_association.h"

#include "gtest/gtest.h"

namespace apollo {
namespace world_model {
namespace {

TemporalLanePolicy Policy() {
  TemporalLanePolicy policy;
  policy.grid_spacing = 0.5;
  policy.sample_lifetime = 1.0;
  policy.association_distance = 0.5;
  policy.min_overlap = 0.5;
  policy.fresh_weight = 0.5;
  return policy;
}

std::map<int, TrackedLaneSample> Samples(double left_y, double right_y,
                                         double time) {
  return {{0, {{0.0, left_y}, {0.0, right_y}, time, 0.02}},
          {1, {{0.5, left_y}, {0.5, right_y}, time, 0.02}},
          {2, {{1.0, left_y}, {1.0, right_y}, time, 0.02}}};
}

TEST(LaneAssociationTest, FusesWithinGateAndKeepsWorstUncertainty) {
  const auto history = Samples(2.5, -2.5, 0.5);
  auto candidates = Samples(2.6, -2.6, 1.0);
  candidates[1].position_error = 0.04;
  LaneAssociationResult result;

  ASSERT_TRUE(
      AssociateLaneSamples(history, candidates, Policy(), 1.0, &result).ok());
  EXPECT_EQ(3U, result.overlap_samples);
  EXPECT_DOUBLE_EQ(2.55, result.samples.at(1).left.y);
  EXPECT_DOUBLE_EQ(-2.55, result.samples.at(1).right.y);
  EXPECT_DOUBLE_EQ(0.04, result.samples.at(1).position_error);
}

TEST(LaneAssociationTest, AppliesInwardRestrictionsImmediately) {
  const auto history = Samples(2.5, -2.5, 0.5);
  const auto candidates = Samples(2.3, -2.3, 1.0);
  LaneAssociationResult result;

  ASSERT_TRUE(
      AssociateLaneSamples(history, candidates, Policy(), 1.0, &result).ok());
  EXPECT_DOUBLE_EQ(2.3, result.samples.at(1).left.y);
  EXPECT_DOUBLE_EQ(-2.3, result.samples.at(1).right.y);
}

TEST(LaneAssociationTest, RejectsDisplacementAndDisconnectedHistory) {
  const auto history = Samples(2.5, -2.5, 0.5);
  LaneAssociationResult result;
  EXPECT_FALSE(AssociateLaneSamples(history, Samples(3.1, -3.1, 1.0), Policy(),
                                    1.0, &result)
                   .ok());
  EXPECT_TRUE(result.samples.empty());

  std::map<int, TrackedLaneSample> disconnected = Samples(2.5, -2.5, 0.5);
  std::map<int, TrackedLaneSample> candidates;
  for (const auto& sample : Samples(2.5, -2.5, 1.0)) {
    candidates.emplace(sample.first + 10, sample.second);
  }
  EXPECT_FALSE(
      AssociateLaneSamples(disconnected, candidates, Policy(), 1.0, &result)
          .ok());
  EXPECT_TRUE(result.samples.empty());
}

}  // namespace
}  // namespace world_model
}  // namespace apollo
