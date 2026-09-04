#include "modules/collision_guardian/core/risk_filter.h"

#include "gtest/gtest.h"

namespace apollo {
namespace collision_guardian {

TEST(RiskFilterTest, ConfirmsAndReleasesWithHysteresis) {
  RiskFilterConfig config;
  config.min_confirmation_frames = 3;
  config.min_release_frames = 2;
  RiskFilter filter(config);

  EXPECT_EQ(filter.Update(true, true).state, FilterState::kSuspected);
  EXPECT_EQ(filter.Update(true, true).state, FilterState::kSuspected);
  EXPECT_EQ(filter.Update(true, true).state, FilterState::kConfirmed);

  FilterState state = FilterState::kConfirmed;
  for (int i = 0; i < 20 && state == FilterState::kConfirmed; ++i) {
    state = filter.Update(false, true).state;
  }
  EXPECT_EQ(state, FilterState::kReleasing);
  EXPECT_EQ(filter.Update(false, true).state, FilterState::kClear);
}

TEST(RiskFilterTest, InvalidInputProducesFault) {
  RiskFilter filter(RiskFilterConfig{});
  EXPECT_EQ(filter.Update(false, false).state, FilterState::kFault);
  EXPECT_EQ(filter.Update(false, true).state, FilterState::kClear);
}

}  // namespace collision_guardian
}  // namespace apollo
