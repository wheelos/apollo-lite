/******************************************************************************
 * Copyright 2026 The Wheel.OS Authors. All Rights Reserved.
 *****************************************************************************/

#include "modules/perception/traffic_light/domain/traffic_light_result.h"

#include "gtest/gtest.h"

namespace apollo {
namespace perception {
namespace traffic_light {
namespace domain {

TEST(TrafficLightResultTest, AggregateDominantColorAndSwap) {
  std::vector<base::TrafficLightPtr> lights;
  base::TrafficLightPtr green(new base::TrafficLight);
  green->id = "g";
  green->status.color = base::TLColor::TL_GREEN;
  green->status.confidence = 0.9;
  lights.push_back(green);

  base::TrafficLightPtr red(new base::TrafficLight);
  red->id = "r";
  red->status.color = base::TLColor::TL_RED;
  red->status.confidence = 0.95;
  lights.push_back(red);

  TrafficLightDecision decision = AggregateTrafficLightDecision(&lights);
  EXPECT_TRUE(decision.contain_lights);
  EXPECT_EQ(decision.dominant_color, base::TLColor::TL_RED);
  EXPECT_EQ(lights.front()->id, "r");
}

TEST(TrafficLightResultTest, EmptyInput) {
  std::vector<base::TrafficLightPtr> lights;
  TrafficLightDecision decision = AggregateTrafficLightDecision(&lights);
  EXPECT_FALSE(decision.contain_lights);
  EXPECT_EQ(decision.dominant_color, base::TLColor::TL_UNKNOWN_COLOR);
}

}  // namespace domain
}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
