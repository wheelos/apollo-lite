/******************************************************************************
 * Copyright 2026 The Wheel.OS Authors. All Rights Reserved.
 *****************************************************************************/
#pragma once

#include <vector>

#include "modules/perception/base/traffic_light.h"

namespace apollo {
namespace perception {
namespace traffic_light {
namespace domain {

struct TrafficLightVoteStats {
  double red = 0.0;
  double green = 0.0;
  double yellow = 0.0;
  double unknown = 0.0;
  int dominant_index = -1;
};

struct TrafficLightDecision {
  bool contain_lights = false;
  base::TLColor dominant_color = base::TLColor::TL_UNKNOWN_COLOR;
  double dominant_confidence = 0.0;
  TrafficLightVoteStats vote_stats;
};

TrafficLightDecision AggregateTrafficLightDecision(
    std::vector<base::TrafficLightPtr>* lights);

}  // namespace domain
}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
