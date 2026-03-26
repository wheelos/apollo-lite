/******************************************************************************
 * Copyright 2026 The Wheel.OS Authors. All Rights Reserved.
 *****************************************************************************/
#include "modules/perception/traffic_light/domain/traffic_light_result.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace apollo {
namespace perception {
namespace traffic_light {
namespace domain {

TrafficLightDecision AggregateTrafficLightDecision(
    std::vector<base::TrafficLightPtr>* lights) {
  TrafficLightDecision decision;
  if (lights == nullptr || lights->empty()) {
    return decision;
  }

  int max_red_id = -1;
  int max_green_id = -1;
  int max_yellow_id = -1;
  int max_unknown_id = -1;
  double max_red_conf = 0.0;
  double max_green_conf = 0.0;
  double max_yellow_conf = 0.0;

  for (int i = 0; i < static_cast<int>(lights->size()); ++i) {
    auto& light = lights->at(i);
    switch (light->status.color) {
      case base::TLColor::TL_RED:
        if (std::abs(light->status.confidence) <
            std::numeric_limits<double>::min()) {
          light->status.color = base::TLColor::TL_UNKNOWN_COLOR;
          max_unknown_id = i;
          break;
        }
        decision.vote_stats.red += light->status.confidence;
        if (light->status.confidence >= max_red_conf) {
          max_red_conf = light->status.confidence;
          max_red_id = i;
        }
        break;
      case base::TLColor::TL_GREEN:
        if (std::abs(light->status.confidence) <
            std::numeric_limits<double>::min()) {
          light->status.color = base::TLColor::TL_UNKNOWN_COLOR;
          max_unknown_id = i;
          break;
        }
        decision.vote_stats.green += light->status.confidence;
        if (light->status.confidence >= max_green_conf) {
          max_green_conf = light->status.confidence;
          max_green_id = i;
        }
        break;
      case base::TLColor::TL_YELLOW:
        if (std::abs(light->status.confidence) <
            std::numeric_limits<double>::min()) {
          light->status.color = base::TLColor::TL_UNKNOWN_COLOR;
          max_unknown_id = i;
          break;
        }
        decision.vote_stats.yellow += light->status.confidence;
        if (light->status.confidence >= max_yellow_conf) {
          max_yellow_conf = light->status.confidence;
          max_yellow_id = i;
        }
        break;
      case base::TLColor::TL_UNKNOWN_COLOR:
      default:
        decision.vote_stats.unknown += light->status.confidence;
        max_unknown_id = i;
        break;
    }
  }

  if (decision.vote_stats.red >= decision.vote_stats.green &&
      decision.vote_stats.red >= decision.vote_stats.yellow &&
      decision.vote_stats.red > 0) {
    decision.vote_stats.dominant_index = max_red_id;
  } else if (decision.vote_stats.yellow > decision.vote_stats.red &&
             decision.vote_stats.yellow >= decision.vote_stats.green) {
    decision.vote_stats.dominant_index = max_yellow_id;
  } else if (decision.vote_stats.green > decision.vote_stats.red &&
             decision.vote_stats.green > decision.vote_stats.yellow) {
    decision.vote_stats.dominant_index = max_green_id;
  } else {
    decision.vote_stats.dominant_index = max_unknown_id;
  }

  if (decision.vote_stats.dominant_index > 0) {
    std::swap(lights->at(0), lights->at(decision.vote_stats.dominant_index));
    decision.vote_stats.dominant_index = 0;
  }

  if (!lights->empty()) {
    decision.contain_lights = true;
    decision.dominant_color = lights->at(0)->status.color;
    decision.dominant_confidence = lights->at(0)->status.confidence;
  }
  return decision;
}

}  // namespace domain
}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
