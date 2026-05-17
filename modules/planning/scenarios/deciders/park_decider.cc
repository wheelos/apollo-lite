// Copyright 2025 WheelOS All Rights Reserved.
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

//  Created Date: 2025-12-07
//  Author: daohu527

#include "modules/planning/scenarios/deciders/park_decider.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/util/point_factory.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/pnc_map/path.h"

namespace {

double ComputePullOverPreparationDistance(
    const apollo::planning::ScenarioPullOverConfig& config) {
  const auto& vehicle_param =
      apollo::common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param();
  return vehicle_param.front_edge_to_center() +
         config.s_distance_to_stop_for_open_space_parking() +
         config.max_valid_stop_distance();
}

double ComputeEffectivePullOverMinDistance(
    const apollo::planning::ScenarioPullOverConfig& config) {
  return std::max({config.pull_over_min_distance_buffer(),
                   config.max_distance_stop_search(),
                   ComputePullOverPreparationDistance(config)});
}

}  // namespace

namespace apollo {
namespace planning {
namespace scenario {

using apollo::hdmap::HDMapUtil;

ScenarioDecisionResult ParkDecider::MakeDecision(
    const DeciderContext& context) {
  // 1. Valet Parking (Priority 1)
  auto decision = CheckValetParking(context);
  if (decision.IsValid()) return decision;

  // 2. Pull Over (Priority 2)
  decision = CheckPullOver(context);
  if (decision.IsValid()) return decision;

  // 3. Park And Go (Priority 3)
  decision = CheckParkAndGo(context);
  if (decision.IsValid()) return decision;

  return ScenarioDecisionResult();
}

ScenarioDecisionResult ParkDecider::CheckValetParking(
    const DeciderContext& context) {
  const auto& frame = context.frame;

  // 0. Load Config
  const auto& config = config_.valet_parking_config();
  uint32_t scenario_entry_score = config.scenario_entry_score();
  double parking_spot_range_to_start = config.parking_spot_range_to_start();

  // 1. Sticky Strategy
  // Valet Parking is a complex maneuver, ensure continuity.
  const auto current_scenario = context.current_scenario;
  if (current_scenario->Type() == ScenarioType::VALET_PARKING) {
    if (current_scenario->GetStatus() !=
        Scenario::ScenarioStatus::STATUS_DONE) {
      return ScenarioDecisionResult(
          ScenarioType::VALET_PARKING, ScenarioGrade::MISSION,
          scenario_entry_score, "Valet Parking In Progress (Sticky)");
    }
  }

  // 2. Routing Check
  const auto& routing = frame->local_view().routing;
  if (!routing || !routing->routing_request().has_parking_info()) {
    return ScenarioDecisionResult();
  }

  const auto& parking_info = routing->routing_request().parking_info();
  if (!parking_info.has_parking_space_id() ||
      parking_info.parking_space_id().empty()) {
    return ScenarioDecisionResult();
  }

  std::string target_parking_spot_id = parking_info.parking_space_id();

  // 3. Map Path Check
  if (frame->reference_line_info().empty()) {
    return ScenarioDecisionResult();
  }
  const auto& nearby_path =
      frame->reference_line_info().front().reference_line().map_path();

  hdmap::PathOverlap parking_space_overlap;

  // 4. Search Target
  if (!SearchTargetParkingSpotOnPath(nearby_path, target_parking_spot_id,
                                     &parking_space_overlap)) {
    return ScenarioDecisionResult();
  }

  // 5. Distance Check
  if (!CheckDistanceToParkingSpot(frame, nearby_path,
                                  parking_spot_range_to_start,
                                  parking_space_overlap)) {
    return ScenarioDecisionResult();
  }

  return ScenarioDecisionResult(ScenarioType::VALET_PARKING,
                                ScenarioGrade::MISSION, scenario_entry_score,
                                "Target Parking Spot Found & Within Range");
}

bool ParkDecider::SearchTargetParkingSpotOnPath(
    const hdmap::Path& nearby_path, const std::string& target_parking_id,
    hdmap::PathOverlap* parking_space_overlap) {
  const auto& parking_space_overlaps = nearby_path.parking_space_overlaps();
  for (const auto& parking_overlap : parking_space_overlaps) {
    if (parking_overlap.object_id == target_parking_id) {
      *parking_space_overlap = parking_overlap;
      return true;
    }
  }
  return false;
}

bool ParkDecider::CheckDistanceToParkingSpot(
    const Frame* frame, const hdmap::Path& nearby_path,
    const double parking_start_range,
    const hdmap::PathOverlap& parking_space_overlap) {
  // 1. Get Parking Spot from HDMap
  const hdmap::HDMap* hdmap = HDMapUtil::BaseMapPtr();
  hdmap::Id id;
  id.set_id(parking_space_overlap.object_id);
  auto target_parking_spot_ptr = hdmap->GetParkingSpaceById(id);
  if (!target_parking_spot_ptr) {
    ADEBUG << "Could not find parking spot in HDMap: "
           << parking_space_overlap.object_id;
    return false;
  }

  // 2. Get initial corner points from Map
  common::math::Vec2d left_bottom_point =
      target_parking_spot_ptr->polygon().points().at(0);
  common::math::Vec2d right_bottom_point =
      target_parking_spot_ptr->polygon().points().at(1);

  // 3. Override with Routing info if available (Crucial for Cloud-based
  // parking)
  const auto& routing = frame->local_view().routing;
  if (routing && routing->routing_request().has_parking_info()) {
    const auto& p_info = routing->routing_request().parking_info();
    if (p_info.has_corner_point() && p_info.corner_point().point_size() >= 2) {
      left_bottom_point.set_x(p_info.corner_point().point(0).x());
      left_bottom_point.set_y(p_info.corner_point().point(0).y());
      right_bottom_point.set_x(p_info.corner_point().point(1).x());
      right_bottom_point.set_y(p_info.corner_point().point(1).y());
    }
  }

  // 4. Project points to Path to get s
  double lb_s = 0.0, lb_l = 0.0;
  double rb_s = 0.0, rb_l = 0.0;
  nearby_path.GetNearestPoint(left_bottom_point, &lb_s, &lb_l);
  nearby_path.GetNearestPoint(right_bottom_point, &rb_s, &rb_l);

  double parking_space_center_s = (lb_s + rb_s) / 2.0;

  // 5. Get ADC s
  const auto& vehicle_state = frame->vehicle_state();
  double vehicle_s = 0.0, vehicle_l = 0.0;
  common::math::Vec2d vehicle_vec(vehicle_state.x(), vehicle_state.y());
  nearby_path.GetNearestPoint(vehicle_vec, &vehicle_s, &vehicle_l);

  // 6. Final Check
  // Using abs() as in original code to allow some tolerance
  double dist = std::abs(parking_space_center_s - vehicle_s);

  ADEBUG << "Valet Distance Check: CenterS=" << parking_space_center_s
         << " VehicleS=" << vehicle_s << " Diff=" << dist
         << " Range=" << parking_start_range;

  return (dist < parking_start_range);
}

ScenarioDecisionResult ParkDecider::CheckPullOver(
    const DeciderContext& context) {
  const auto& frame = context.frame;
  const auto& overlaps = context.first_encountered_overlaps;

  // 3. Load Configuration
  const auto& config = config_.pull_over_config();
  const double min_dist = ComputeEffectivePullOverMinDistance(config);
  const double max_dist = config.start_pull_over_scenario_distance();
  const double stop_search_dist = config.max_distance_stop_search();
  const double junction_buffer = config.avoid_junction_distance();
  const uint32_t scenario_entry_score = config.scenario_entry_score();

  // 2. Pre-conditions Check
  // Must be in a single lane (not changing lanes) and have valid routing.
  if (frame->reference_line_info().size() != 1) {
    return ScenarioDecisionResult();
  }

  const auto& routing = frame->local_view().routing;
  if (!routing || routing->routing_request().waypoint().empty()) {
    return ScenarioDecisionResult();
  }

  // 4. Distance Calculation
  const auto& reference_line_info = frame->reference_line_info().front();
  const auto& reference_line = reference_line_info.reference_line();
  const auto& routing_end = *(routing->routing_request().waypoint().rbegin());

  common::SLPoint dest_sl;
  reference_line.XYToSL(routing_end.pose(), &dest_sl);
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double dist_to_dest = dest_sl.s() - adc_front_edge_s;

  // 1. Sticky Strategy
  // Keep PullOver once a valid target is established, or while the destination
  // is still far enough ahead that the pull-over path search remains feasible.
  const auto current_scenario = context.current_scenario;
  if (current_scenario->Type() == ScenarioType::PULL_OVER &&
      current_scenario->GetStatus() != Scenario::ScenarioStatus::STATUS_DONE) {
    const auto& pull_over_status =
        injector_->planning_context()->planning_status().pull_over();
    if (pull_over_status.has_position() || dist_to_dest >= min_dist) {
      return ScenarioDecisionResult(
          ScenarioType::PULL_OVER, ScenarioGrade::MISSION, scenario_entry_score,
          "Pull Over In Progress (Sticky)");
    }
    ADEBUG << "PullOver no longer feasible before target selection. distance["
           << dist_to_dest << "] effective_min_distance[" << min_dist << "]";
  }

  // Check if the vehicle is within the operational range
  if (dist_to_dest < min_dist || dist_to_dest > max_dist) {
    return ScenarioDecisionResult();
  }

  // 5. Abort Logic (Critical Safety)
  // If we are too close to the destination but Perception hasn't found a
  // spot yet, we should abort PullOver to prevent stopping in the middle of
  // the road.
  if (dist_to_dest < stop_search_dist) {
    const auto& planning_status =
        injector_->planning_context()->planning_status();
    const auto& pull_over_status = planning_status.pull_over();
    if (!pull_over_status.has_position()) {
      ADEBUG << "Too close to destination (" << dist_to_dest
             << "m) and no parking spot found. Aborting PullOver.";
      return ScenarioDecisionResult();
    }
  }

  // 6. Junction/Signal Avoidance Check
  // Do not pull over if the destination is inside or too close to a
  // junction.
  auto check_overlap = [&](ReferenceLineInfo::OverlapType type) {
    if (overlaps->count(type)) {
      const auto& overlap = overlaps->at(type);
      double d_to_start = overlap.start_s - dest_sl.s();
      double d_passed_end = dest_sl.s() - overlap.end_s;

      // Check if destination is near the start or end of the overlap
      if ((d_to_start > 0 && d_to_start < junction_buffer) ||
          (d_passed_end > 0 && d_passed_end < junction_buffer)) {
        return true;
      }
      // Note: Strict "Inside" check (start < s < end) is implicitly handled
      // by the logic above or can be added if strict exclusion is needed.
    }
    return false;
  };

  const bool near_junction =
      check_overlap(ReferenceLineInfo::PNC_JUNCTION) ||
      check_overlap(ReferenceLineInfo::SIGNAL) ||
      check_overlap(ReferenceLineInfo::STOP_SIGN) ||
      check_overlap(ReferenceLineInfo::YIELD_SIGN);
  if (near_junction) {
    return ScenarioDecisionResult();
  }

  // 7. Rightmost Lane Check
  // Ensure we are pulling over from the rightmost driving lane to avoid
  // crossing traffic.
  bool rightmost_driving_lane = true;
  double check_s = adc_front_edge_s;
  const double kStep = 5.0;

  while (check_s < dest_sl.s()) {
    check_s += kStep;
    std::vector<hdmap::LaneInfoConstPtr> lanes;
    reference_line.GetLaneFromS(check_s, &lanes);

    if (lanes.empty()) continue;

    const auto& lane = lanes[0];
    for (const auto& neighbor_id :
         lane->lane().right_neighbor_forward_lane_id()) {
      auto neighbor_ptr = HDMapUtil::BaseMapPtr()->GetLaneById(neighbor_id);
      if (!neighbor_ptr) continue;

      // If the right neighbor is a normal driving lane, we are not on the
      // edge.
      if (neighbor_ptr->lane().type() == hdmap::Lane::CITY_DRIVING) {
        rightmost_driving_lane = false;
        break;
      }
    }
    if (!rightmost_driving_lane) break;
  }

  if (!rightmost_driving_lane) {
    return ScenarioDecisionResult();
  }

  // 8. Success: Generate positive decision
  // We assign MISSION grade to PullOver.
  return ScenarioDecisionResult(ScenarioType::PULL_OVER, ScenarioGrade::MISSION,
                                scenario_entry_score,
                                "Destination Approach & Safe to PullOver");
}

ScenarioDecisionResult ParkDecider::CheckParkAndGo(
    const DeciderContext& context) {
  const auto& frame = context.frame;

  // 0. Load Configs
  // Use global vehicle config for consistency
  const double max_abs_speed_when_stopped =
      common::VehicleConfigHelper::Instance()
          ->GetConfig()
          .vehicle_param()
          .max_abs_speed_when_stopped();

  // Use ParkAndGo specific config
  const auto& config = config_.park_and_go_config();
  const double min_dist_to_dest = config.min_dist_to_dest();
  const uint32_t scenario_entry_score = config.scenario_entry_score();

  // 1. Sticky Strategy
  const auto current_scenario = context.current_scenario;
  if (current_scenario->Type() == ScenarioType::PARK_AND_GO) {
    if (current_scenario->GetStatus() !=
        Scenario::ScenarioStatus::STATUS_DONE) {
      return ScenarioDecisionResult(
          ScenarioType::PARK_AND_GO, ScenarioGrade::MANEUVER,
          scenario_entry_score, "ParkAndGo In Progress (Sticky)");
    }
  }

  // 2. Speed Check: Must be stationary (to enter)
  const auto vehicle_state = injector_->vehicle_state()->vehicle_state();
  double adc_speed = std::abs(vehicle_state.linear_velocity());

  if (adc_speed > max_abs_speed_when_stopped) {
    return ScenarioDecisionResult();
  }

  // 3. Destination Check
  if (frame->reference_line_info().empty()) {
    return ScenarioDecisionResult();
  }

  const auto& routing = frame->local_view().routing;
  if (!routing || routing->routing_request().waypoint().empty())
    return ScenarioDecisionResult();

  const auto& reference_line_info = frame->reference_line_info().front();
  const auto& routing_end = *(routing->routing_request().waypoint().rbegin());

  common::SLPoint dest_sl;
  reference_line_info.reference_line().XYToSL(routing_end.pose(), &dest_sl);

  double dist_to_dest =
      dest_sl.s() - reference_line_info.AdcSlBoundary().end_s();

  if (dist_to_dest < min_dist_to_dest) {
    return ScenarioDecisionResult();
  }

  // 4. Map Location Check
  auto adc_point = common::util::PointFactory::ToPointENU(vehicle_state);
  hdmap::LaneInfoConstPtr lane;
  double s = 0.0, l = 0.0;

  int ret = HDMapUtil::BaseMap().GetNearestLaneWithHeading(
      adc_point, 2.0, vehicle_state.heading(), M_PI / 3.0, &lane, &s, &l);

  bool is_off_lane = (ret != 0);
  // Check if lane type is NOT CITY_DRIVING (e.g., BIKING, SIDEWALK,
  // PARKING)
  bool is_non_city_driving =
      (!is_off_lane && lane->lane().type() != hdmap::Lane::CITY_DRIVING);

  if (is_off_lane || is_non_city_driving) {
    return ScenarioDecisionResult(ScenarioType::PARK_AND_GO,
                                  ScenarioGrade::MANEUVER, scenario_entry_score,
                                  "Stopped Off-Road/Curb");
  }

  return ScenarioDecisionResult();
}

}  // namespace scenario
}  // namespace planning
}  // namespace apollo
