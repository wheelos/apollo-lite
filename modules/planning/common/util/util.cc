/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/planning/common/util/util.h"

#include <limits>
#include <vector>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {
namespace util {

using apollo::common::VehicleState;
using apollo::hdmap::PathOverlap;
using apollo::routing::RoutingResponse;

namespace {

RoutingResponse NormalizeRouting(const RoutingResponse& routing) {
  RoutingResponse normalized;
  for (const auto& road : routing.road()) {
    normalized.add_road()->CopyFrom(road);
  }
  if (routing.has_routing_request()) {
    auto* request = normalized.mutable_routing_request();
    request->CopyFrom(routing.routing_request());
    request->clear_header();
  }
  if (routing.has_map_version()) {
    normalized.set_map_version(routing.map_version());
  }
  return normalized;
}

apollo::routing::RoutingRequest NormalizeRoutingRequest(
    const RoutingResponse& routing) {
  apollo::routing::RoutingRequest normalized;
  if (routing.has_routing_request()) {
    normalized.CopyFrom(routing.routing_request());
    normalized.clear_header();
  }
  return normalized;
}

}  // namespace

bool IsVehicleStateValid(const VehicleState& vehicle_state) {
  if (std::isnan(vehicle_state.x()) || std::isnan(vehicle_state.y()) ||
      std::isnan(vehicle_state.z()) || std::isnan(vehicle_state.heading()) ||
      std::isnan(vehicle_state.kappa()) ||
      std::isnan(vehicle_state.linear_velocity()) ||
      std::isnan(vehicle_state.linear_acceleration())) {
    return false;
  }
  return true;
}

bool IsDifferentRouting(const RoutingResponse& first,
                        const RoutingResponse& second) {
  const auto normalized_first = NormalizeRouting(first);
  const auto normalized_second = NormalizeRouting(second);
  return normalized_first.SerializeAsString() !=
         normalized_second.SerializeAsString();
}

bool HasSameRoutingRequest(const RoutingResponse& first,
                           const RoutingResponse& second) {
  const auto normalized_first = NormalizeRoutingRequest(first);
  const auto normalized_second = NormalizeRoutingRequest(second);
  return normalized_first.SerializeAsString() ==
         normalized_second.SerializeAsString();
}

bool HasParkingRoutingCommand(const RoutingResponse& routing_response) {
  if (!routing_response.has_routing_request() ||
      !routing_response.routing_request().has_parking_info()) {
    return false;
  }

  const auto& parking_info = routing_response.routing_request().parking_info();
  return (parking_info.has_parking_space_id() &&
          !parking_info.parking_space_id().empty()) ||
         (parking_info.has_corner_point() &&
          parking_info.corner_point().point_size() > 0);
}

bool HasParkingSpaceIdRoutingCommand(const RoutingResponse& routing_response) {
  return routing_response.has_routing_request() &&
         routing_response.routing_request().has_parking_info() &&
         routing_response.routing_request()
             .parking_info()
             .has_parking_space_id() &&
         !routing_response.routing_request()
              .parking_info()
              .parking_space_id()
              .empty();
}

bool SupportsDirectValetParkingEntry(const ScenarioConfig& config) {
  return config.stage_type_size() == 1 &&
         config.stage_type(0) == StageType::VALET_PARKING_PARKING;
}

bool ShouldUseDirectValetParkingMode(
    const bool supports_direct_valet_parking_entry,
    const std::shared_ptr<RoutingResponse>& routing_response) {
  return supports_direct_valet_parking_entry && routing_response != nullptr &&
         HasParkingSpaceIdRoutingCommand(*routing_response);
}

double GetADCStopDeceleration(
    apollo::common::VehicleStateProvider* vehicle_state,
    const double adc_front_edge_s, const double stop_line_s) {
  double adc_speed = vehicle_state->linear_velocity();
  const double max_adc_stop_speed = common::VehicleConfigHelper::Instance()
                                        ->GetConfig()
                                        .vehicle_param()
                                        .max_abs_speed_when_stopped();
  if (adc_speed < max_adc_stop_speed) {
    return 0.0;
  }

  double stop_distance = 0;

  if (stop_line_s > adc_front_edge_s) {
    stop_distance = stop_line_s - adc_front_edge_s;
  }
  if (stop_distance < 1e-5) {
    return std::numeric_limits<double>::max();
  }
  return (adc_speed * adc_speed) / (2 * stop_distance);
}

/*
 * @brief: check if a stop_sign_overlap is still along reference_line
 */
bool CheckStopSignOnReferenceLine(const ReferenceLineInfo& reference_line_info,
                                  const std::string& stop_sign_overlap_id) {
  const std::vector<PathOverlap>& stop_sign_overlaps =
      reference_line_info.reference_line().map_path().stop_sign_overlaps();
  auto stop_sign_overlap_it =
      std::find_if(stop_sign_overlaps.begin(), stop_sign_overlaps.end(),
                   [&stop_sign_overlap_id](const PathOverlap& overlap) {
                     return overlap.object_id == stop_sign_overlap_id;
                   });
  return (stop_sign_overlap_it != stop_sign_overlaps.end());
}

/*
 * @brief: check if a traffic_light_overlap is still along reference_line
 */
bool CheckTrafficLightOnReferenceLine(
    const ReferenceLineInfo& reference_line_info,
    const std::string& traffic_light_overlap_id) {
  const std::vector<PathOverlap>& traffic_light_overlaps =
      reference_line_info.reference_line().map_path().signal_overlaps();
  auto traffic_light_overlap_it =
      std::find_if(traffic_light_overlaps.begin(), traffic_light_overlaps.end(),
                   [&traffic_light_overlap_id](const PathOverlap& overlap) {
                     return overlap.object_id == traffic_light_overlap_id;
                   });
  return (traffic_light_overlap_it != traffic_light_overlaps.end());
}

/*
 * @brief: check if ADC is till inside a pnc-junction
 */
bool CheckInsideJunction(const ReferenceLineInfo& reference_line_info) {
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double adc_back_edge_s = reference_line_info.AdcSlBoundary().start_s();

  hdmap::PathOverlap junction_overlap;
  reference_line_info.GetJunction(adc_front_edge_s, &junction_overlap);
  if (junction_overlap.object_id.empty()) {
    return false;
  }

  static constexpr double kIntersectionPassDist = 2.0;  // unit: m
  const double distance_adc_pass_intersection =
      adc_back_edge_s - junction_overlap.end_s;
  ADEBUG << "distance_adc_pass_intersection[" << distance_adc_pass_intersection
         << "] junction_overlap[" << junction_overlap.object_id << "] start_s["
         << junction_overlap.start_s << "]";

  return distance_adc_pass_intersection < kIntersectionPassDist;
}

/*
 * @brief: get files at a path
 */
void GetFilesByPath(const boost::filesystem::path& path,
                    std::vector<std::string>* files) {
  ACHECK(files);
  if (!boost::filesystem::exists(path)) {
    return;
  }
  if (boost::filesystem::is_regular_file(path)) {
    AINFO << "Found record file: " << path.c_str();
    files->push_back(path.c_str());
    return;
  }
  if (boost::filesystem::is_directory(path)) {
    for (auto& entry : boost::make_iterator_range(
             boost::filesystem::directory_iterator(path), {})) {
      GetFilesByPath(entry.path(), files);
    }
  }
}

}  // namespace util
}  // namespace planning
}  // namespace apollo
