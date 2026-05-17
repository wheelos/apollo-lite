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

/**
 * @file
 **/

#include "modules/planning/scenarios/park/valet_parking/valet_parking_scenario.h"

#include "modules/planning/common/util/common.h"
#include "modules/planning/common/util/util.h"
#include "modules/planning/scenarios/park/valet_parking/stage_parking.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace valet_parking {

using apollo::common::VehicleState;
using apollo::common::math::Vec2d;
using apollo::hdmap::ParkingSpaceInfoConstPtr;

apollo::common::util::Factory<
    StageType, Stage,
    Stage* (*)(const ScenarioConfig::StageConfig& stage_config,
               const std::shared_ptr<DependencyInjector>& injector)>
    ValetParkingScenario::s_stage_factory_;

void ValetParkingScenario::Init() {
  if (init_) {
    return;
  }

  Scenario::Init();

  if (!GetScenarioConfig()) {
    AERROR << "fail to get scenario specific config";
    return;
  }

  hdmap_ = hdmap::HDMapUtil::BaseMapPtr();
  CHECK_NOTNULL(hdmap_);
}

void ValetParkingScenario::RegisterStages() {
  if (s_stage_factory_.Empty()) {
    s_stage_factory_.Clear();
  }
  s_stage_factory_.Register(
      StageType::VALET_PARKING_PARKING,
      [](const ScenarioConfig::StageConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Stage* {
        return new StageParking(config, injector);
      });
}

std::unique_ptr<Stage> ValetParkingScenario::CreateStage(
    const ScenarioConfig::StageConfig& stage_config,
    const std::shared_ptr<DependencyInjector>& injector) {
  if (s_stage_factory_.Empty()) {
    RegisterStages();
  }
  auto ptr = s_stage_factory_.CreateObjectOrNull(stage_config.stage_type(),
                                                 stage_config, injector);
  if (ptr) {
    ptr->SetContext(&context_);
  }
  return ptr;
}

bool ValetParkingScenario::GetScenarioConfig() {
  if (!config_.has_valet_parking_config()) {
    AERROR << "miss scenario specific config";
    return false;
  }
  context_.scenario_config.CopyFrom(config_.valet_parking_config());
  return true;
}

bool ValetParkingScenario::SupportsDirectParkingEntry(
    const ScenarioConfig& config) {
  return util::SupportsDirectValetParkingEntry(config);
}

bool ValetParkingScenario::HasParkingRoutingCommand(
    const routing::RoutingResponse& routing_response) {
  return util::HasParkingRoutingCommand(routing_response);
}

bool ValetParkingScenario::IsTransferable(const Frame& frame,
                                          const double parking_start_range) {
  // TODO(all) Implement available parking spot detection by preception results
  std::string target_parking_spot_id;
  bool has_parking_info_from_routing = false;

  const auto& routing_request = frame.local_view().routing->routing_request();
  if (routing_request.has_parking_info()) {
    const auto& parking_info = routing_request.parking_info();
    if (parking_info.has_parking_space_id()) {
      target_parking_spot_id = parking_info.parking_space_id();
    }
    if (parking_info.has_corner_point() &&
        parking_info.corner_point().point_size() > 0) {
      has_parking_info_from_routing = true;
    }
  }

  if (target_parking_spot_id.empty() && !has_parking_info_from_routing) {
    ADEBUG << "No parking space id or corner points from routing";
    return false;
  }

  ParkingSpaceInfoConstPtr target_parking_spot;
  const auto& vehicle_state = frame.vehicle_state();

  bool found_parking_spot_in_map = false;
  if (!target_parking_spot_id.empty()) {
    found_parking_spot_in_map =
        GetTargetParkingSpotById(target_parking_spot_id, &target_parking_spot);
  }

  if (!found_parking_spot_in_map && !has_parking_info_from_routing) {
    ADEBUG << "No such parking spot found in HDMap and no routing corner "
              "points, parking_space_id: "
           << target_parking_spot_id;
    return false;
  }

  if (!CheckDistanceToParkingSpot(frame, vehicle_state, parking_start_range,
                                  target_parking_spot)) {
    ADEBUG << "target parking spot found, but euclidean distance is larger "
              "than configured threshold. parking_space_id: "
           << target_parking_spot_id;
    return false;
  }

  return true;
}

bool ValetParkingScenario::GetTargetParkingSpotById(
    const std::string& target_parking_id,
    ParkingSpaceInfoConstPtr* target_parking_spot) {
  const hdmap::HDMap* hdmap = hdmap::HDMapUtil::BaseMapPtr();
  CHECK_NOTNULL(hdmap);
  hdmap::Id id;
  id.set_id(target_parking_id);
  *target_parking_spot = hdmap->GetParkingSpaceById(id);
  return *target_parking_spot != nullptr;
}

bool ValetParkingScenario::CheckDistanceToParkingSpot(
    const Frame& frame, const VehicleState& vehicle_state,
    const double parking_start_range,
    const ParkingSpaceInfoConstPtr& target_parking_spot) {
  Vec2d parking_spot_center;
  bool has_center = apollo::planning::util::GetParkingSpotCenterFromRouting(
      frame, &parking_spot_center);

  if (!has_center) {
    if (target_parking_spot == nullptr) {
      AERROR << "No parking spot found in map and no routing info available.";
      return false;
    }
    if (target_parking_spot->polygon().points().empty()) {
      ADEBUG << "parking spot polygon is empty";
      return false;
    }
    parking_spot_center = apollo::planning::util::GetParkingSpotCenterFromMap(
        target_parking_spot);
  }

  const Vec2d vehicle_vec(vehicle_state.x(), vehicle_state.y());
  const double distance_to_parking_spot =
      vehicle_vec.DistanceTo(parking_spot_center);
  ADEBUG << "distance_to_parking_spot[" << distance_to_parking_spot
         << "] parking_start_range[" << parking_start_range << "]";

  return distance_to_parking_spot < parking_start_range;
}

}  // namespace valet_parking
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
