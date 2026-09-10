// Copyright 2026 WheelOS. All Rights Reserved.
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

#pragma once

#include <memory>
#include <mutex>

#include "modules/open_space_planning/proto/open_space_planning.pb.h"
#include "wheelos_msgs/chassis_msgs/chassis.pb.h"
#include "wheelos_msgs/localization_msgs/localization.pb.h"
#include "wheelos_msgs/planning_msgs/planning.pb.h"
#include "wheelos_msgs/prediction_msgs/prediction_obstacle.pb.h"
#include "wheelos_msgs/routing_msgs/routing.pb.h"

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/open_space_planning/common/types.h"
#include "modules/open_space_planning/runtime/open_space_planner.h"

namespace apollo {
namespace open_space_planning {

class OpenSpacePlanningComponent final
    : public cyber::Component<prediction::PredictionObstacles, canbus::Chassis,
                              localization::LocalizationEstimate> {
 public:
  bool Init() override;

  bool Proc(const std::shared_ptr<prediction::PredictionObstacles>& prediction,
            const std::shared_ptr<canbus::Chassis>& chassis,
            const std::shared_ptr<localization::LocalizationEstimate>&
                localization) override;

 private:
  bool BuildProblem(const prediction::PredictionObstacles& prediction,
                    const canbus::Chassis& chassis,
                    const localization::LocalizationEstimate& localization,
                    PlanningProblem* problem) const;
  void Publish(const PlanningResult& result, const Status& status);

  std::shared_ptr<cyber::Reader<proto::GridMap>> grid_map_reader_;
  std::shared_ptr<cyber::Reader<proto::VehicleModel>> vehicle_model_reader_;
  std::shared_ptr<cyber::Reader<routing::RoutingRequest>> routing_reader_;
  std::shared_ptr<cyber::Writer<proto::OpenSpacePlanningResult>> result_writer_;
  std::shared_ptr<cyber::Writer<planning::ADCTrajectory>> trajectory_writer_;

  mutable std::mutex mutex_;
  proto::GridMap grid_map_;
  proto::VehicleModel vehicle_model_;
  routing::RoutingRequest routing_request_;
  bool has_grid_map_ = false;
  bool has_vehicle_model_ = false;
  bool has_routing_request_ = false;

  std::unique_ptr<OpenSpacePlanner> planner_;
};

CYBER_REGISTER_COMPONENT(OpenSpacePlanningComponent)

}  // namespace open_space_planning
}  // namespace apollo
