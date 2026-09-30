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

#pragma once

#include <string>

#include "modules/common/status/status.h"
#include "modules/world_model/local_map/local_scene.h"
#include "modules/world_model/local_map/temporal_lane_map.h"

namespace apollo {
namespace world_model {

// Owns source admission, epoch/time lifecycle and routes typed module outputs
// into the scene assembler. Geometry/tracking algorithms live in their modules.
class LocalSceneBuilder {
 public:
  LocalSceneBuilder(const TemporalLanePolicy& policy, std::string odom_frame,
                    std::string body_frame, std::string clock_id);
  common::Status BeginEpoch(const SceneSource& source,
                            const RelativePose& initial, double now,
                            bool stationary);
  common::Status AddOdometry(const SceneSource& source,
                             const RelativePose& pose, double now);
  common::Status Observe(const SceneSource& source,
                         const LaneObservation& observation, double now);
  common::Status ObserveLaneAbsence(const SceneSource& source, double now);
  common::Status ObserveEnvironment(const SceneSource& source,
                                    const EnvironmentObservation& observation,
                                    double now);
  common::Status Build(double now, LocalScene* scene,
                       const NavigationPrior* navigation = nullptr);

 private:
  common::Status CheckSource(const SceneSource& source, double now,
                             const std::string& frame, double max_age) const;
  bool SameEpoch(const SceneSource& source) const;
  void Invalidate(const std::string& reason, bool odometry);
  TemporalLanePolicy policy_;
  TemporalLaneMap map_;
  std::string odom_frame_;
  std::string body_frame_;
  std::string clock_id_;
  SceneSource odometry_;
  SceneSource observation_;
  SceneSource environment_frontier_;
  DrivableEnvironment environment_;
  bool armed_ = false;
  bool observed_ = false;
  bool environment_observed_ = false;
  bool environment_frontier_observed_ = false;
  uint64_t scene_sequence_ = 0;
  uint64_t lane_id_ = 0;
  uint64_t next_boundary_id_ = 1;
  uint64_t left_id_ = 0;
  uint64_t right_id_ = 0;
  std::string lane_reason_ = "no lane observation";
  std::string invalid_reason_ = "epoch not initialized";
};

}  // namespace world_model
}  // namespace apollo
