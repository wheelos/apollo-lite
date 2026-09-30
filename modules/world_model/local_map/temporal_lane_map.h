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

#include <deque>
#include <map>

#include "modules/common/status/status.h"
#include "modules/world_model/local_map/lane_types.h"

namespace apollo {
namespace world_model {

// Paired, ordered boundary samples in base_link at measurement_time.
// Pairing/direction is supplied by world-model ingestion, not inferred by this
// numerical tracker. Live camera ingestion remains a separate adapter.
// Bounded single-corridor layer inside the local map. No localization
// estimation or planner dependency.
class TemporalLaneMap {
 public:
  explicit TemporalLaneMap(const TemporalLanePolicy& policy);
  common::Status BeginEpoch(const std::string& session, uint64_t generation,
                            const RelativePose& initial, bool stationary);
  common::Status AddPose(const std::string& session, uint64_t generation,
                         const RelativePose& pose);
  common::Status PoseAt(double time, RelativePose* pose) const;
  common::Status Observe(const LaneObservation& observation, double now);
  common::Status Snapshot(double now, TemporalLaneSnapshot* snapshot);
  void Invalidate();
  void ClearLaneHistory();

 private:
  TemporalLanePolicy policy_;
  std::string session_;
  uint64_t generation_ = 0;
  std::deque<RelativePose> poses_;
  std::map<int, TrackedLaneSample> samples_;
  TrackedLaneSample observed_end_;
  bool armed_ = false;
  uint64_t last_sequence_ = 0;
  uint64_t version_ = 0;
  uint64_t lane_id_ = 0;
  double last_observation_ = -1.0;
  double last_snapshot_ = -1.0;
  double last_activity_ = -1.0;
  LanePoint anchor_;
  double anchor_heading_ = 0.0;
};

}  // namespace world_model
}  // namespace apollo
