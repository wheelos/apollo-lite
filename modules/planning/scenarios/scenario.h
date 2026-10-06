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

#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "absl/strings/str_cat.h"
#include "absl/strings/substitute.h"

#include "modules/planning/proto/planning_config.pb.h"

#include "modules/planning/common/dependency_injector.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/scenarios/stage.h"

namespace apollo {
namespace planning {
namespace scenario {

// Scenario Grade: Determines preemption priority
enum class ScenarioGrade {
  CRUISE = 0,    // L3: LaneFollow (Score 0-100)
  MANEUVER = 1,  // L2: Intersection, ParkAndGo (Score 100-200)
  MISSION = 2,   // L1: PullOver, ValetParking (Score 200-300)
  CRITICAL = 3   // L0: Emergency (Score > 300)
};

inline bool IsLocalScenarioGrade(ScenarioGrade grade) {
  return grade == ScenarioGrade::CRUISE ||
         grade == ScenarioGrade::MANEUVER ||
         grade == ScenarioGrade::CRITICAL;
}

inline std::string ScenarioGrade_Name(ScenarioGrade grade) {
  switch (grade) {
    case ScenarioGrade::CRUISE:
      return "CRUISE(L3)";
    case ScenarioGrade::MANEUVER:
      return "MANEUVER(L2)";
    case ScenarioGrade::MISSION:
      return "MISSION(L1)";
    case ScenarioGrade::CRITICAL:
      return "CRITICAL(L0)";
    default:
      return "UNKNOWN";
  }
}

// Decision Result Structure: Output from Deciders
struct ScenarioDecisionResult {
  ScenarioType type = ScenarioType::LANE_FOLLOW;
  ScenarioGrade grade = ScenarioGrade::CRUISE;
  uint32_t score = 0;
  std::string reason;
  bool can_enter = false;

  ScenarioDecisionResult() = default;
  ScenarioDecisionResult(ScenarioType t, ScenarioGrade g, uint32_t s,
                         const std::string& r)
      : type(t), grade(g), score(s), reason(r), can_enter(true) {}

  bool IsValid() const { return can_enter; }

  // Priority comparison: Grade first, then Score
  bool IsBetterThan(const ScenarioDecisionResult& other) const {
    if (grade != other.grade) {
      return static_cast<int>(grade) > static_cast<int>(other.grade);
    }
    return score > other.score;
  }

  std::string DebugString() const {
    if (!can_enter) {
      return "Result[Invalid]";
    }

    return absl::Substitute(
        "Result[Type: $0, Grade: $1, Score: $2, Reason: $3]",
        ScenarioType_Name(type), ScenarioGrade_Name(grade), score, reason);
  }
};

struct ScenarioContext {};

class Scenario {
 public:
  enum ScenarioStatus {
    STATUS_UNKNOWN = 0,
    STATUS_PROCESSING = 1,
    STATUS_DONE = 2,
  };

  Scenario(const ScenarioConfig& config, const ScenarioContext* context,
           const std::shared_ptr<DependencyInjector>& injector);

  virtual ~Scenario() = default;

  // -----------------------------------------------------------
  // Lifecycle Management
  // -----------------------------------------------------------

  // 1. Enter: Initialize resources, reset state.
  virtual void OnEnter(Frame* frame);

  // 2. Process: Execute core logic
  virtual ScenarioStatus Process(Frame* frame);

  // 3. Exit: Clean up resources and save critical state to the Global Context
  virtual void OnExit(Frame* frame);

  // -----------------------------------------------------------
  // Factory & Configuration
  // -----------------------------------------------------------
  virtual void Init();

  virtual std::unique_ptr<Stage> CreateStage(
      const ScenarioConfig::StageConfig& stage_config,
      const std::shared_ptr<DependencyInjector>& injector) = 0;

  static bool LoadConfig(const std::string& config_file,
                         ScenarioConfig* config);

  // -----------------------------------------------------------
  // Getters
  // -----------------------------------------------------------
  const std::string& Name() const { return name_; }
  ScenarioType Type() const { return config_.scenario_type(); }
  ScenarioStatus GetStatus() const { return scenario_status_; }
  const std::string& GetMsg() const { return msg_; }

  virtual ScenarioGrade Grade() const = 0;

  StageType GetStageType() const {
    return current_stage_ ? current_stage_->stage_type() : StageType::NO_STAGE;
  }

 protected:
  template <typename T>
  const T* GetContext() {
    return dynamic_cast<const T*>(scenario_context_);
  }

  ScenarioStatus scenario_status_ = STATUS_UNKNOWN;

  std::unique_ptr<Stage> current_stage_;
  ScenarioConfig config_;

  std::unordered_map<StageType, const ScenarioConfig::StageConfig*>
      stage_config_map_;

  // Scenario-specific context (assigned by the subclass in Init)
  const ScenarioContext* scenario_context_;

  std::string name_;
  std::string msg_;

  std::shared_ptr<DependencyInjector> injector_;
};

}  // namespace scenario
}  // namespace planning
}  // namespace apollo
