/******************************************************************************
 * Copyright 2026 The Apollo Authors. All Rights Reserved.
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

#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "cyber/common/log.h"
#include "modules/planning/planning_base.h"

namespace apollo {
namespace planning {

class PlanningShellBridge final : public PlanningBase {
 public:
  PlanningShellBridge(const std::shared_ptr<DependencyInjector>& injector,
                      PlanningMode mode, std::string name,
                      PlanningBase* delegate)
      : PlanningBase(injector),
        mode_(mode),
        name_(std::move(name)),
        delegate_(delegate) {}

  common::Status Init(const PlanningConfig& config) override {
    (void)config;
    if (delegate_ == nullptr) {
      return common::Status(common::ErrorCode::PLANNING_ERROR,
                            "planning shell bridge delegate unavailable");
    }
    return common::Status::OK();
  }

  std::string Name() const override { return name_; }

  PlanningMode Mode() const override { return mode_; }

  void RunOnce(const LocalView& local_view,
               ADCTrajectory* const adc_trajectory) override {
    ACHECK(delegate_ != nullptr);
    delegate_->RunOnce(local_view, adc_trajectory);
  }

  common::Status Plan(
      const double current_time_stamp,
      const std::vector<common::TrajectoryPoint>& stitching_trajectory,
      ADCTrajectory* const trajectory) override {
    ACHECK(delegate_ != nullptr);
    return delegate_->Plan(current_time_stamp, stitching_trajectory, trajectory);
  }

 private:
  PlanningMode mode_ = MODE_UNKNOWN;
  std::string name_;
  PlanningBase* delegate_ = nullptr;
};

}  // namespace planning
}  // namespace apollo
