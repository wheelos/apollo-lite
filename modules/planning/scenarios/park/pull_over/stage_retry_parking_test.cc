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

#include "modules/planning/scenarios/park/pull_over/stage_retry_parking.h"

#include "gtest/gtest.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace pull_over {

class PullOverStageRetryParkingTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    config_.set_stage_type(StageType::PULL_OVER_RETRY_PARKING);
    injector_ = std::make_shared<DependencyInjector>();
  }

 protected:
  ScenarioConfig::StageConfig config_;
  std::shared_ptr<DependencyInjector> injector_;
};

TEST_F(PullOverStageRetryParkingTest, Init) {
  PullOverStageRetryParking pull_over_stage_retry_parking(config_, injector_);
  EXPECT_EQ(pull_over_stage_retry_parking.Name(),
            StageType_Name(StageType::PULL_OVER_RETRY_PARKING));
}

}  // namespace pull_over
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
