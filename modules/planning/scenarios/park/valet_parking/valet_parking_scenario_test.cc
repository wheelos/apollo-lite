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

#include "gtest/gtest.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/util/util.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace valet_parking {

class ValetParkingScenarioTest : public ::testing::Test {
 public:
  virtual void SetUp() {}

 protected:
  std::unique_ptr<ValetParkingScenario> scenario_;
};

TEST_F(ValetParkingScenarioTest, Init) {
  FLAGS_scenario_valet_parking_config_file =
      "/apollo/modules/planning/conf/scenario/valet_parking_config.pb.txt";

  ScenarioConfig config;
  EXPECT_TRUE(apollo::cyber::common::GetProtoFromFile(
      FLAGS_scenario_valet_parking_config_file, &config));
  ScenarioContext context;
  auto injector = std::make_shared<DependencyInjector>();
  scenario_.reset(new ValetParkingScenario(config, &context, injector));
  EXPECT_EQ(scenario_->Type(), ScenarioType::VALET_PARKING);
}

TEST_F(ValetParkingScenarioTest, SupportsDirectParkingEntry) {
  FLAGS_scenario_valet_parking_config_file =
      "/apollo/modules/planning/conf/scenario/valet_parking_config.pb.txt";

  ScenarioConfig config;
  ASSERT_TRUE(apollo::cyber::common::GetProtoFromFile(
      FLAGS_scenario_valet_parking_config_file, &config));
  EXPECT_TRUE(ValetParkingScenario::SupportsDirectParkingEntry(config));

  routing::RoutingResponse routing_response;
  EXPECT_FALSE(
      ValetParkingScenario::HasParkingRoutingCommand(routing_response));

  routing_response.mutable_routing_request()
      ->mutable_parking_info()
      ->set_parking_space_id("parking_space_1");
  EXPECT_TRUE(
      ValetParkingScenario::HasParkingRoutingCommand(routing_response));

  routing::RoutingResponse corner_only_routing;
  corner_only_routing.mutable_routing_request()
      ->mutable_parking_info()
      ->mutable_corner_point()
      ->add_point()
      ->set_x(1.0);
  EXPECT_TRUE(
      ValetParkingScenario::HasParkingRoutingCommand(corner_only_routing));
  EXPECT_FALSE(util::ShouldUseDirectValetParkingMode(true,
      std::make_shared<routing::RoutingResponse>(corner_only_routing)));
}

}  // namespace valet_parking
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
