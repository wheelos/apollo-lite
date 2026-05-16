#include "modules/planning/scenarios/scenario_manager.h"

#include "gtest/gtest.h"

#include "cyber/common/environment.h"
#include "cyber/common/file.h"

#include "modules/planning/integration_tests/scenario_manager/scenario_builder.h"
#include "modules/planning/integration_tests/scenario_manager/scenario_case.h"
#include "modules/planning/integration_tests/scenario_manager/scenario_loader.h"

namespace apollo {
namespace planning {
namespace scenario {

namespace {

std::string ResolveTestDataPath(const std::string& workspace_relative_path) {
  if (cyber::common::PathExists(workspace_relative_path)) {
    return workspace_relative_path;
  }

  const auto test_srcdir = cyber::common::GetEnv("TEST_SRCDIR");
  const auto test_workspace = cyber::common::GetEnv("TEST_WORKSPACE");
  if (!test_srcdir.empty() && !test_workspace.empty()) {
    const auto runfile_path = test_srcdir + "/" + test_workspace + "/" +
                              workspace_relative_path;
    if (cyber::common::PathExists(runfile_path)) {
      return runfile_path;
    }
  }

  return "/apollo/" + workspace_relative_path;
}

}  // namespace

class ScenarioManagerTest : public ::testing::TestWithParam<ScenarioCase> {
 protected:
  void SetUp() override {
    injector_ = std::make_shared<DependencyInjector>();

    config_ = PlanningConfig();
    ASSERT_TRUE(cyber::common::GetProtoFromFile(
        ResolveTestDataPath("modules/planning/conf/planning_config.pb.txt"),
        &config_));

    scenario_manager_ = std::make_unique<ScenarioManager>(injector_);
    ASSERT_TRUE(scenario_manager_->Init(config_));
  }

  std::unique_ptr<ScenarioManager> scenario_manager_;
  std::shared_ptr<DependencyInjector> injector_;
  PlanningConfig config_;
  ScenarioBuilder builder_;
};

class ScenarioManagerDirectParkingTest : public ::testing::Test {
 protected:
  void SetUp() override {
    injector_ = std::make_shared<DependencyInjector>();

    config_ = PlanningConfig();
    ASSERT_TRUE(cyber::common::GetProtoFromFile(
        ResolveTestDataPath("modules/planning/conf/planning_config.pb.txt"),
        &config_));

    scenario_manager_ = std::make_unique<ScenarioManager>(injector_);
    ASSERT_TRUE(scenario_manager_->Init(config_));
  }

  std::unique_ptr<ScenarioManager> scenario_manager_;
  std::shared_ptr<DependencyInjector> injector_;
  PlanningConfig config_;
};

TEST_P(ScenarioManagerTest, VerifyScenarioTransitions) {
  const auto& test_case = GetParam();
  SCOPED_TRACE("Testing Case ID: " + test_case.id + " - " +
               test_case.description);

  // 1. Preparing a data snapshot: LocalView
  LocalView local_view;
  local_view.prediction_obstacles =
      std::make_shared<prediction::PredictionObstacles>();

  // 2. Construct Frame
  uint32_t sequence_num = 1;
  common::TrajectoryPoint start_point;
  common::VehicleState vehicle_state;

  // First, populate the local_view and injector using the Builder.
  builder_.Build(test_case.input, injector_.get(), &local_view, &vehicle_state);

  // 3. Instantiate Frame
  auto frame = std::make_unique<Frame>(sequence_num, local_view, start_point,
                                       vehicle_state);
  frame->ResetPadMsgDrivingAction();
  frame->ReadPadMsgDrivingAction();

  // 4. Execution scheduling
  scenario_manager_->ScenarioDispatch(*frame);

  // 5. Verification results
  auto current_scen = scenario_manager_->mutable_scenario();
  ASSERT_NE(current_scen, nullptr);
  EXPECT_EQ(ScenarioType_Name(current_scen->Type()),
            test_case.expected.scenario);
}

TEST_F(ScenarioManagerDirectParkingTest,
       DetectsParkingRouteWithoutReferenceLine) {
  LocalView local_view;
  local_view.prediction_obstacles =
      std::make_shared<prediction::PredictionObstacles>();
  local_view.routing = std::make_shared<routing::RoutingResponse>();
  auto* parking_info =
      local_view.routing->mutable_routing_request()->mutable_parking_info();
  parking_info->set_parking_space_id("test_parking_spot");

  common::TrajectoryPoint start_point;
  common::VehicleState vehicle_state;
  auto frame = std::make_unique<Frame>(1, local_view, start_point,
                                       vehicle_state);

  EXPECT_TRUE(frame->reference_line_info().empty());
  EXPECT_TRUE(scenario_manager_->ShouldEnterDirectValetParking(*frame));
}

TEST_F(ScenarioManagerDirectParkingTest,
       RejectsCornerOnlyDirectParkingWithoutReferenceLine) {
  LocalView local_view;
  local_view.prediction_obstacles =
      std::make_shared<prediction::PredictionObstacles>();
  local_view.routing = std::make_shared<routing::RoutingResponse>();
  auto* corner_point = local_view.routing->mutable_routing_request()
                           ->mutable_parking_info()
                           ->mutable_corner_point();
  corner_point->add_point()->set_x(0.0);
  corner_point->add_point()->set_x(1.0);

  common::TrajectoryPoint start_point;
  common::VehicleState vehicle_state;
  auto frame = std::make_unique<Frame>(1, local_view, start_point,
                                       vehicle_state);

  EXPECT_TRUE(frame->reference_line_info().empty());
  EXPECT_FALSE(scenario_manager_->ShouldEnterDirectValetParking(*frame));
}

// Ensure you use the correct Loader method name
INSTANTIATE_TEST_SUITE_P(ScenarioIntegrationTests, ScenarioManagerTest,
                         ::testing::ValuesIn(ScenarioLoader::Load(
               ResolveTestDataPath(
                 "modules/planning/integration_tests/"
                 "scenario_manager/scenario_cases.yaml"))));

}  // namespace scenario
}  // namespace planning
}  // namespace apollo
