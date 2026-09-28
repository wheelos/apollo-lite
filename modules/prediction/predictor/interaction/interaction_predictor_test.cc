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

#include "modules/prediction/predictor/interaction/interaction_predictor.h"

#include <cmath>
#include <string>

#include "wheelos_msgs/planning_msgs/planning.pb.h"

#include "cyber/common/file.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/prediction/common/kml_map_based_test.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/container/obstacles/obstacles_container.h"

namespace apollo {
namespace prediction {

class InteractionPredictorTest : public KMLMapBasedTest {
 public:
  virtual void SetUp() {
    const std::string file =
        "modules/prediction/testdata/single_perception_vehicle_onlane.pb.txt";
    cyber::common::GetProtoFromFile(file, &perception_obstacles_);
  }

 protected:
  apollo::perception::PerceptionObstacles perception_obstacles_;
};

TEST_F(InteractionPredictorTest, GeneratesNormalizedCandidatesWithAlignedPlan) {
  EXPECT_DOUBLE_EQ(perception_obstacles_.header().timestamp_sec(),
                   1501183430.161906);
  ObstaclesContainer container;
  ADCTrajectoryContainer adc_trajectory_container;
  container.Insert(perception_obstacles_);
  container.BuildLaneGraph();
  Obstacle* obstacle_ptr = container.GetObstacle(1);
  ASSERT_NE(obstacle_ptr, nullptr);
  auto* lane_graph = obstacle_ptr->mutable_latest_feature()
                         ->mutable_lane()
                         ->mutable_lane_graph();
  ASSERT_GT(lane_graph->lane_sequence_size(), 0);
  for (const auto& lane_sequence : lane_graph->lane_sequence()) {
    EXPECT_DOUBLE_EQ(lane_sequence.probability(), 0.0);
  }

  apollo::planning::ADCTrajectory adc_trajectory;
  adc_trajectory.mutable_header()->set_timestamp_sec(
      perception_obstacles_.header().timestamp_sec() - 0.05);
  const double time_step = FLAGS_prediction_trajectory_time_resolution;
  const int point_count = static_cast<int>(
      std::ceil(FLAGS_prediction_trajectory_time_length / time_step));
  for (int i = 0; i <= point_count; ++i) {
    auto* point = adc_trajectory.add_trajectory_point();
    point->set_relative_time(i * time_step);
    point->set_v(0.0);
    point->mutable_path_point()->set_x(
        obstacle_ptr->latest_feature().position().x() - 100.0);
    point->mutable_path_point()->set_y(
        obstacle_ptr->latest_feature().position().y());
    point->mutable_path_point()->set_theta(0.0);
  }
  adc_trajectory_container.Insert(adc_trajectory);

  InteractionPredictor predictor;
  ASSERT_TRUE(
      predictor.Predict(&adc_trajectory_container, obstacle_ptr, &container));
  const auto& trajectories =
      obstacle_ptr->latest_feature().predicted_trajectory();
  ASSERT_GT(trajectories.size(), 0);
  double probability_sum = 0.0;
  for (const auto& trajectory : trajectories) {
    EXPECT_GT(trajectory.trajectory_point_size(), 0);
    EXPECT_TRUE(std::isfinite(trajectory.probability()));
    EXPECT_GE(trajectory.probability(), 0.0);
    probability_sum += trajectory.probability();
  }
  EXPECT_NEAR(probability_sum, 1.0, 1e-6);
}

TEST_F(InteractionPredictorTest, RejectsMissingAdcPlan) {
  ObstaclesContainer container;
  ADCTrajectoryContainer adc_trajectory_container;
  container.Insert(perception_obstacles_);
  container.BuildLaneGraph();
  Obstacle* obstacle_ptr = container.GetObstacle(1);
  ASSERT_NE(obstacle_ptr, nullptr);

  InteractionPredictor predictor;
  EXPECT_FALSE(
      predictor.Predict(&adc_trajectory_container, obstacle_ptr, &container));
  EXPECT_EQ(predictor.NumOfTrajectories(*obstacle_ptr), 0);
}

TEST_F(InteractionPredictorTest, RejectsCollidingCandidate) {
  ObstaclesContainer container;
  container.Insert(perception_obstacles_);
  Obstacle* obstacle_ptr = container.GetObstacle(1);
  ASSERT_NE(obstacle_ptr, nullptr);
  ASSERT_GT(obstacle_ptr->latest_feature().length(), 0.0);
  ASSERT_GT(obstacle_ptr->latest_feature().width(), 0.0);

  Trajectory candidate;
  auto* target_point = candidate.add_trajectory_point();
  target_point->set_relative_time(0.3);
  target_point->set_v(1.0);
  target_point->mutable_path_point()->set_x(0.0);
  target_point->mutable_path_point()->set_y(0.0);
  target_point->mutable_path_point()->set_theta(0.0);

  std::vector<apollo::common::TrajectoryPoint> adc_trajectory(1);
  auto* ego_point = adc_trajectory.front().mutable_path_point();
  ego_point->set_x(0.0);
  ego_point->set_y(0.0);
  ego_point->set_theta(0.0);

  InteractionPredictor predictor;
  EXPECT_TRUE(std::isinf(predictor.CollisionWithEgoVehicleCost(
      *obstacle_ptr, candidate, adc_trajectory)));
}

TEST_F(InteractionPredictorTest, PenalizesOnlyClosingSameLaneTtc) {
  ObstaclesContainer container;
  container.Insert(perception_obstacles_);
  Obstacle* obstacle_ptr = container.GetObstacle(1);
  ASSERT_NE(obstacle_ptr, nullptr);
  ASSERT_GT(obstacle_ptr->latest_feature().length(), 0.0);
  ASSERT_GT(obstacle_ptr->latest_feature().width(), 0.0);

  Trajectory candidate;
  auto* target_point = candidate.add_trajectory_point();
  target_point->set_relative_time(0.5);
  target_point->set_v(5.0);
  target_point->mutable_path_point()->set_x(0.0);
  target_point->mutable_path_point()->set_y(0.0);
  target_point->mutable_path_point()->set_theta(0.0);

  std::vector<apollo::common::TrajectoryPoint> adc_trajectory(1);
  auto* ego_point = adc_trajectory.front().mutable_path_point();
  ego_point->set_x(12.0);
  ego_point->set_y(0.0);
  ego_point->set_theta(0.0);
  adc_trajectory.front().set_v(2.0);

  InteractionPredictor predictor;
  const double closing_cost = predictor.CollisionWithEgoVehicleCost(
      *obstacle_ptr, candidate, adc_trajectory);
  EXPECT_GT(closing_cost, 0.0);

  adc_trajectory.front().set_v(8.0);
  const double receding_cost = predictor.CollisionWithEgoVehicleCost(
      *obstacle_ptr, candidate, adc_trajectory);
  EXPECT_DOUBLE_EQ(receding_cost, 0.0);
}

TEST_F(InteractionPredictorTest, ScoresCrossingConflictTimeGap) {
  ObstaclesContainer container;
  container.Insert(perception_obstacles_);
  Obstacle* obstacle_ptr = container.GetObstacle(1);
  ASSERT_NE(obstacle_ptr, nullptr);
  ASSERT_GT(obstacle_ptr->latest_feature().length(), 0.0);
  ASSERT_GT(obstacle_ptr->latest_feature().width(), 0.0);

  Trajectory candidate;
  std::vector<apollo::common::TrajectoryPoint> ego_trajectory;
  for (int i = 0; i < 100; ++i) {
    const double time = i * 0.1;
    auto* target_point = candidate.add_trajectory_point();
    target_point->set_relative_time(time);
    target_point->set_v(3.0);
    target_point->mutable_path_point()->set_x(-9.0 + 0.3 * i);
    target_point->mutable_path_point()->set_y(0.0);
    target_point->mutable_path_point()->set_theta(0.0);

    ego_trajectory.emplace_back();
    auto* ego_path = ego_trajectory.back().mutable_path_point();
    ego_path->set_x(0.0);
    ego_path->set_y(-12.0 + 0.3 * i);
    ego_path->set_theta(M_PI / 2.0);
    ego_trajectory.back().set_v(3.0);
    ego_trajectory.back().set_relative_time(time);
  }

  InteractionPredictor predictor;
  const auto& vehicle_param =
      apollo::common::VehicleConfigHelper::GetConfig().vehicle_param();
  const double close_gap_cost = predictor.ConflictZoneTimeGapCost(
      *obstacle_ptr, candidate, ego_trajectory, vehicle_param.length(),
      vehicle_param.width());
  EXPECT_GT(close_gap_cost, 0.0);

  for (int i = 0; i < 100; ++i) {
    ego_trajectory[i].mutable_path_point()->set_y(-27.0 + 0.3 * i);
  }
  const double separated_gap_cost = predictor.ConflictZoneTimeGapCost(
      *obstacle_ptr, candidate, ego_trajectory, vehicle_param.length(),
      vehicle_param.width());
  EXPECT_DOUBLE_EQ(separated_gap_cost, 0.0);
}

}  // namespace prediction
}  // namespace apollo
