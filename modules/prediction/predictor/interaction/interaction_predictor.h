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
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
 *implied. See the License for the specific language governing
 *permissions and limitations under the License.
 *****************************************************************************/
/**
 * @file
 * @brief Define interaction predictor
 */

#pragma once

#include <vector>

#include "modules/prediction/predictor/sequence/sequence_predictor.h"

namespace apollo {
namespace prediction {

class InteractionPredictor : public SequencePredictor {
 public:
  /**
   * @brief Constructor
   */
  InteractionPredictor();

  /**
   * @brief Destructor
   */
  virtual ~InteractionPredictor() = default;

  /**
   * @brief Make prediction
   * @param Obstacle pointer
   * @param Obstacles container
   * @return If predicted successfully
   */
  bool Predict(const ADCTrajectoryContainer* adc_trajectory_container,
               Obstacle* obstacle,
               ObstaclesContainer* obstacles_container) override;

 private:
  void Clear();

  bool BuildADCTrajectory(
      const ADCTrajectoryContainer* adc_trajectory_container,
      const double obstacle_timestamp, const double time_resolution,
      const double prediction_horizon,
      std::vector<apollo::common::TrajectoryPoint>* adc_trajectory);

  bool DrawTrajectory(
      const Obstacle& obstacle, const LaneSequence& lane_sequence,
      const double lon_acceleration, const double total_time,
      const double period,
      std::vector<apollo::common::TrajectoryPoint>* trajectory_points);

  double ComputeTrajectoryCost(
      const Obstacle& obstacle, const LaneSequence& lane_sequence,
      const double acceleration, const Trajectory& candidate,
      const std::vector<apollo::common::TrajectoryPoint>& adc_trajectory);

  double LongitudinalAccelerationCost(const double acceleration);

  double CentripetalAccelerationCost(const LaneSequence& lane_sequence,
                                     const double speed,
                                     const double acceleration);

  double CollisionWithEgoVehicleCost(
      const Obstacle& obstacle, const Trajectory& candidate,
      const std::vector<apollo::common::TrajectoryPoint>& adc_trajectory) const;

  double ConflictZoneTimeGapCost(
      const Obstacle& obstacle, const Trajectory& candidate,
      const std::vector<apollo::common::TrajectoryPoint>& adc_trajectory,
      double ego_length, double ego_width) const;

  FRIEND_TEST(InteractionPredictorTest, RejectsCollidingCandidate);
  FRIEND_TEST(InteractionPredictorTest, PenalizesOnlyClosingSameLaneTtc);
  FRIEND_TEST(InteractionPredictorTest, ScoresCrossingConflictTimeGap);
};

}  // namespace prediction
}  // namespace apollo
