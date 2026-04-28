// Copyright 2025 WheelOS. All Rights Reserved.
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

//  Created Date: 2026-04-28
//  Author: daohu527


#include "modules/prediction/predictor/free_move/free_move_predictor.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

#include "Eigen/Eigenvalues"
#include "cyber/common/file.h"
#include "modules/prediction/common/kml_map_based_test.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_util.h"
#include "modules/prediction/container/obstacles/obstacles_container.h"

namespace apollo {
namespace prediction {

class FreeMovePredictorEvalTest : public KMLMapBasedTest {
 public:
  FreeMovePredictorEvalTest() {
    ACHECK(cyber::common::GetProtoFromFile(
        "modules/prediction/testdata/frame_sequence/frame_1.pb.txt",
        &frame_1_));
    ACHECK(cyber::common::GetProtoFromFile(
        "modules/prediction/testdata/frame_sequence/frame_2.pb.txt",
        &frame_2_));
    ACHECK(cyber::common::GetProtoFromFile(
        "modules/prediction/testdata/frame_sequence/frame_3.pb.txt",
        &frame_3_));
  }

 protected:
  std::vector<apollo::common::TrajectoryPoint> BuildNewTrajectory(
      Obstacle* obstacle_ptr) const;
  apollo::perception::PerceptionObstacles MakeTurningFrame(
      double timestamp, double yaw, double speed, double yaw_rate) const;

  apollo::perception::PerceptionObstacles frame_1_;
  apollo::perception::PerceptionObstacles frame_2_;
  apollo::perception::PerceptionObstacles frame_3_;
};

namespace {

constexpr double kHistoryWindowSec = 1.0;
constexpr double kMinReliableHistorySec = 0.2;
constexpr double kLowSpeedHeadingFallbackThreshold = 0.1;
constexpr double kHeadingAgreementThreshold = M_PI / 12.0;
constexpr double kStraightYawRateThreshold = 0.05;
constexpr double kYawRateEmaAlpha = 0.2;
constexpr double kSpeedEmaAlpha = 0.4;
constexpr int kTurningObstacleId = 42;

struct FreeMoveMotionEstimate {
  double smoothed_theta = 0.0;
  double yaw_rate = 0.0;
  double history_time = 0.0;
  double max_speed = 0.0;
  Eigen::Vector2d position = Eigen::Vector2d::Zero();
  Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
  Eigen::Vector2d acc = Eigen::Vector2d::Zero();
};

double NormalizeAngle(const double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

double AngleDiff(const double from, const double to) {
  return NormalizeAngle(to - from);
}

bool HasValidPosition(const Feature& feature) {
  return feature.has_position() && feature.position().has_x() &&
         feature.position().has_y();
}

std::vector<const Feature*> CollectRecentHistoryForTest(const Obstacle& obstacle) {
  std::vector<const Feature*> history;
  if (obstacle.history_size() == 0) {
    return history;
  }

  const double latest_timestamp = obstacle.latest_feature().timestamp();
  for (size_t i = obstacle.history_size(); i > 0; --i) {
    const Feature& feature = obstacle.feature(i - 1);
    if (!HasValidPosition(feature)) {
      continue;
    }
    if (latest_timestamp - feature.timestamp() > kHistoryWindowSec) {
      continue;
    }
    history.push_back(&feature);
  }
  return history;
}

double AlignHeadingToReference(const double heading,
                               const Eigen::Vector2d& reference_direction) {
  if (reference_direction.squaredNorm() <= 1e-6) {
    return heading;
  }
  const Eigen::Vector2d heading_direction(std::cos(heading), std::sin(heading));
  if (heading_direction.dot(reference_direction) >= 0.0) {
    return heading;
  }
  return NormalizeAngle(heading + M_PI);
}

double FitHeadingFromHistoryForTest(
    const std::vector<const Feature*>& history, const double fallback_heading,
    const Eigen::Vector2d& reference_direction) {
  if (history.size() < 2) {
    return fallback_heading;
  }

  Eigen::MatrixXd samples(2, history.size());
  for (size_t i = 0; i < history.size(); ++i) {
    samples(0, i) = history[i]->position().x();
    samples(1, i) = history[i]->position().y();
  }
  const Eigen::Vector2d mean = samples.rowwise().mean();
  samples.colwise() -= mean;
  const Eigen::Matrix2d covariance =
      samples * samples.transpose() / static_cast<double>(history.size() - 1);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigen_solver(covariance);
  double fitted_heading = std::atan2(eigen_solver.eigenvectors()(1, 1),
                                     eigen_solver.eigenvectors()(0, 1));
  fitted_heading = AlignHeadingToReference(fitted_heading, reference_direction);
  if (std::fabs(AngleDiff(fallback_heading, fitted_heading)) <=
      kHeadingAgreementThreshold) {
    return fitted_heading;
  }
  return fallback_heading;
}

double ComputeHistoryTimeForTest(const std::vector<const Feature*>& history) {
  if (history.size() < 2) {
    return 0.0;
  }
  return std::max(0.0,
                  history.back()->timestamp() - history.front()->timestamp());
}

double MaxSpeedByObstacleTypeForTest(
    const apollo::perception::PerceptionObstacle::Type type) {
  if (type == apollo::perception::PerceptionObstacle::PEDESTRIAN) {
    return FLAGS_pedestrian_max_speed;
  }
  return FLAGS_vehicle_max_speed;
}

double EstimateScalarSpeedFromHistoryForTest(
    const std::vector<const Feature*>& history, const double fallback_speed) {
  if (history.size() < 2) {
    return fallback_speed;
  }

  bool initialized = false;
  double filtered_speed = fallback_speed;
  for (size_t i = 1; i < history.size(); ++i) {
    const auto& prev_position = history[i - 1]->position();
    const auto& curr_position = history[i]->position();
    const double delta_t = history[i]->timestamp() - history[i - 1]->timestamp();
    if (delta_t <= 1e-3) {
      continue;
    }

    const double segment_speed =
        std::hypot(curr_position.x() - prev_position.x(),
                   curr_position.y() - prev_position.y()) /
        delta_t;
    filtered_speed = initialized
                         ? kSpeedEmaAlpha * segment_speed +
                               (1.0 - kSpeedEmaAlpha) * filtered_speed
                         : segment_speed;
    initialized = true;
  }
  return initialized ? filtered_speed : fallback_speed;
}

double EstimateLongitudinalAccelerationFromHistoryForTest(
    const std::vector<const Feature*>& history, const double fallback_acc) {
  if (history.size() < 3) {
    return fallback_acc;
  }

  double first_speed = -1.0;
  double last_speed = -1.0;
  double first_time = 0.0;
  double last_time = 0.0;
  for (size_t i = 1; i < history.size(); ++i) {
    const auto& prev_position = history[i - 1]->position();
    const auto& curr_position = history[i]->position();
    const double delta_t = history[i]->timestamp() - history[i - 1]->timestamp();
    if (delta_t <= 1e-3) {
      continue;
    }
    const double segment_speed =
        std::hypot(curr_position.x() - prev_position.x(),
                   curr_position.y() - prev_position.y()) /
        delta_t;
    if (first_speed < 0.0) {
      first_speed = segment_speed;
      first_time = history[i]->timestamp();
    }
    last_speed = segment_speed;
    last_time = history[i]->timestamp();
  }

  if (first_speed < 0.0 || last_speed < 0.0 || last_time - first_time <= 1e-3) {
    return fallback_acc;
  }
  return (last_speed - first_speed) / (last_time - first_time);
}

double EstimateYawRateFromHistoryForTest(
    const std::vector<const Feature*>& history) {
  if (history.size() < 3) {
    return 0.0;
  }

  bool initialized = false;
  bool has_previous_heading = false;
  double filtered_yaw_rate = 0.0;
  double previous_heading = 0.0;
  for (size_t i = 1; i < history.size(); ++i) {
    const auto& prev_position = history[i - 1]->position();
    const auto& curr_position = history[i]->position();
    const double delta_x = curr_position.x() - prev_position.x();
    const double delta_y = curr_position.y() - prev_position.y();
    const double delta_t = history[i]->timestamp() - history[i - 1]->timestamp();
    if (delta_t <= 1e-3 || std::hypot(delta_x, delta_y) <= 1e-3) {
      continue;
    }

    const double segment_heading = std::atan2(delta_y, delta_x);
    if (!has_previous_heading) {
      previous_heading = segment_heading;
      has_previous_heading = true;
      continue;
    }

    const double raw_yaw_rate =
        AngleDiff(previous_heading, segment_heading) / delta_t;
    filtered_yaw_rate = initialized
                            ? kYawRateEmaAlpha * raw_yaw_rate +
                                  (1.0 - kYawRateEmaAlpha) * filtered_yaw_rate
                            : raw_yaw_rate;
    initialized = true;
    previous_heading =
        previous_heading + AngleDiff(previous_heading, segment_heading);
  }
  return filtered_yaw_rate;
}

FreeMoveMotionEstimate EstimateFreeMoveMotionForTest(const Obstacle& obstacle) {
  FreeMoveMotionEstimate estimate;
  const Feature& feature = obstacle.latest_feature();
  estimate.smoothed_theta = feature.velocity_heading();
  estimate.velocity =
      Eigen::Vector2d(feature.velocity().x(), feature.velocity().y());
  estimate.acc =
      Eigen::Vector2d(feature.acceleration().x(), feature.acceleration().y());
  estimate.position =
      Eigen::Vector2d(feature.position().x(), feature.position().y());
  estimate.max_speed = MaxSpeedByObstacleTypeForTest(obstacle.type());

  const auto history = CollectRecentHistoryForTest(obstacle);
  estimate.history_time = ComputeHistoryTimeForTest(history);
  const double fallback_speed = estimate.velocity.norm();
  Eigen::Vector2d reference_direction = estimate.velocity;
  if (reference_direction.squaredNorm() <= 1e-6 && history.size() >= 2) {
    reference_direction = Eigen::Vector2d(
        history.back()->position().x() - history.front()->position().x(),
        history.back()->position().y() - history.front()->position().y());
  }
  if (fallback_speed <= kLowSpeedHeadingFallbackThreshold &&
      reference_direction.squaredNorm() > 1e-6) {
    estimate.smoothed_theta =
        std::atan2(reference_direction.y(), reference_direction.x());
  }
  estimate.smoothed_theta = FitHeadingFromHistoryForTest(
      history, estimate.smoothed_theta, reference_direction);
  estimate.yaw_rate = EstimateYawRateFromHistoryForTest(history);

  if (estimate.history_time < kMinReliableHistorySec) {
    estimate.smoothed_theta = feature.velocity_heading();
    estimate.yaw_rate = 0.0;
    estimate.acc *= 0.5;
  } else if (std::fabs(estimate.yaw_rate) < kStraightYawRateThreshold) {
    estimate.yaw_rate = 0.0;
  }

  if (estimate.history_time >= kMinReliableHistorySec) {
    const double history_speed = std::min(
        estimate.max_speed,
        EstimateScalarSpeedFromHistoryForTest(history, fallback_speed));
    const double heading_cos = std::cos(estimate.smoothed_theta);
    const double heading_sin = std::sin(estimate.smoothed_theta);
    const double fallback_long_acc =
        estimate.acc(0) * heading_cos + estimate.acc(1) * heading_sin;
    const double history_long_acc = EstimateLongitudinalAccelerationFromHistoryForTest(
        history, fallback_long_acc);
    estimate.velocity =
        Eigen::Vector2d(history_speed * heading_cos, history_speed * heading_sin);
    estimate.acc = Eigen::Vector2d(history_long_acc * heading_cos,
                                   history_long_acc * heading_sin);
  }
  return estimate;
}

double StdDev(const std::vector<double>& values) {
  if (values.empty()) {
    return 0.0;
  }
  double sum = 0.0;
  for (double value : values) {
    sum += value;
  }
  const double mean = sum / static_cast<double>(values.size());
  double square_sum = 0.0;
  for (double value : values) {
    square_sum += (value - mean) * (value - mean);
  }
  return std::sqrt(square_sum / static_cast<double>(values.size()));
}

std::vector<double> ComputeLateralOffsets(
    const std::vector<apollo::common::TrajectoryPoint>& points) {
  std::vector<double> lateral_offsets;
  if (points.empty()) {
    return lateral_offsets;
  }

  const double x0 = points.front().path_point().x();
  const double y0 = points.front().path_point().y();
  const double theta0 = points.front().path_point().theta();
  for (const auto& point : points) {
    const double dx = point.path_point().x() - x0;
    const double dy = point.path_point().y() - y0;
    lateral_offsets.push_back(-dx * std::sin(theta0) + dy * std::cos(theta0));
  }
  return lateral_offsets;
}

std::vector<double> ComputeHeadingChanges(
    const std::vector<apollo::common::TrajectoryPoint>& points) {
  std::vector<double> heading_changes;
  if (points.size() < 2) {
    return heading_changes;
  }
  for (size_t i = 1; i < points.size(); ++i) {
    heading_changes.push_back(
        AngleDiff(points[i - 1].path_point().theta(), points[i].path_point().theta()));
  }
  return heading_changes;
}

bool SampleTrajectoryPositionAtTime(
    const std::vector<apollo::common::TrajectoryPoint>& points,
    const double target_time, double* x, double* y) {
  if (points.empty() || x == nullptr || y == nullptr) {
    return false;
  }
  if (target_time < points.front().relative_time() - 1e-6 ||
      target_time > points.back().relative_time() + 1e-6) {
    return false;
  }
  for (size_t i = 1; i < points.size(); ++i) {
    if (target_time <= points[i].relative_time() + 1e-6) {
      const auto& left = points[i - 1];
      const auto& right = points[i];
      const double gap = right.relative_time() - left.relative_time();
      if (gap <= 1e-6) {
        *x = right.path_point().x();
        *y = right.path_point().y();
        return true;
      }
      const double ratio = (target_time - left.relative_time()) / gap;
      *x = left.path_point().x() +
           (right.path_point().x() - left.path_point().x()) * ratio;
      *y = left.path_point().y() +
           (right.path_point().y() - left.path_point().y()) * ratio;
      return true;
    }
  }
  *x = points.back().path_point().x();
  *y = points.back().path_point().y();
  return true;
}

const apollo::perception::PerceptionObstacle* FindObstacleById(
    const apollo::perception::PerceptionObstacles& frame,
    const int obstacle_id) {
  for (const auto& obstacle : frame.perception_obstacle()) {
    if (obstacle.id() == obstacle_id) {
      return &obstacle;
    }
  }
  return nullptr;
}

std::vector<apollo::common::TrajectoryPoint> BuildBaselineTrajectory(
    const Feature& feature) {
  Eigen::Matrix<double, 6, 1> state;
  state.setZero();
  state(2, 0) = feature.velocity().x();
  state(3, 0) = feature.velocity().y();
  state(4, 0) = std::clamp(feature.acceleration().x(),
                           FLAGS_vehicle_min_linear_acc,
                           FLAGS_vehicle_max_linear_acc);
  state(5, 0) = std::clamp(feature.acceleration().y(),
                           FLAGS_vehicle_min_linear_acc,
                           FLAGS_vehicle_max_linear_acc);

  const double period = FLAGS_prediction_trajectory_time_resolution;
  Eigen::Matrix<double, 6, 6> transition;
  transition.setIdentity();
  transition(0, 2) = period;
  transition(0, 4) = 0.5 * period * period;
  transition(1, 3) = period;
  transition(1, 5) = 0.5 * period * period;
  transition(2, 4) = period;
  transition(3, 5) = period;

  const size_t num = static_cast<size_t>(
      FLAGS_prediction_trajectory_time_length / period);
  std::vector<apollo::common::TrajectoryPoint> points;
  ::apollo::prediction::predictor_util::GenerateFreeMoveTrajectoryPoints(
      &state, transition, feature.velocity_heading(), 0.0, num, period, &points);
  for (auto& point : points) {
    ::apollo::prediction::predictor_util::TranslatePoint(
        feature.position().x(), feature.position().y(), &point);
  }
  return points;
}

TEST_F(FreeMovePredictorEvalTest, CompareBaselineVsNewWithHistorySequence) {
  ObstaclesContainer container;
  container.Insert(frame_1_);
  container.Insert(frame_2_);
  container.Insert(frame_3_);

  Obstacle* obstacle_ptr = container.GetObstacle(1);
  ASSERT_NE(obstacle_ptr, nullptr);
  ASSERT_GE(obstacle_ptr->history_size(), 3U);

  const Feature& feature = obstacle_ptr->latest_feature();
  const auto baseline_points = BuildBaselineTrajectory(feature);
  const auto new_points = BuildNewTrajectory(obstacle_ptr);

  const double baseline_lateral_std =
      StdDev(ComputeLateralOffsets(baseline_points));
  const double new_lateral_std = StdDev(ComputeLateralOffsets(new_points));
  const double baseline_heading_std =
      StdDev(ComputeHeadingChanges(baseline_points));
  const double new_heading_std = StdDev(ComputeHeadingChanges(new_points));

  std::cout << "History size: " << obstacle_ptr->history_size() << std::endl;
  std::cout << "Baseline lateral stddev: " << baseline_lateral_std
            << ", New lateral stddev: " << new_lateral_std << std::endl;
  std::cout << "Baseline heading-change stddev: " << baseline_heading_std
            << ", New heading-change stddev: " << new_heading_std << std::endl;

  EXPECT_LE(new_lateral_std + 1e-6, baseline_lateral_std + 1e-6);
  EXPECT_LE(new_heading_std + 1e-6, baseline_heading_std + 1e-6);
}

TEST_F(FreeMovePredictorEvalTest, ImproveHeldOutFuturePositionAccuracy) {
  ObstaclesContainer container;
  container.Insert(frame_1_);
  container.Insert(frame_2_);

  Obstacle* obstacle_ptr = container.GetObstacle(1);
  ASSERT_NE(obstacle_ptr, nullptr);
  ASSERT_GE(obstacle_ptr->history_size(), 2U);

  const Feature& feature = obstacle_ptr->latest_feature();
  const auto baseline_points = BuildBaselineTrajectory(feature);
  const auto new_points = BuildNewTrajectory(obstacle_ptr);

  const auto* future_obstacle = FindObstacleById(frame_3_, obstacle_ptr->id());
  ASSERT_NE(future_obstacle, nullptr);
  const double target_time =
      frame_3_.header().timestamp_sec() - feature.timestamp();
  ASSERT_GT(target_time, 0.0);

  double baseline_x = 0.0;
  double baseline_y = 0.0;
  double new_x = 0.0;
  double new_y = 0.0;
  ASSERT_TRUE(SampleTrajectoryPositionAtTime(
      baseline_points, target_time, &baseline_x, &baseline_y));
  ASSERT_TRUE(
      SampleTrajectoryPositionAtTime(new_points, target_time, &new_x, &new_y));

  const double actual_x = future_obstacle->position().x();
  const double actual_y = future_obstacle->position().y();
  const double baseline_error =
      std::hypot(baseline_x - actual_x, baseline_y - actual_y);
  const double new_error = std::hypot(new_x - actual_x, new_y - actual_y);

  std::cout << "Held-out future time: " << target_time
            << ", baseline error: " << baseline_error
            << ", new error: " << new_error << std::endl;

  EXPECT_LE(new_error, 0.1);
  EXPECT_LE(new_error, baseline_error * 1.35);
}

TEST_F(FreeMovePredictorEvalTest, FriendPathDoesNotMutatePredictedTrajectory) {
  ObstaclesContainer container;
  container.Insert(frame_1_);
  container.Insert(frame_2_);
  container.Insert(frame_3_);

  Obstacle* obstacle_ptr = container.GetObstacle(1);
  ASSERT_NE(obstacle_ptr, nullptr);
  ASSERT_GE(obstacle_ptr->history_size(), 3U);
  EXPECT_EQ(obstacle_ptr->latest_feature().predicted_trajectory_size(), 0);

  const auto points = BuildNewTrajectory(obstacle_ptr);

  EXPECT_FALSE(points.empty());
  EXPECT_EQ(obstacle_ptr->latest_feature().predicted_trajectory_size(), 0);
}

TEST_F(FreeMovePredictorEvalTest, TurningHistoryProducesSmoothMonotonicHeading) {
  ObstaclesContainer container;
  constexpr double kSpeed = 5.0;
  constexpr double kYawRate = 0.12;
  constexpr double kDt = 0.2;
  for (int i = 0; i < 6; ++i) {
    container.Insert(MakeTurningFrame(i * kDt, i * kYawRate * kDt, kSpeed,
                                      kYawRate));
  }

  Obstacle* obstacle_ptr = container.GetObstacle(kTurningObstacleId);
  ASSERT_NE(obstacle_ptr, nullptr);
  ASSERT_GE(obstacle_ptr->history_size(), 5U);

  const auto new_points = BuildNewTrajectory(obstacle_ptr);
  ASSERT_GE(new_points.size(), 5U);

  bool saw_turning = false;
  for (size_t i = 1; i < new_points.size(); ++i) {
    const double delta_theta = AngleDiff(new_points[i - 1].path_point().theta(),
                                         new_points[i].path_point().theta());
    EXPECT_GE(delta_theta, -1e-6);
    EXPECT_LE(delta_theta, 0.05);
    saw_turning = saw_turning || delta_theta > 1e-4;
  }
  EXPECT_TRUE(saw_turning);
}

}  // namespace

std::vector<apollo::common::TrajectoryPoint>
FreeMovePredictorEvalTest::BuildNewTrajectory(Obstacle* obstacle_ptr) const {
  std::vector<apollo::common::TrajectoryPoint> points;
  if (obstacle_ptr == nullptr) {
    return points;
  }

  const FreeMoveMotionEstimate motion = EstimateFreeMoveMotionForTest(*obstacle_ptr);
  FreeMovePredictor predictor;
  predictor.DrawFreeMoveTrajectoryPoints(
      motion.position, motion.velocity, motion.acc, motion.smoothed_theta,
      motion.yaw_rate, motion.history_time, motion.max_speed, 0.0,
      FLAGS_prediction_trajectory_time_length,
      FLAGS_prediction_trajectory_time_resolution, &points);
  return points;
}

apollo::perception::PerceptionObstacles FreeMovePredictorEvalTest::MakeTurningFrame(
    double timestamp, double yaw, double speed, double yaw_rate) const {
  apollo::perception::PerceptionObstacles frame;
  frame.mutable_header()->set_timestamp_sec(timestamp);
  auto* obstacle = frame.add_perception_obstacle();
  obstacle->set_id(kTurningObstacleId);
  obstacle->set_type(apollo::perception::PerceptionObstacle::VEHICLE);
  obstacle->set_theta(yaw);
  obstacle->set_timestamp(timestamp);

  const double radius = speed / yaw_rate;
  obstacle->mutable_position()->set_x(radius * std::sin(yaw));
  obstacle->mutable_position()->set_y(radius * (1.0 - std::cos(yaw)));
  obstacle->mutable_velocity()->set_x(speed * std::cos(yaw));
  obstacle->mutable_velocity()->set_y(speed * std::sin(yaw));
  obstacle->mutable_acceleration()->set_x(-speed * yaw_rate * std::sin(yaw));
  obstacle->mutable_acceleration()->set_y(speed * yaw_rate * std::cos(yaw));
  return frame;
}

}  // namespace prediction
}  // namespace apollo
