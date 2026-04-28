/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/prediction/predictor/free_move/free_move_predictor.h"

#include <algorithm>
#include <cmath>
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_util.h"
#include "modules/prediction/proto/prediction_conf.pb.h"

namespace apollo {
namespace prediction {

namespace {

constexpr double kHistoryWindowSec = 1.0;
constexpr double kMinReliableHistorySec = 0.2;
constexpr double kLowSpeedHeadingFallbackThreshold = 0.1;
constexpr double kHeadingAgreementThreshold = M_PI / 12.0;
constexpr double kStraightYawRateThreshold = 0.05;
constexpr double kYawRateEmaAlpha = 0.2;
constexpr double kSpeedEmaAlpha = 0.4;

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

std::vector<const Feature*> CollectRecentHistory(const Obstacle& obstacle) {
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
  Eigen::Vector2d heading_direction(std::cos(heading), std::sin(heading));
  if (heading_direction.dot(reference_direction) >= 0.0) {
    return heading;
  }
  return NormalizeAngle(heading + M_PI);
}

double FitHeadingFromHistory(const std::vector<const Feature*>& history,
                             const double fallback_heading,
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

double MaxSpeedByObstacleType(const apollo::perception::PerceptionObstacle::Type type) {
  if (type == apollo::perception::PerceptionObstacle::PEDESTRIAN) {
    return FLAGS_pedestrian_max_speed;
  }
  return FLAGS_vehicle_max_speed;
}

double IntegrateCappedLongitudinalDisplacement(const double v_long,
                                               const double a_long,
                                               const double t,
                                               const double max_speed) {
  if (t <= 0.0) {
    return 0.0;
  }
  if (a_long <= 1e-6 || v_long >= max_speed) {
    return std::max(0.0, v_long * t + 0.5 * a_long * t * t);
  }

  const double time_to_cap = (max_speed - v_long) / a_long;
  if (time_to_cap <= 0.0 || time_to_cap >= t) {
    return std::max(0.0, v_long * t + 0.5 * a_long * t * t);
  }

  const double capped_displacement =
      v_long * time_to_cap + 0.5 * a_long * time_to_cap * time_to_cap;
  return capped_displacement + max_speed * (t - time_to_cap);
}

double ComputeHistoryTime(const std::vector<const Feature*>& history) {
  if (history.size() < 2) {
    return 0.0;
  }
  return std::max(0.0, history.back()->timestamp() - history.front()->timestamp());
}

double EstimateScalarSpeedFromHistory(const std::vector<const Feature*>& history,
                                      const double fallback_speed) {
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
    filtered_speed =
        initialized ? kSpeedEmaAlpha * segment_speed +
                           (1.0 - kSpeedEmaAlpha) * filtered_speed
                    : segment_speed;
    initialized = true;
  }
  return initialized ? filtered_speed : fallback_speed;
}

double EstimateLongitudinalAccelerationFromHistory(
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

double EstimateYawRateFromHistory(const std::vector<const Feature*>& history) {
  if (history.size() < 3) {
    return 0.0;
  }

  bool initialized = false;
  double filtered_yaw_rate = 0.0;
  double previous_heading = 0.0;
  bool has_previous_heading = false;

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
    filtered_yaw_rate =
        initialized ? kYawRateEmaAlpha * raw_yaw_rate +
                          (1.0 - kYawRateEmaAlpha) * filtered_yaw_rate
                    : raw_yaw_rate;
    initialized = true;
    previous_heading =
        previous_heading + AngleDiff(previous_heading, segment_heading);
  }
  return filtered_yaw_rate;
}

FreeMoveMotionEstimate EstimateFreeMoveMotion(const Obstacle& obstacle) {
  FreeMoveMotionEstimate estimate;
  const Feature& feature = obstacle.latest_feature();

  estimate.smoothed_theta = feature.velocity_heading();
  estimate.yaw_rate = 0.0;
  estimate.velocity =
      Eigen::Vector2d(feature.velocity().x(), feature.velocity().y());
  estimate.acc =
      Eigen::Vector2d(feature.acceleration().x(), feature.acceleration().y());
  estimate.position =
      Eigen::Vector2d(feature.position().x(), feature.position().y());
  estimate.max_speed = MaxSpeedByObstacleType(obstacle.type());

  const auto history = CollectRecentHistory(obstacle);
  estimate.history_time = ComputeHistoryTime(history);
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
  estimate.smoothed_theta =
      FitHeadingFromHistory(history, estimate.smoothed_theta, reference_direction);
  estimate.yaw_rate = EstimateYawRateFromHistory(history);

  if (estimate.history_time < kMinReliableHistorySec) {
    estimate.smoothed_theta = feature.velocity_heading();
    estimate.yaw_rate = 0.0;
    estimate.acc *= 0.5;
  } else if (std::fabs(estimate.yaw_rate) < kStraightYawRateThreshold) {
    estimate.yaw_rate = 0.0;
  }

  if (estimate.history_time >= kMinReliableHistorySec) {
    const double history_speed = std::min(
        estimate.max_speed, EstimateScalarSpeedFromHistory(history, fallback_speed));
    const double heading_cos = std::cos(estimate.smoothed_theta);
    const double heading_sin = std::sin(estimate.smoothed_theta);
    const double fallback_long_acc =
        estimate.acc(0) * heading_cos + estimate.acc(1) * heading_sin;
    const double history_long_acc =
        EstimateLongitudinalAccelerationFromHistory(history, fallback_long_acc);
    estimate.velocity =
        Eigen::Vector2d(history_speed * heading_cos, history_speed * heading_sin);
    estimate.acc = Eigen::Vector2d(history_long_acc * heading_cos,
                                   history_long_acc * heading_sin);
  }
  return estimate;
}

}  // namespace

using apollo::common::TrajectoryPoint;
using apollo::perception::PerceptionObstacle;

FreeMovePredictor::FreeMovePredictor() {
  predictor_type_ = ObstacleConf::FREE_MOVE_PREDICTOR;
}

bool FreeMovePredictor::Predict(
    const ADCTrajectoryContainer* adc_trajectory_container, Obstacle* obstacle,
    ObstaclesContainer* obstacles_container) {
  Clear();

  CHECK_NOTNULL(obstacle);
  CHECK_GT(obstacle->history_size(), 0U);

  obstacle->SetPredictorType(predictor_type_);

  const Feature& feature = obstacle->latest_feature();

  if (!feature.has_position() || !feature.has_velocity() ||
      !feature.position().has_x() || !feature.position().has_y()) {
    AERROR << "Obstacle [" << obstacle->id()
           << " is missing position or velocity";
    return false;
  }

  double prediction_total_time = FLAGS_prediction_trajectory_time_length;
  if (obstacle->type() == PerceptionObstacle::PEDESTRIAN) {
    prediction_total_time = FLAGS_prediction_trajectory_time_length;
  }

  const FreeMoveMotionEstimate motion = EstimateFreeMoveMotion(*obstacle);

  if (feature.predicted_trajectory().empty()) {
    std::vector<TrajectoryPoint> points;
    DrawFreeMoveTrajectoryPoints(motion.position, motion.velocity, motion.acc,
                                 motion.smoothed_theta, motion.yaw_rate,
                                 motion.history_time, motion.max_speed, 0.0,
                                 prediction_total_time,
                                 FLAGS_prediction_trajectory_time_resolution,
                                 &points);

    Trajectory trajectory = GenerateTrajectory(points);
    obstacle->mutable_latest_feature()->add_predicted_trajectory()->CopyFrom(
        trajectory);
    SetEqualProbability(1.0, 0, obstacle);
  } else {
    for (int i = 0; i < feature.predicted_trajectory_size(); ++i) {
      Trajectory* trajectory =
          obstacle->mutable_latest_feature()->mutable_predicted_trajectory(i);
      const int traj_size = trajectory->trajectory_point_size();
      if (traj_size == 0) {
        AERROR << "Empty predicted trajectory found";
        continue;
      }

      std::vector<TrajectoryPoint> points;
      const TrajectoryPoint& last_point =
          trajectory->trajectory_point(traj_size - 1);
      double start_theta = last_point.path_point().theta();
      if (std::fabs(AngleDiff(start_theta, motion.smoothed_theta)) <=
          kHeadingAgreementThreshold) {
        start_theta = motion.smoothed_theta;
      }
      const Eigen::Vector2d traj_position(last_point.path_point().x(),
                                          last_point.path_point().y());
      const Eigen::Vector2d traj_velocity(last_point.v() * std::cos(start_theta),
                                          last_point.v() * std::sin(start_theta));
      const Eigen::Vector2d traj_acc(last_point.a() * std::cos(start_theta),
                                     last_point.a() * std::sin(start_theta));
      const double last_relative_time = last_point.relative_time();
      DrawFreeMoveTrajectoryPoints(
          traj_position, traj_velocity, traj_acc, start_theta, motion.yaw_rate,
          motion.history_time, motion.max_speed, last_relative_time,
          prediction_total_time - last_relative_time,
          FLAGS_prediction_trajectory_time_resolution, &points);
      for (size_t j = 1; j < points.size(); ++j) {
        trajectory->add_trajectory_point()->CopyFrom(points[j]);
      }
    }
  }
  return true;
}

void FreeMovePredictor::DrawFreeMoveTrajectoryPoints(
    const Eigen::Vector2d& position, const Eigen::Vector2d& velocity,
    const Eigen::Vector2d& acc, const double theta, const double yaw_rate,
    const double history_time, const double max_speed, const double start_time,
    const double total_time, const double period,
    std::vector<TrajectoryPoint>* points) {
  // New along-heading extrapolation with lateral damping and CTRV fallback.
  points->clear();

  const double heading = theta;  // already smoothed by caller when available

  const double vx = velocity(0);
  const double vy = velocity(1);
  const double ax = acc(0);
  const double ay = acc(1);

  // Decompose into longitudinal and lateral components in heading frame
  double v_long = vx * std::cos(heading) + vy * std::sin(heading);
  double v_lat = -vx * std::sin(heading) + vy * std::cos(heading);
  double a_long = ax * std::cos(heading) + ay * std::sin(heading);
  double a_lat = -ax * std::sin(heading) + ay * std::cos(heading);

  // Clamp longitudinal acceleration to reasonable bounds and let lateral
  // dynamics decay quickly for low-confidence history.
  a_long = std::max(-3.0, std::min(2.0, a_long));
  v_long = std::max(0.0, std::min(max_speed, v_long));

  size_t num = static_cast<size_t>(total_time / period);

  double curr_x = 0.0;
  double curr_y = 0.0;

  // initial point (relative coordinates, will translate at the end)
  apollo::common::PathPoint path_point0;
  apollo::common::TrajectoryPoint tp0;
  path_point0.set_x(curr_x);
  path_point0.set_y(curr_y);
  path_point0.set_z(0.0);
  path_point0.set_theta(heading);
  tp0.mutable_path_point()->CopyFrom(path_point0);
  tp0.set_v(v_long);
  tp0.set_a(a_long);
  tp0.set_relative_time(start_time);
  points->push_back(tp0);

  for (size_t i = 1; i <= num; ++i) {
    const double t = static_cast<double>(i) * period;
    const double prev_t = static_cast<double>(i - 1) * period;
    const double theta_t =
        std::fabs(yaw_rate) > kStraightYawRateThreshold ? heading + yaw_rate * t
                                                        : heading;
    const double v_long_t =
        std::max(0.0, std::min(max_speed, v_long + a_long * t));
    if (std::fabs(yaw_rate) <= kStraightYawRateThreshold) {
      const double s_long =
          IntegrateCappedLongitudinalDisplacement(v_long, a_long, t, max_speed);
      double s_lat = v_lat * t + 0.5 * a_lat * t * t;
      if (history_time < kMinReliableHistorySec) {
        s_lat *= std::pow(0.5, static_cast<double>(i));
      } else {
        s_lat = 0.0;
      }
      curr_x = s_long * std::cos(heading) - s_lat * std::sin(heading);
      curr_y = s_long * std::sin(heading) + s_lat * std::cos(heading);
    } else {
      const double mid_t = 0.5 * (prev_t + t);
      const double theta_mid = heading + yaw_rate * mid_t;
      const double v_long_mid =
          std::max(0.0, std::min(max_speed, v_long + a_long * mid_t));
      double v_lat_mid = v_lat + a_lat * mid_t;
      if (history_time < kMinReliableHistorySec) {
        v_lat_mid *= std::pow(0.5, mid_t / period);
      } else {
        v_lat_mid = 0.0;
      }
      curr_x +=
          (v_long_mid * std::cos(theta_mid) - v_lat_mid * std::sin(theta_mid)) *
          period;
      curr_y +=
          (v_long_mid * std::sin(theta_mid) + v_lat_mid * std::cos(theta_mid)) *
          period;
    }

    apollo::common::PathPoint p;
    apollo::common::TrajectoryPoint point;
    p.set_x(curr_x);
    p.set_y(curr_y);
    p.set_z(0.0);
    p.set_theta(theta_t);
    point.mutable_path_point()->CopyFrom(p);
    point.set_v(v_long_t);
    point.set_a(a_long);
    point.set_relative_time(start_time + t);
    points->push_back(point);
  }

  // Translate to absolute coordinates
  for (size_t i = 0; i < points->size(); ++i) {
    ::apollo::prediction::predictor_util::TranslatePoint(
        position[0], position[1], &(points->operator[](i)));
  }
}

}  // namespace prediction
}  // namespace apollo
