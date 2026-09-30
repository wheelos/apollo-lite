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

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/box2d.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/prediction/common/feature_output.h"
#include "modules/prediction/common/prediction_constants.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_system_gflags.h"
#include "modules/prediction/common/prediction_util.h"
#include "modules/prediction/container/adc_trajectory/adc_trajectory_container.h"
#include "modules/prediction/container/container_manager.h"

namespace apollo {
namespace prediction {

using apollo::common::PathPoint;
using apollo::common::TrajectoryPoint;
using apollo::common::VehicleConfigHelper;
using apollo::common::math::Box2d;
using apollo::common::math::InterpolateUsingLinearApproximation;
using apollo::common::math::Vec2d;
using apollo::hdmap::LaneInfo;
using apollo::prediction::math_util::GetSByConstantAcceleration;

namespace {

constexpr int kMaxLaneCandidates = 4;
constexpr double kSoftClearanceMeters = 3.0;
constexpr double kFootprintUncertaintyMargin = 0.25;
constexpr double kTtcThresholdSeconds = 3.0;
constexpr double kHeadingAlignmentThreshold = 0.35;
constexpr double kConflictGapThresholdSeconds = 3.0;
constexpr double kMinimumAdcScoringHorizonSeconds = 3.0;
const std::vector<double> kCandidateAccelerations = {0.0, -0.75, -1.5, -2.5};

struct InteractionCandidate {
  Trajectory trajectory;
  double score = -std::numeric_limits<double>::infinity();
  double probability = 0.0;
};

}  // namespace

InteractionPredictor::InteractionPredictor() {
  predictor_type_ = ObstacleConf::INTERACTION_PREDICTOR;
}

bool InteractionPredictor::Predict(
    const ADCTrajectoryContainer* adc_trajectory_container, Obstacle* obstacle,
    ObstaclesContainer* obstacles_container) {
  Clear();
  CHECK_NOTNULL(obstacle);
  CHECK_GT(obstacle->history_size(), 0U);

  std::vector<TrajectoryPoint> adc_trajectory;
  if (!BuildADCTrajectory(
          adc_trajectory_container, obstacle->latest_feature().timestamp(),
          FLAGS_prediction_trajectory_time_resolution,
          FLAGS_prediction_trajectory_time_length, &adc_trajectory)) {
    AERROR << "Interactive prediction has no valid time-aligned ADC plan for "
           << obstacle->id();
    return false;
  }

  obstacle->SetPredictorType(predictor_type_);

  Feature* feature_ptr = obstacle->mutable_latest_feature();

  if (!feature_ptr->lane().has_lane_graph()) {
    AERROR << "Obstacle [" << obstacle->id() << "] has no lane graph.";
    return false;
  }
  auto* lane_graph = feature_ptr->mutable_lane()->mutable_lane_graph();

  std::vector<int> lane_indices;
  lane_indices.reserve(lane_graph->lane_sequence_size());
  std::vector<double> lane_priors(lane_graph->lane_sequence_size(), 0.0);
  bool has_lane_priors = false;
  for (int i = 0; i < lane_graph->lane_sequence_size(); ++i) {
    const auto& sequence = lane_graph->lane_sequence(i);
    if (sequence.lane_segment_size() > 0) {
      lane_indices.push_back(i);
      if (std::isfinite(sequence.probability()) &&
          sequence.probability() > 0.0) {
        lane_priors[i] = sequence.probability();
        has_lane_priors = true;
      }
    }
  }
  if (lane_indices.empty()) {
    AERROR << "Obstacle [" << obstacle->id() << "] has no lane candidates";
    return false;
  }
  if (!has_lane_priors) {
    for (const int lane_index : lane_indices) {
      lane_priors[lane_index] = 1.0;
    }
  }
  std::stable_sort(lane_indices.begin(), lane_indices.end(),
                   [&lane_priors](const int left, const int right) {
                     return lane_priors[left] > lane_priors[right];
                   });
  std::string current_lane_id;
  if (feature_ptr->has_lane() && feature_ptr->lane().has_lane_feature()) {
    current_lane_id = feature_ptr->lane().lane_feature().lane_id();
  }
  std::vector<int> selected_lane_indices;
  std::vector<LaneChangeType> selected_maneuvers;
  for (const int lane_index : lane_indices) {
    const auto maneuver = GetLaneChangeType(
        current_lane_id, lane_graph->lane_sequence(lane_index));
    if (maneuver != LaneChangeType::INVALID &&
        std::find(selected_maneuvers.begin(), selected_maneuvers.end(),
                  maneuver) == selected_maneuvers.end()) {
      selected_lane_indices.push_back(lane_index);
      selected_maneuvers.push_back(maneuver);
      if (selected_lane_indices.size() == kMaxLaneCandidates) {
        break;
      }
    }
  }
  for (const int lane_index : lane_indices) {
    if (selected_lane_indices.size() == kMaxLaneCandidates) {
      break;
    }
    if (std::find(selected_lane_indices.begin(), selected_lane_indices.end(),
                  lane_index) == selected_lane_indices.end()) {
      selected_lane_indices.push_back(lane_index);
    }
  }
  lane_indices.swap(selected_lane_indices);

  std::vector<InteractionCandidate> candidates;
  candidates.reserve(lane_indices.size() * kCandidateAccelerations.size());
  for (const int lane_index : lane_indices) {
    const LaneSequence& lane_sequence = lane_graph->lane_sequence(lane_index);
    std::vector<double> accelerations = kCandidateAccelerations;
    if (lane_sequence.has_stop_sign()) {
      double stop_acceleration = 0.0;
      const double stop_distance =
          lane_sequence.stop_sign().lane_sequence_s() - lane_sequence.lane_s();
      if (SupposedToStop(*feature_ptr, stop_distance, &stop_acceleration) &&
          std::find_if(accelerations.begin(), accelerations.end(),
                       [stop_acceleration](const double acceleration) {
                         return std::abs(acceleration - stop_acceleration) <
                                1e-3;
                       }) == accelerations.end()) {
        accelerations.push_back(stop_acceleration);
      }
    }
    const double raw_lane_prior = lane_priors[lane_index];
    if (raw_lane_prior <= 0.0) {
      continue;
    }
    for (const double acceleration : accelerations) {
      std::vector<TrajectoryPoint> points;
      if (!DrawTrajectory(*obstacle, lane_sequence, acceleration,
                          FLAGS_prediction_trajectory_time_length,
                          FLAGS_prediction_trajectory_time_resolution,
                          &points)) {
        continue;
      }
      InteractionCandidate candidate;
      candidate.trajectory = GenerateTrajectory(points);
      const double prior =
          raw_lane_prior * std::exp(-0.1 * acceleration * acceleration);
      const double cost =
          ComputeTrajectoryCost(*obstacle, lane_sequence, acceleration,
                                candidate.trajectory, adc_trajectory);
      if (prior > 0.0 && std::isfinite(prior) && std::isfinite(cost)) {
        candidate.score =
            std::log(prior) - FLAGS_likelihood_exp_coefficient * cost;
        if (std::isfinite(candidate.score)) {
          candidates.emplace_back(std::move(candidate));
        }
      }
    }
  }
  if (candidates.empty()) {
    AERROR << "No feasible interaction candidates for obstacle "
           << obstacle->id();
    return false;
  }

  const double max_score =
      std::max_element(candidates.begin(), candidates.end(),
                       [](const auto& left, const auto& right) {
                         return left.score < right.score;
                       })
          ->score;
  double probability_sum = 0.0;
  for (auto& candidate : candidates) {
    candidate.probability = std::exp(candidate.score - max_score);
    probability_sum += candidate.probability;
  }
  if (!std::isfinite(probability_sum) || probability_sum <= 0.0) {
    AERROR << "Invalid interaction candidate posterior for obstacle "
           << obstacle->id();
    return false;
  }
  for (const auto& candidate : candidates) {
    auto* trajectory = feature_ptr->add_predicted_trajectory();
    trajectory->CopyFrom(candidate.trajectory);
    trajectory->set_probability(candidate.probability / probability_sum);
  }
  return true;
}

void InteractionPredictor::Clear() { Predictor::Clear(); }

bool InteractionPredictor::BuildADCTrajectory(
    const ADCTrajectoryContainer* adc_trajectory_container,
    const double obstacle_timestamp, const double time_resolution,
    const double prediction_horizon,
    std::vector<TrajectoryPoint>* adc_trajectory) {
  CHECK_NOTNULL(adc_trajectory);
  adc_trajectory->clear();
  if (adc_trajectory_container == nullptr ||
      !std::isfinite(obstacle_timestamp) || !std::isfinite(time_resolution) ||
      !std::isfinite(prediction_horizon) || time_resolution <= 0.0 ||
      prediction_horizon <= 0.0) {
    return false;
  }
  const auto& adc_trajectory_msg = adc_trajectory_container->adc_trajectory();
  const auto& points = adc_trajectory_msg.trajectory_point();
  if (!adc_trajectory_msg.has_header() || points.size() < 2) {
    return false;
  }
  for (int i = 0; i < points.size(); ++i) {
    if (!std::isfinite(points[i].relative_time()) ||
        !points[i].has_path_point() ||
        !std::isfinite(points[i].path_point().x()) ||
        !std::isfinite(points[i].path_point().y()) ||
        !std::isfinite(points[i].path_point().theta()) ||
        !std::isfinite(points[i].v()) ||
        (i > 0 && points[i].relative_time() <= points[i - 1].relative_time())) {
      return false;
    }
  }
  const double plan_time_offset =
      obstacle_timestamp - adc_trajectory_msg.header().timestamp_sec();
  if (!std::isfinite(plan_time_offset) ||
      plan_time_offset < points[0].relative_time() ||
      plan_time_offset > points.rbegin()->relative_time()) {
    return false;
  }
  const double available_horizon =
      points.rbegin()->relative_time() - plan_time_offset;
  const double required_horizon =
      std::min(prediction_horizon, kMinimumAdcScoringHorizonSeconds);
  if (available_horizon + 1e-6 < required_horizon) {
    return false;
  }
  const int full_sample_count =
      static_cast<int>(std::floor(prediction_horizon / time_resolution));
  const int available_sample_count =
      static_cast<int>(
          std::floor((available_horizon + 1e-6) / time_resolution)) +
      1;
  const int sample_count = std::min(full_sample_count, available_sample_count);
  if (sample_count <= 0) {
    return false;
  }
  adc_trajectory->reserve(sample_count);
  for (int i = 0; i < sample_count; ++i) {
    const double prediction_time = i * time_resolution;
    const double plan_time = plan_time_offset + prediction_time;
    auto upper =
        std::lower_bound(points.begin(), points.end(), plan_time,
                         [](const TrajectoryPoint& point, const double time) {
                           return point.relative_time() < time;
                         });
    if (upper == points.end()) {
      adc_trajectory->clear();
      return false;
    }
    TrajectoryPoint sample;
    if (upper == points.begin() ||
        std::abs(upper->relative_time() - plan_time) < 1e-6) {
      sample = *upper;
    } else {
      sample =
          InterpolateUsingLinearApproximation(*(upper - 1), *upper, plan_time);
    }
    sample.set_relative_time(prediction_time);
    adc_trajectory->emplace_back(std::move(sample));
  }
  return adc_trajectory->size() == static_cast<std::size_t>(sample_count);
}

bool InteractionPredictor::DrawTrajectory(
    const Obstacle& obstacle, const LaneSequence& lane_sequence,
    const double lon_acceleration, const double total_time, const double period,
    std::vector<TrajectoryPoint>* trajectory_points) {
  // Sanity check.
  CHECK_NOTNULL(trajectory_points);
  trajectory_points->clear();
  const Feature& feature = obstacle.latest_feature();
  if (!feature.has_position() || !feature.has_velocity() ||
      !feature.position().has_x() || !feature.position().has_y() ||
      !std::isfinite(feature.position().x()) ||
      !std::isfinite(feature.position().y()) ||
      !std::isfinite(feature.speed()) || feature.speed() < 0.0 ||
      feature.speed() > FLAGS_vehicle_max_speed ||
      lon_acceleration < FLAGS_vehicle_min_linear_acc ||
      lon_acceleration > FLAGS_vehicle_max_linear_acc ||
      lane_sequence.lane_segment_size() == 0 || period <= 0.0 ||
      total_time <= 0.0) {
    AERROR << "Obstacle [" << obstacle.id()
           << " is missing position or velocity";
    return false;
  }

  Eigen::Vector2d position(feature.position().x(), feature.position().y());
  double speed = feature.speed();

  int lane_segment_index = 0;
  std::string lane_id =
      lane_sequence.lane_segment(lane_segment_index).lane_id();
  std::shared_ptr<const LaneInfo> lane_info = PredictionMap::LaneById(lane_id);
  if (lane_info == nullptr) {
    AERROR << "Cannot find lane [" << lane_id << "] for obstacle "
           << obstacle.id();
    return false;
  }
  double lane_s = 0.0;
  double lane_l = 0.0;
  if (!PredictionMap::GetProjection(position, lane_info, &lane_s, &lane_l)) {
    AERROR << "Failed in getting lane s and lane l";
    return false;
  }
  double approach_rate = FLAGS_go_approach_rate;
  if (!lane_sequence.vehicle_on_lane()) {
    approach_rate = FLAGS_cutin_approach_rate;
  }
  size_t total_num = static_cast<size_t>(total_time / period);
  for (size_t i = 0; i < total_num; ++i) {
    double relative_time = static_cast<double>(i) * period;
    Eigen::Vector2d point;
    double theta = M_PI;
    if (!PredictionMap::SmoothPointFromLane(lane_id, lane_s, lane_l, &point,
                                            &theta)) {
      AERROR << "Unable to get smooth point from lane [" << lane_id
             << "] with s [" << lane_s << "] and l [" << lane_l << "]";
      break;
    }
    TrajectoryPoint trajectory_point;
    PathPoint path_point;
    path_point.set_x(point.x());
    path_point.set_y(point.y());
    path_point.set_z(0.0);
    path_point.set_theta(theta);
    path_point.set_lane_id(lane_id);
    trajectory_point.mutable_path_point()->CopyFrom(path_point);
    trajectory_point.set_v(speed);
    trajectory_point.set_a(lon_acceleration);
    trajectory_point.set_relative_time(relative_time);
    trajectory_points->emplace_back(std::move(trajectory_point));

    lane_s += std::max(
        0.0, speed * period + 0.5 * lon_acceleration * period * period);
    speed = std::max(0.0, speed + lon_acceleration * period);

    while (lane_s > lane_info->total_length() &&
           lane_segment_index + 1 < lane_sequence.lane_segment_size()) {
      lane_segment_index += 1;
      lane_s = lane_s - lane_info->total_length();
      lane_id = lane_sequence.lane_segment(lane_segment_index).lane_id();
      lane_info = PredictionMap::LaneById(lane_id);
      if (lane_info == nullptr) {
        AERROR << "Cannot find lane [" << lane_id << "] for obstacle "
               << obstacle.id();
        return false;
      }
    }

    lane_l *= approach_rate;
  }

  return trajectory_points->size() == total_num;
}

double InteractionPredictor::ComputeTrajectoryCost(
    const Obstacle& obstacle, const LaneSequence& lane_sequence,
    const double acceleration, const Trajectory& candidate,
    const std::vector<TrajectoryPoint>& adc_trajectory) {
  double speed = obstacle.latest_feature().speed();
  double total_cost = 0.0;

  double lon_acc_cost = LongitudinalAccelerationCost(acceleration);
  total_cost += FLAGS_longitudinal_acceleration_cost_weight * lon_acc_cost;

  double centri_acc_cost =
      CentripetalAccelerationCost(lane_sequence, speed, acceleration);
  total_cost += FLAGS_centripedal_acceleration_cost_weight * centri_acc_cost;

  const double collision_cost =
      CollisionWithEgoVehicleCost(obstacle, candidate, adc_trajectory);
  if (!std::isfinite(collision_cost)) {
    return collision_cost;
  }
  total_cost += FLAGS_collision_cost_weight * collision_cost;

  if (FLAGS_prediction_offline_mode ==
      PredictionConstants::kDumpDataForTuning) {
    std::vector<double> cost_values = {lon_acc_cost, centri_acc_cost,
                                       collision_cost};
    FeatureOutput::InsertDataForTuning(obstacle.latest_feature(), cost_values,
                                       "interaction", lane_sequence,
                                       adc_trajectory);
  }

  return total_cost;
}

double InteractionPredictor::LongitudinalAccelerationCost(
    const double acceleration) {
  return acceleration * acceleration;
}

double InteractionPredictor::CentripetalAccelerationCost(
    const LaneSequence& lane_sequence, const double speed,
    const double acceleration) {
  double cost_abs_sum = 0.0;
  double cost_sqr_sum = 0.0;
  double curr_time = 0.0;
  while (curr_time < FLAGS_prediction_trajectory_time_length) {
    double s = GetSByConstantAcceleration(speed, acceleration, curr_time);
    double v = std::max(0.0, speed + acceleration * curr_time);
    double kappa = GetLaneSequenceCurvatureByS(lane_sequence, s);
    double centri_acc = v * v * kappa;
    cost_abs_sum += std::abs(centri_acc);
    cost_sqr_sum += centri_acc * centri_acc;
    curr_time += FLAGS_collision_cost_time_resolution;
  }
  return cost_sqr_sum / (cost_abs_sum + FLAGS_double_precision);
}

double InteractionPredictor::CollisionWithEgoVehicleCost(
    const Obstacle& obstacle, const Trajectory& candidate,
    const std::vector<TrajectoryPoint>& adc_trajectory) const {
  const auto& vehicle_param = VehicleConfigHelper::GetConfig().vehicle_param();
  const auto& feature = obstacle.latest_feature();
  const int sample_count = std::min(candidate.trajectory_point_size(),
                                    static_cast<int>(adc_trajectory.size()));
  if (feature.length() <= 0.0 || feature.width() <= 0.0 ||
      vehicle_param.length() <= 0.0 || vehicle_param.width() <= 0.0 ||
      sample_count <= 0) {
    return std::numeric_limits<double>::infinity();
  }
  double clearance_cost = 0.0;
  double ttc_cost = 0.0;
  for (int i = 0; i < sample_count; ++i) {
    const auto& target_point = candidate.trajectory_point(i);
    const auto& ego_point = adc_trajectory[i];
    if (!target_point.has_path_point() || !std::isfinite(target_point.v()) ||
        !std::isfinite(target_point.path_point().x()) ||
        !std::isfinite(target_point.path_point().y()) ||
        !std::isfinite(target_point.path_point().theta())) {
      return std::numeric_limits<double>::infinity();
    }
    const auto& target_path = target_point.path_point();
    const auto& ego_path = ego_point.path_point();
    const Box2d target_box(Vec2d(target_path.x(), target_path.y()),
                           target_path.theta(),
                           feature.length() + 2.0 * kFootprintUncertaintyMargin,
                           feature.width() + 2.0 * kFootprintUncertaintyMargin);
    const Box2d ego_box(
        Vec2d(ego_path.x(), ego_path.y()), ego_path.theta(),
        vehicle_param.length() + 2.0 * kFootprintUncertaintyMargin,
        vehicle_param.width() + 2.0 * kFootprintUncertaintyMargin);
    if (target_box.HasOverlap(ego_box)) {
      if (target_point.relative_time() > 0.2) {
        return std::numeric_limits<double>::infinity();
      }
      clearance_cost += 1.0;
    } else {
      const double clearance = target_box.DistanceTo(ego_box);
      if (clearance < kSoftClearanceMeters) {
        const double normalized =
            (kSoftClearanceMeters - clearance) / kSoftClearanceMeters;
        clearance_cost += normalized * normalized;
      }
    }

    const double heading_difference = std::abs(
        std::remainder(target_path.theta() - ego_path.theta(), 2.0 * M_PI));
    if (heading_difference < kHeadingAlignmentThreshold) {
      const double dx = ego_path.x() - target_path.x();
      const double dy = ego_path.y() - target_path.y();
      const double longitudinal_gap = dx * std::cos(target_path.theta()) +
                                      dy * std::sin(target_path.theta());
      const double lateral_gap = -dx * std::sin(target_path.theta()) +
                                 dy * std::cos(target_path.theta());
      const double lateral_clearance =
          (feature.width() + vehicle_param.width()) * 0.5 +
          2.0 * kFootprintUncertaintyMargin;
      const double relative_speed =
          ego_point.v() * std::cos(heading_difference) - target_point.v();
      const double longitudinal_sign = longitudinal_gap > 0.0 ? 1.0 : -1.0;
      const double closing_speed = -relative_speed * longitudinal_sign;
      const double bumper_gap =
          std::max(0.0, std::abs(longitudinal_gap) -
                            0.5 * (feature.length() + vehicle_param.length() +
                                   4.0 * kFootprintUncertaintyMargin));
      if (bumper_gap > 0.0 && std::abs(lateral_gap) < lateral_clearance &&
          closing_speed > 0.1) {
        const double ttc = bumper_gap / closing_speed;
        if (ttc < kTtcThresholdSeconds) {
          const double normalized =
              (kTtcThresholdSeconds - ttc) / kTtcThresholdSeconds;
          ttc_cost = std::max(ttc_cost, normalized * normalized);
        }
      }
    }
  }
  const double conflict_cost =
      ConflictZoneTimeGapCost(obstacle, candidate, adc_trajectory,
                              vehicle_param.length(), vehicle_param.width());
  return clearance_cost / sample_count + 2.0 * ttc_cost + 2.0 * conflict_cost;
}

double InteractionPredictor::ConflictZoneTimeGapCost(
    const Obstacle& obstacle, const Trajectory& candidate,
    const std::vector<TrajectoryPoint>& adc_trajectory, const double ego_length,
    const double ego_width) const {
  const auto& feature = obstacle.latest_feature();
  const int sample_count = std::min(candidate.trajectory_point_size(),
                                    static_cast<int>(adc_trajectory.size()));
  if (sample_count < 4) {
    return 0.0;
  }

  const int stride = std::max(1, sample_count / 40);
  double min_distance_squared = std::numeric_limits<double>::infinity();
  int target_conflict_index = -1;
  int ego_conflict_index = -1;
  for (int target_index = 2; target_index < sample_count;
       target_index += stride) {
    const auto& target_path =
        candidate.trajectory_point(target_index).path_point();
    for (int ego_index = 2; ego_index < sample_count; ego_index += stride) {
      const auto& ego_path = adc_trajectory[ego_index].path_point();
      const double heading_difference = std::abs(
          std::remainder(target_path.theta() - ego_path.theta(), 2.0 * M_PI));
      if (heading_difference < kHeadingAlignmentThreshold ||
          std::abs(M_PI - heading_difference) < kHeadingAlignmentThreshold) {
        continue;
      }
      const double dx = target_path.x() - ego_path.x();
      const double dy = target_path.y() - ego_path.y();
      const double distance_squared = dx * dx + dy * dy;
      if (distance_squared < min_distance_squared) {
        min_distance_squared = distance_squared;
        target_conflict_index = target_index;
        ego_conflict_index = ego_index;
      }
    }
  }
  if (target_conflict_index < 0 ||
      min_distance_squared > kSoftClearanceMeters * kSoftClearanceMeters) {
    return 0.0;
  }

  const auto& target_conflict_point =
      candidate.trajectory_point(target_conflict_index).path_point();
  const auto& ego_conflict_point =
      adc_trajectory[ego_conflict_index].path_point();
  const Vec2d conflict_center(
      0.5 * (target_conflict_point.x() + ego_conflict_point.x()),
      0.5 * (target_conflict_point.y() + ego_conflict_point.y()));

  auto get_occupancy_interval = [&conflict_center](
                                    const auto& point_at, const int count,
                                    const double length, const double width,
                                    double* enter_time, double* exit_time) {
    bool occupied = false;
    for (int i = 0; i < count; ++i) {
      const auto& point = point_at(i);
      const auto& path = point.path_point();
      const Box2d box(Vec2d(path.x(), path.y()), path.theta(),
                      length + 2.0 * kFootprintUncertaintyMargin,
                      width + 2.0 * kFootprintUncertaintyMargin);
      if (box.IsPointIn(conflict_center)) {
        if (!occupied) {
          *enter_time = point.relative_time();
          occupied = true;
        }
        *exit_time = point.relative_time();
      } else if (occupied) {
        break;
      }
    }
    return occupied;
  };

  double target_enter = 0.0;
  double target_exit = 0.0;
  double ego_enter = 0.0;
  double ego_exit = 0.0;
  if (!get_occupancy_interval(
          [&candidate](const int i) -> const TrajectoryPoint& {
            return candidate.trajectory_point(i);
          },
          candidate.trajectory_point_size(), feature.length(), feature.width(),
          &target_enter, &target_exit) ||
      !get_occupancy_interval(
          [&adc_trajectory](const int i) -> const TrajectoryPoint& {
            return adc_trajectory[i];
          },
          sample_count, ego_length, ego_width, &ego_enter, &ego_exit)) {
    return 0.0;
  }

  const double time_gap =
      target_exit < ego_enter
          ? ego_enter - target_exit
          : (ego_exit < target_enter ? target_enter - ego_exit : 0.0);
  if (time_gap >= kConflictGapThresholdSeconds) {
    return 0.0;
  }
  const double normalized =
      (kConflictGapThresholdSeconds - time_gap) / kConflictGapThresholdSeconds;
  return normalized * normalized;
}

}  // namespace prediction
}  // namespace apollo
