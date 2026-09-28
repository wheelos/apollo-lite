// Copyright 2026 WheelOS. All Rights Reserved.
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

#include "modules/prediction/evaluator/vehicle/hivt_scene_evaluator.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "cyber/common/log.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_system_gflags.h"

namespace apollo {
namespace prediction {

namespace {

struct ModePrediction {
  int actor_id = 0;
  double probability = 0.0;
  std::vector<common::TrajectoryPoint> points;
};

bool BuildPredictions(const HiVTSceneInput& input,
                      const HiVTSceneOutput& output,
                      const std::vector<Obstacle*>& targets,
                      std::vector<ModePrediction>* predictions) {
  if (predictions == nullptr ||
      output.trajectories.size() != static_cast<std::size_t>(kHiVTNumModes) *
                                        input.actor_count * kHiVTFutureSteps *
                                        4 ||
      output.logits.size() !=
          static_cast<std::size_t>(input.actor_count) * kHiVTNumModes) {
    AERROR << "HiVT returned output tensors with invalid dimensions";
    return false;
  }
  std::unordered_map<int, int> actor_indices;
  for (int index = 0; index < input.actor_count; ++index) {
    actor_indices.emplace(input.actor_ids[index], index);
  }

  predictions->clear();
  predictions->reserve(targets.size() * kHiVTNumModes);
  const double time_step = FLAGS_prediction_trajectory_time_resolution;
  for (const Obstacle* actor : targets) {
    const auto actor_it = actor_indices.find(actor->id());
    if (actor_it == actor_indices.end()) {
      AERROR << "HiVT output is missing target actor " << actor->id();
      return false;
    }
    const int actor_index = actor_it->second;
    double max_logit = -std::numeric_limits<double>::infinity();
    for (int mode = 0; mode < kHiVTNumModes; ++mode) {
      const float logit = output.logits[actor_index * kHiVTNumModes + mode];
      if (!std::isfinite(logit)) {
        AERROR << "HiVT returned a non-finite mode logit for actor "
               << actor->id();
        return false;
      }
      max_logit = std::max(max_logit, static_cast<double>(logit));
    }
    double probability_sum = 0.0;
    std::array<double, kHiVTNumModes> probabilities{};
    for (int mode = 0; mode < kHiVTNumModes; ++mode) {
      probabilities[mode] = std::exp(
          output.logits[actor_index * kHiVTNumModes + mode] - max_logit);
      probability_sum += probabilities[mode];
    }
    if (!std::isfinite(probability_sum) || probability_sum <= 0.0) {
      AERROR << "HiVT returned invalid mode probabilities for actor "
             << actor->id();
      return false;
    }
    for (double& probability : probabilities) {
      probability /= probability_sum;
    }

    const auto& latest = actor->latest_feature();
    for (int mode = 0; mode < kHiVTNumModes; ++mode) {
      ModePrediction prediction;
      prediction.actor_id = actor->id();
      prediction.probability = probabilities[mode];
      prediction.points.reserve(kHiVTFutureSteps);
      double previous_x = latest.position().x();
      double previous_y = latest.position().y();
      for (int step = 0; step < kHiVTFutureSteps; ++step) {
        const std::size_t output_index =
            ((static_cast<std::size_t>(mode) * input.actor_count +
              actor_index) *
                 kHiVTFutureSteps +
             step) *
            4;
        const double dx = output.trajectories[output_index];
        const double dy = output.trajectories[output_index + 1];
        if (!std::isfinite(dx) || !std::isfinite(dy) ||
            !std::isfinite(output.trajectories[output_index + 2]) ||
            !std::isfinite(output.trajectories[output_index + 3])) {
          AERROR << "HiVT returned a non-finite trajectory for actor "
                 << actor->id();
          return false;
        }
        const double x =
            input.origin_x + dx * input.origin_cos - dy * input.origin_sin;
        const double y =
            input.origin_y + dx * input.origin_sin + dy * input.origin_cos;
        common::TrajectoryPoint point;
        point.mutable_path_point()->set_x(x);
        point.mutable_path_point()->set_y(y);
        point.mutable_path_point()->set_theta(
            std::hypot(x - previous_x, y - previous_y) > 1e-6
                ? std::atan2(y - previous_y, x - previous_x)
                : latest.velocity_heading());
        point.set_relative_time((step + 1) * time_step);
        point.set_v(step == 0 ? latest.speed()
                              : std::hypot(x - previous_x, y - previous_y) /
                                    time_step);
        prediction.points.push_back(std::move(point));
        previous_x = x;
        previous_y = y;
      }
      predictions->push_back(std::move(prediction));
    }
  }
  return true;
}

}  // namespace

HiVTSceneEvaluator::HiVTSceneEvaluator() {
  evaluator_type_ = ObstacleConf::HIVT_SCENE_EVALUATOR;
  if (FLAGS_use_cuda && !FLAGS_hivt_tensorrt_engine.empty()) {
    executor_ready_ = executor_.Init(FLAGS_hivt_tensorrt_engine,
                                     FLAGS_hivt_tensorrt_device_id);
  } else {
    AERROR << "HiVT TensorRT evaluator requires CUDA and an engine path";
  }
}

bool HiVTSceneEvaluator::Evaluate(Obstacle*, ObstaclesContainer*) {
  AERROR << "HiVT inference must be executed once for the full scene";
  return false;
}

bool HiVTSceneEvaluator::EvaluateScene(
    const std::vector<Obstacle*>& targets,
    ObstaclesContainer* obstacles_container) {
  if (!executor_ready_ || obstacles_container == nullptr || targets.empty()) {
    AERROR << "HiVT scene evaluator is not ready or has no targets";
    return false;
  }
  std::vector<Obstacle*> actors;
  Obstacle* ego = obstacles_container->GetObstacle(FLAGS_ego_vehicle_id);
  if (ego == nullptr) {
    AERROR << "HiVT scene evaluator could not find the ego vehicle";
    return false;
  }
  const std::vector<Obstacle*> selected_targets =
      HiVTSceneFeatureBuilder::SelectTargets(ego, targets);
  if (selected_targets.size() < targets.size()) {
    AWARN << "HiVT scene admission retained " << selected_targets.size()
          << " of " << targets.size()
          << " caution targets with valid history, prioritized by distance";
  }
  if (selected_targets.empty()) {
    ADEBUG << "HiVT scene has no targets with sufficient history";
    return true;
  }

  std::vector<Obstacle*> scene_candidates;
  scene_candidates.reserve(
      obstacles_container->curr_frame_considered_obstacle_ids().size());
  for (const int actor_id :
       obstacles_container->curr_frame_considered_obstacle_ids()) {
    Obstacle* actor = obstacles_container->GetObstacle(actor_id);
    if (actor != nullptr) {
      scene_candidates.push_back(actor);
    }
  }
  actors =
      feature_builder_.SelectActors(ego, selected_targets, scene_candidates);
  if (actors.empty()) {
    AERROR << "HiVT scene could not select actors within the input capacity";
    return false;
  }

  HiVTSceneInput input;
  if (!feature_builder_.Build(ego, actors, &input)) {
    return false;
  }
  HiVTSceneOutput output;
  if (!executor_.Run(input, &output)) {
    return false;
  }

  std::vector<ModePrediction> predictions;
  if (!BuildPredictions(input, output, selected_targets, &predictions)) {
    return false;
  }
  std::unordered_map<int, Feature*> target_features;
  for (Obstacle* target : selected_targets) {
    if (target == nullptr || target_features.count(target->id()) != 0) {
      AERROR << "HiVT target list contains a null or duplicate actor";
      return false;
    }
    Feature* feature = target->mutable_latest_feature();
    if (feature == nullptr) {
      AERROR << "HiVT target has no mutable latest feature: " << target->id();
      return false;
    }
    target_features.emplace(target->id(), feature);
  }
  for (Obstacle* target : selected_targets) {
    target->SetEvaluatorType(evaluator_type_);
    target_features.at(target->id())->clear_predicted_trajectory();
  }
  for (auto& prediction : predictions) {
    Trajectory* trajectory =
        target_features.at(prediction.actor_id)->add_predicted_trajectory();
    trajectory->set_probability(prediction.probability);
    for (const auto& point : prediction.points) {
      trajectory->add_trajectory_point()->CopyFrom(point);
    }
  }
  return true;
}

}  // namespace prediction
}  // namespace apollo
