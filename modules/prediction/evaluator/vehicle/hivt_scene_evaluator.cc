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
#include <chrono>
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

bool HasFiniteSceneTransform(const HiVTSceneInput& input) {
  return std::isfinite(input.origin_x) && std::isfinite(input.origin_y) &&
         std::isfinite(input.origin_cos) && std::isfinite(input.origin_sin);
}

}  // namespace

bool DecodeHiVTSceneOutput(const HiVTSceneInput& input,
                           const HiVTSceneOutput& output,
                           const std::vector<Obstacle*>& targets,
                           std::vector<HiVTModePrediction>* predictions) {
  if (predictions == nullptr ||
      input.actor_ids.size() != static_cast<std::size_t>(input.actor_count) ||
      input.rotate_angles.size() !=
          static_cast<std::size_t>(input.actor_count) ||
      !HasFiniteSceneTransform(input) ||
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
    if (actor == nullptr || actor->history_size() == 0 ||
        !actor->latest_feature().has_position()) {
      AERROR << "HiVT output target is null or has no current position";
      return false;
    }
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
    const double actor_rotation = input.rotate_angles[actor_index];
    if (!std::isfinite(actor_rotation) ||
        !std::isfinite(latest.position().x()) ||
        !std::isfinite(latest.position().y())) {
      AERROR << "HiVT output target has an invalid coordinate transform: "
             << actor->id();
      return false;
    }
    const double actor_cos = std::cos(actor_rotation);
    const double actor_sin = std::sin(actor_rotation);
    for (int mode = 0; mode < kHiVTNumModes; ++mode) {
      HiVTModePrediction prediction;
      prediction.actor_id = actor->id();
      prediction.probability = probabilities[mode];
      prediction.points.reserve(kHiVTFutureSteps);
      double previous_x = latest.position().x();
      double previous_y = latest.position().y();
      double previous_speed = latest.speed();
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
        const double scene_dx = dx * actor_cos - dy * actor_sin;
        const double scene_dy = dx * actor_sin + dy * actor_cos;
        const double world_dx =
            scene_dx * input.origin_cos - scene_dy * input.origin_sin;
        const double world_dy =
            scene_dx * input.origin_sin + scene_dy * input.origin_cos;
        const double x = latest.position().x() + world_dx;
        const double y = latest.position().y() + world_dy;
        const double displacement = std::hypot(x - previous_x, y - previous_y);
        const double speed = displacement / time_step;
        common::TrajectoryPoint point;
        point.mutable_path_point()->set_x(x);
        point.mutable_path_point()->set_y(y);
        point.mutable_path_point()->set_theta(
            displacement > 1e-6
                ? std::atan2(y - previous_y, x - previous_x)
                : latest.velocity_heading());
        point.set_relative_time((step + 1) * time_step);
        point.set_v(speed);
        point.set_a((speed - previous_speed) / time_step);
        prediction.points.push_back(std::move(point));
        previous_x = x;
        previous_y = y;
        previous_speed = speed;
      }
      predictions->push_back(std::move(prediction));
    }
  }
  return true;
}

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
  const auto profile_start = std::chrono::steady_clock::now();
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
  const auto actor_selection_end = std::chrono::steady_clock::now();

  HiVTSceneInput input;
  if (!feature_builder_.Build(ego, actors, &input)) {
    return false;
  }
  const auto feature_build_end = std::chrono::steady_clock::now();
  HiVTSceneOutput output;
  if (!executor_.Run(input, &output)) {
    return false;
  }
  const auto inference_end = std::chrono::steady_clock::now();

  std::vector<HiVTModePrediction> predictions;
  if (!DecodeHiVTSceneOutput(input, output, selected_targets, &predictions)) {
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
  if (FLAGS_prediction_enable_profiling) {
    const auto milliseconds = [](const auto& start, const auto& end) {
      return std::chrono::duration<double, std::milli>(end - start).count();
    };
    AINFO << "HiVT profiling target_count=" << selected_targets.size()
          << " actor_count=" << input.actor_count
          << " lane_vector_count=" << input.lane_vector_count
          << " lane_actor_edge_count=" << input.lane_actor_edge_count
          << " selection_ms="
          << milliseconds(profile_start, actor_selection_end)
          << " feature_build_ms="
          << milliseconds(actor_selection_end, feature_build_end)
          << " tensorrt_run_ms="
          << milliseconds(feature_build_end, inference_end)
          << " input_prepare_ms=" << output.input_prepare_ms
          << " input_pre_transfer_ms=" << output.input_pre_transfer_ms
          << " input_pre_transfer_host_cpu_ms="
          << output.input_pre_transfer_host_cpu_ms
          << " input_pre_transfer_voluntary_context_switches="
          << output.input_pre_transfer_voluntary_context_switches
          << " input_pre_transfer_involuntary_context_switches="
          << output.input_pre_transfer_involuntary_context_switches
          << " input_device_select_ms=" << output.input_device_select_ms
          << " input_validation_ms=" << output.input_validation_ms
          << " input_shape_setup_ms=" << output.input_shape_setup_ms
          << " input_pack_ms=" << output.input_pack_ms
          << " input_transfer_profile_probe_ms="
          << output.input_transfer_profile_probe_ms
          << " input_transfer_submit_ms=" << output.input_transfer_submit_ms
          << " input_transfer_host_cpu_ms="
          << output.input_transfer_host_cpu_ms
          << " input_transfer_voluntary_context_switches="
          << output.input_transfer_voluntary_context_switches
          << " input_transfer_involuntary_context_switches="
          << output.input_transfer_involuntary_context_switches
          << " input_copy_submit_ms=" << output.input_copy_submit_ms
          << " tensor_binding_setup_ms=" << output.tensor_binding_setup_ms
          << " enqueue_cpu_ms=" << output.enqueue_cpu_ms
          << " enqueue_host_cpu_ms=" << output.enqueue_host_cpu_ms
          << " enqueue_voluntary_context_switches="
          << output.enqueue_voluntary_context_switches
          << " enqueue_involuntary_context_switches="
          << output.enqueue_involuntary_context_switches
          << " completion_wait_ms=" << output.completion_wait_ms
          << " gpu_inference_ms=" << output.gpu_inference_ms
          << " decode_and_apply_ms="
          << milliseconds(inference_end, std::chrono::steady_clock::now())
          << " total_ms="
          << milliseconds(profile_start, std::chrono::steady_clock::now());
  }
  return true;
}

}  // namespace prediction
}  // namespace apollo
