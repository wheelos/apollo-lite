// Copyright 2026 The Apollo Authors. All Rights Reserved.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "modules/perception/common/graph/hungarian_optimizer.h"
#include "modules/traffic_light/interface/stage.h"

namespace apollo {
namespace traffic_light {

class TrackerStage : public BaseStage {
 public:
  explicit TrackerStage(const TrackerOptions& options)
      : BaseStage(false), options_(options) {}

  std::string Name() const override { return "TrackerStage"; }

  bool Predict(PipelineContext* context) {
    if (context == nullptr) {
      return false;
    }
    std::lock_guard<std::mutex> lock(context->rw_mutex);
    PredictLocked(context);
    return true;
  }

  bool Process(PipelineContext* context) override {
    if (context == nullptr) {
      return false;
    }
    std::lock_guard<std::mutex> lock(context->rw_mutex);
    PredictLocked(context);
    context->metrics.tracker = StageRuntimeMetrics();
    context->tracked_lights.clear();
    std::map<std::string, std::vector<size_t>> detections;
    for (size_t i = 0; i < context->visual_lights.size(); ++i) {
      auto& light = context->visual_lights[i];
      light.track_id = 0;
      if (light.camera_name.empty()) {
        light.camera_name = context->primary_camera_name;
      }
      if (!ValidObservation(light) ||
          active_cameras_.count(light.camera_name) == 0) {
        context->AppendDegradeReason("tracker invalid visual observation");
        ++context->metrics.tracker.rejected_count;
        continue;
      }
      detections[light.camera_name].push_back(i);
    }
    for (const auto& name : active_cameras_) {
      CameraPool& pool = pools_[name];
      if (!pool.updated) {
        Update(detections[name], context, &pool);
        pool.updated = true;
      }
      for (const auto& item : pool.tracks) {
        if (item.second.output.confirmed) {
          context->tracked_lights.push_back(item.second.output);
        }
      }
    }
    auto& metrics = context->metrics.tracker;
    metrics.input_count = static_cast<int>(context->visual_lights.size());
    metrics.output_count = static_cast<int>(context->tracked_lights.size());
    for (const auto& track : context->tracked_lights) {
      metrics.max_confidence =
          std::max(metrics.max_confidence, track.stabilized_confidence);
      metrics.avg_confidence += track.stabilized_confidence;
    }
    if (!context->tracked_lights.empty()) {
      metrics.avg_confidence /= context->tracked_lights.size();
    }
    SyncRuntime(context);
    return true;
  }

 private:
  // Image-space constant-velocity prediction; no global pose is required.
  struct Axis {
    double position = 0.0;
    double velocity = 0.0;
    double pp = 4.0;
    double pv = 0.0;
    double vv = 400.0;

    void Predict(double dt) {
      position += velocity * dt;
      constexpr double kAccelerationVariance = 100.0;
      pp += 2.0 * dt * pv + dt * dt * vv +
            kAccelerationVariance * dt * dt * dt * dt / 4.0;
      pv += dt * vv + kAccelerationVariance * dt * dt * dt / 2.0;
      vv += kAccelerationVariance * dt * dt;
    }

    void Observe(double measurement, double variance) {
      const double innovation = measurement - position;
      const double denominator = pp + variance;
      const double kp = pp / denominator;
      const double kv = pv / denominator;
      position += kp * innovation;
      velocity += kv * innovation;
      const double old_pv = pv;
      pp = std::max(0.0, pp * (1.0 - kp));
      pv *= 1.0 - kp;
      vv = std::max(0.0, vv - kv * old_pv);
    }
  };

  struct Track {
    TrackedLight output;
    Axis x;
    Axis y;
    Axis width;
    Axis height;
    bool has_state = false;
  };

  struct CameraPool {
    std::map<int, Track> tracks;
    double timestamp = -1.0;
    uint64_t frame_id = 0;
    int rows = 0;
    int cols = 0;
    bool updated = false;
  };

  static bool ValidBox(const Rect2f& box) {
    return std::isfinite(box.x) && std::isfinite(box.y) &&
           std::isfinite(box.width) && std::isfinite(box.height) &&
           box.IsValid() && std::isfinite(box.x + box.width) &&
           std::isfinite(box.y + box.height) && std::isfinite(box.Area());
  }

  static bool ValidObservation(const VisualLight& light) {
    const int color = static_cast<int>(light.color);
    return ValidBox(light.bbox) && std::isfinite(light.confidence) &&
           std::isfinite(light.existence_confidence) &&
           std::isfinite(light.state_confidence) && light.confidence >= 0.0f &&
           light.confidence <= 1.0f && light.existence_confidence >= 0.0f &&
           light.existence_confidence <= 1.0f &&
           light.state_confidence >= 0.0f && light.state_confidence <= 1.0f &&
           color >= 0 && color <= static_cast<int>(LightColor::BLACK);
  }

  static void SetPredictedBox(Track* track) {
    auto& box = track->output.current_state.visual_light.bbox;
    box.width = static_cast<float>(std::max(1.0, track->width.position));
    box.height = static_cast<float>(std::max(1.0, track->height.position));
    box.x = static_cast<float>(track->x.position - box.width / 2.0);
    box.y = static_cast<float>(track->y.position - box.height / 2.0);
  }

  void ExpireState(double timestamp, Track* track) const {
    if (!track->has_state || timestamp - track->output.last_state_observed_sec >
                                 options_.state_validity_sec) {
      track->has_state = false;
      track->output.color_belief.fill(0.0f);
      track->output.stabilized_color = LightColor::UNKNOWN;
      track->output.stabilized_confidence = 0.0f;
      track->output.blink = false;
    }
    auto& visual = track->output.current_state.visual_light;
    visual.color = track->output.stabilized_color;
    visual.state_confidence = track->output.stabilized_confidence;
    visual.confidence = track->output.stabilized_confidence;
    visual.blink = track->output.blink;
  }

  void PredictCamera(const std::string& name, const Image& image,
                     uint64_t timestamp_ns, PipelineContext* context) {
    CameraPool& pool = pools_[name];
    const double timestamp = static_cast<double>(timestamp_ns) * 1e-9;
    if (image.rows <= 0 || image.cols <= 0) {
      pool = CameraPool();
      context->AppendDegradeReason("tracker invalid image dimensions");
      return;
    }
    const bool reset =
        pool.timestamp >= 0.0 &&
        (pool.rows != image.rows || pool.cols != image.cols ||
         timestamp < pool.timestamp ||
         (timestamp == pool.timestamp && pool.frame_id != context->frame_id) ||
         timestamp - pool.timestamp > options_.max_lost_sec);
    if (reset) {
      pool = CameraPool();
      context->AppendDegradeReason("tracker camera geometry or time reset");
    }
    active_cameras_.insert(name);
    const double dt = pool.timestamp >= 0.0 ? timestamp - pool.timestamp : 0.0;
    if (pool.timestamp != timestamp) {
      pool.updated = false;
    }
    pool.timestamp = timestamp;
    pool.frame_id = context->frame_id;
    pool.rows = image.rows;
    pool.cols = image.cols;
    for (auto it = pool.tracks.begin(); it != pool.tracks.end();) {
      Track& track = it->second;
      if (timestamp - track.output.last_visible_timestamp_sec >
          options_.max_lost_sec) {
        it = pool.tracks.erase(it);
        continue;
      }
      track.x.Predict(dt);
      track.y.Predict(dt);
      track.width.Predict(dt);
      track.height.Predict(dt);
      const double decay = options_.state_validity_sec > 0.0
                               ? std::exp(-dt / options_.state_validity_sec)
                               : 0.0;
      for (auto& belief : track.output.color_belief) {
        belief *= decay;
      }
      track.output.stabilized_confidence *= decay;
      SetPredictedBox(&track);
      ExpireState(timestamp, &track);
      if (!ValidBox(track.output.current_state.visual_light.bbox) ||
          !std::isfinite(track.x.pp) || !std::isfinite(track.y.pp)) {
        context->AppendDegradeReason("tracker nonfinite prediction");
        it = pool.tracks.erase(it);
        continue;
      }
      ++it;
    }
  }

  void PredictLocked(PipelineContext* context) {
    active_cameras_.clear();
    std::set<std::string> seen;
    for (const auto& frame : context->camera_frames) {
      // Other entries are asynchronous cached images, not detector frames.
      if (frame.camera_name != context->primary_camera_name) {
        continue;
      }
      if (!seen.insert(frame.camera_name).second) {
        context->AppendDegradeReason("tracker duplicate camera frame");
        continue;
      }
      if (!frame.is_working) {
        pools_.erase(frame.camera_name);
        context->AppendDegradeReason("tracker camera unavailable");
        continue;
      }
      PredictCamera(
          frame.camera_name, frame.image,
          frame.timestamp_ns != 0 ? frame.timestamp_ns : context->timestamp,
          context);
    }
    if (context->camera_frames.empty()) {
      PredictCamera(context->primary_camera_name, context->image_wide,
                    context->timestamp, context);
    }
    // Absent cameras do not receive synthetic image observations, but their
    // memory must still age out when other cameras keep producing frames.
    const double timestamp = static_cast<double>(context->timestamp) * 1e-9;
    for (auto& camera : pools_) {
      if (active_cameras_.count(camera.first) != 0) {
        continue;
      }
      auto& tracks = camera.second.tracks;
      for (auto it = tracks.begin(); it != tracks.end();) {
        if (timestamp - it->second.output.last_visible_timestamp_sec >
            options_.max_lost_sec) {
          it = tracks.erase(it);
        } else {
          ExpireState(timestamp, &it->second);
          ++it;
        }
      }
    }
    context->track_predictions.clear();
    for (const auto& name : active_cameras_) {
      const auto& pool = pools_.at(name);
      for (const auto& item : pool.tracks) {
        const auto& track = item.second;
        TrackPrediction prediction;
        prediction.track_id = item.first;
        prediction.camera_name = name;
        prediction.bbox = track.output.current_state.visual_light.bbox;
        prediction.position_stddev_px = static_cast<float>(
            std::sqrt(std::max(track.x.pp, track.y.pp)) +
            0.5 * std::sqrt(std::max(track.width.pp, track.height.pp)));
        prediction.timestamp_sec = pool.timestamp;
        prediction.last_observed_sec = track.output.last_visible_timestamp_sec;
        context->track_predictions.push_back(prediction);
      }
    }
    SyncRuntime(context);
  }

  static float ComputeIou(const Rect2f& lhs, const Rect2f& rhs) {
    const float x1 = std::max(lhs.x, rhs.x);
    const float y1 = std::max(lhs.y, rhs.y);
    const float x2 = std::min(lhs.x + lhs.width, rhs.x + rhs.width);
    const float y2 = std::min(lhs.y + lhs.height, rhs.y + rhs.height);
    const float intersection =
        std::max(0.0f, x2 - x1) * std::max(0.0f, y2 - y1);
    const float area = lhs.Area() + rhs.Area() - intersection;
    return area > 0.0f ? intersection / area : 0.0f;
  }

  float AssociationScore(const Track& track, const VisualLight& light) const {
    const auto& box = track.output.current_state.visual_light.bbox;
    const float iou = ComputeIou(box, light.bbox);
    if (iou >= options_.min_iou_match) {
      return 1.0f + iou;
    }
    const double dx = light.bbox.x + light.bbox.width / 2.0 - track.x.position;
    const double dy = light.bbox.y + light.bbox.height / 2.0 - track.y.position;
    const double variance = MeasurementVariance(light.bbox);
    const double distance =
        dx * dx / (track.x.pp + variance) + dy * dy / (track.y.pp + variance);
    // The extent gate prevents large uncertainty from merging distant lights.
    if (distance <= 9.0 && std::abs(dx) <= 2.0 * box.width &&
        std::abs(dy) <= 2.0 * box.height) {
      return static_cast<float>(1.0 / (1.0 + distance));
    }
    return -1.0f;
  }

  static double MeasurementVariance(const Rect2f& box) {
    const double sigma = std::max(1.0f, std::min(box.width, box.height) * 0.1f);
    return sigma * sigma;
  }

  void Observe(VisualLight* light, double timestamp, bool allow_state,
               Track* track) {
    const double variance = MeasurementVariance(light->bbox);
    track->x.Observe(light->bbox.x + light->bbox.width / 2.0, variance);
    track->y.Observe(light->bbox.y + light->bbox.height / 2.0, variance);
    track->width.Observe(light->bbox.width, variance);
    track->height.Observe(light->bbox.height, variance);
    auto& output = track->output;
    light->track_id = output.track_id;
    // Binder annotations are per-frame, never tracker identity or memory.
    output.current_state = BoundLight();
    output.current_state.visual_light = *light;
    SetPredictedBox(track);
    ++output.visible_count;
    output.lost_frames = 0;
    output.confirmed =
        output.visible_count >= options_.min_confirmed_visible_count;
    output.last_visible_timestamp_sec = timestamp;
    const size_t color = static_cast<size_t>(light->color);
    if (allow_state && color > 0 && color < output.color_belief.size() &&
        light->state_confidence >= options_.high_score_threshold) {
      output.color_belief[color] +=
          light->state_confidence * options_.measurement_gain;
      output.last_state_observed_sec = timestamp;
      track->has_state = true;
      const auto best = std::max_element(output.color_belief.begin(),
                                         output.color_belief.end());
      float other = 0.0f;
      for (float evidence : output.color_belief) {
        other += evidence;
      }
      other -= *best;
      output.stabilized_color =
          static_cast<LightColor>(best - output.color_belief.begin());
      // Evidence strength is retained: a single observation is never unity.
      output.stabilized_confidence = std::min(0.99f, *best / (1.0f + other));
      output.blink = light->blink;
    }
    ExpireState(timestamp, track);
  }

  void Match(const std::vector<size_t>& indices, bool low,
             PipelineContext* context, CameraPool* pool,
             std::set<int>* used_tracks, std::set<size_t>* used_detections) {
    std::vector<int> tracks;
    for (const auto& item : pool->tracks) {
      if (used_tracks->count(item.first) != 0 ||
          (low && !item.second.output.confirmed)) {
        continue;
      }
      tracks.push_back(item.first);
    }
    if (tracks.empty() || indices.empty()) {
      return;
    }
    // Dummy columns permit unmatched tracks. Global assignment avoids the
    // greedy edge choice starving another nearby light of its only match.
    const size_t columns = indices.size() + tracks.size();
    apollo::perception::common::HungarianOptimizer<float> matcher(
        static_cast<int>(std::max(tracks.size(), columns)));
    auto* costs = matcher.costs();
    costs->Resize(tracks.size(), columns);
    for (size_t row = 0; row < tracks.size(); ++row) {
      for (size_t col = 0; col < columns; ++col) {
        (*costs)(row, col) =
            col < indices.size()
                ? AssociationScore(pool->tracks.at(tracks[row]),
                                   context->visual_lights[indices[col]])
                : 0.0f;
      }
    }
    std::vector<std::pair<size_t, size_t>> assignments;
    matcher.Maximize(&assignments);
    for (const auto& assignment : assignments) {
      if (assignment.second >= indices.size()) {
        continue;
      }
      const int id = tracks[assignment.first];
      const size_t index = indices[assignment.second];
      if (AssociationScore(pool->tracks.at(id), context->visual_lights[index]) <
          0.0f) {
        continue;
      }
      Observe(&context->visual_lights[index], pool->timestamp, !low,
              &pool->tracks.at(id));
      used_tracks->insert(id);
      used_detections->insert(index);
    }
  }

  void Update(const std::vector<size_t>& indices, PipelineContext* context,
              CameraPool* pool) {
    std::vector<size_t> high;
    std::vector<size_t> low;
    for (size_t index : indices) {
      const float score = context->visual_lights[index].existence_confidence;
      if (score >= options_.high_score_threshold) {
        high.push_back(index);
      } else if (score >= options_.low_score_threshold) {
        low.push_back(index);
      }
    }
    const auto strongest_first = [&](size_t a, size_t b) {
      return context->visual_lights[a].existence_confidence >
             context->visual_lights[b].existence_confidence;
    };
    for (auto* candidates : {&high, &low}) {
      if (candidates->size() > options_.max_tracks_per_camera) {
        std::stable_sort(candidates->begin(), candidates->end(),
                         strongest_first);
        candidates->resize(options_.max_tracks_per_camera);
        context->AppendDegradeReason("tracker observation budget exceeded");
      }
    }
    for (auto& item : pool->tracks) {
      ++item.second.output.age;
      if (context->status.neural_detector_ran) {
        ++item.second.output.lost_frames;
      }
    }
    std::set<int> used_tracks;
    std::set<size_t> used_detections;
    // ByteTrack's high/low association policy, with measurement-time dynamics.
    Match(high, false, context, pool, &used_tracks, &used_detections);
    Match(low, true, context, pool, &used_tracks, &used_detections);
    for (size_t index : high) {
      if (used_detections.count(index) != 0) {
        continue;
      }
      if (pool->tracks.size() >= options_.max_tracks_per_camera) {
        context->AppendDegradeReason("tracker track budget exceeded");
        ++context->metrics.tracker.rejected_count;
        continue;
      }
      const auto& box = context->visual_lights[index].bbox;
      Track track;
      track.output.track_id = next_track_id_++;
      track.output.age = 1;
      track.output.color_belief.fill(0.0f);
      track.x.position = box.x + box.width / 2.0;
      track.y.position = box.y + box.height / 2.0;
      track.width.position = box.width;
      track.height.position = box.height;
      track.x.pp = MeasurementVariance(box);
      track.y.pp = track.x.pp;
      track.width.pp = track.x.pp;
      track.height.pp = track.x.pp;
      Observe(&context->visual_lights[index], pool->timestamp, true, &track);
      pool->tracks.emplace(track.output.track_id, track);
    }
    if (!context->status.neural_detector_ran) {
      context->AppendDegradeReason("tracker detector unavailable");
    }
  }

  void SyncRuntime(PipelineContext* context) const {
    if (context->runtime_state == nullptr) {
      return;
    }
    auto& runtime = *context->runtime_state;
    runtime.tracked_memory.clear();
    for (const auto& camera : pools_) {
      for (const auto& item : camera.second.tracks) {
        runtime.tracked_memory.push_back(item.second.output);
      }
    }
    runtime.last_frame_id = context->frame_id;
    runtime.last_processed_ts_sec =
        static_cast<double>(context->timestamp) * 1e-9;
  }

  TrackerOptions options_;
  int next_track_id_ = 1;
  std::map<std::string, CameraPool> pools_;
  std::set<std::string> active_cameras_;
};

}  // namespace traffic_light
}  // namespace apollo
