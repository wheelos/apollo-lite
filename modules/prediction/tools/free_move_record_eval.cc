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

// Offline industrial evaluator for the production free-move predictor.
// This tool reads perception tracks from Cyber record files, runs the current
// production FreeMovePredictor directly, and compares it against the legacy
// CA baseline with layered motion-mode and runtime metrics.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "cyber/cyber.h"
#include "cyber/record/record_message.h"
#include "cyber/record/record_reader.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/math/math_utils.h"
#include "modules/common_msgs/perception_msgs/perception_obstacle.pb.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/prediction/container/obstacles/obstacles_container.h"
#include "modules/prediction/predictor/free_move/free_move_predictor.h"

namespace apollo {
namespace prediction {
namespace free_move_record_eval {

using apollo::common::TrajectoryPoint;
using apollo::cyber::record::RecordMessage;
using apollo::cyber::record::RecordReader;
using apollo::perception::PerceptionObstacle;
using apollo::perception::PerceptionObstacles;

constexpr char kPerceptionTopic[] = "/apollo/perception/obstacles";
constexpr char kDefaultRecordPath[] = "/mnt/synology/apollo/demo_3.5.record";
constexpr double kHistoryWindowSec = 1.0;
constexpr double kMinReliableHistorySec = 0.2;
constexpr double kStraightYawRateThreshold = 0.05;
constexpr double kGentleTurnYawRateThreshold = 0.2;
constexpr double kLegacyMinAcc = -4.0;
constexpr double kLegacyMaxAcc = 4.0;
constexpr double kSampleSpacingSec = 0.5;
constexpr double kMaxInterpGapSec = 0.25;
constexpr double kMinEvalSpeed = 1.0;
constexpr double kHorizonSec = 2.0;
constexpr double kPeriodSec = 0.1;

struct TrackPoint {
  double timestamp = 0.0;
  double x = 0.0;
  double y = 0.0;
  double vx = 0.0;
  double vy = 0.0;
  double ax = 0.0;
  double ay = 0.0;
  double theta = 0.0;
  PerceptionObstacle::Type obstacle_type = PerceptionObstacle::UNKNOWN;
};

struct PredPoint {
  double relative_time = 0.0;
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
};

struct SampleMetrics {
  double ade_1s = 0.0;
  double fde_1s = 0.0;
  double ade_2s = 0.0;
  double fde_2s = 0.0;
  double lateral_std = 0.0;
  double heading_std = 0.0;
  double pred_speed_1s = 0.0;
  double actual_speed_1s = 0.0;
  double pred_speed_2s = 0.0;
  double actual_speed_2s = 0.0;
};

struct MetricSummary {
  size_t samples = 0;
  double ade_1s_mean = 0.0;
  double fde_1s_mean = 0.0;
  double ade_2s_mean = 0.0;
  double fde_2s_mean = 0.0;
  double lateral_std_mean = 0.0;
  double lateral_std_p95 = 0.0;
  double heading_std_mean = 0.0;
  double pred_speed_1s_mean = 0.0;
  double actual_speed_1s_mean = 0.0;
  double pred_speed_2s_mean = 0.0;
  double actual_speed_2s_mean = 0.0;
};

struct CohortSummary {
  MetricSummary legacy;
  MetricSummary current;
  size_t unique_obstacles = 0;
  bool gate_pass = false;
};

enum class MotionMode { kLinear, kGentleTurn, kSharpTurn };

using TrackMap = std::unordered_map<int, std::vector<TrackPoint>>;

bool IsEvalObstacleType(const PerceptionObstacle::Type type) {
  return type == PerceptionObstacle::VEHICLE ||
         type == PerceptionObstacle::BICYCLE ||
         type == PerceptionObstacle::PEDESTRIAN;
}

double NormalizeAngle(const double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

double AngleDiff(const double from, const double to) {
  return NormalizeAngle(to - from);
}

double Mean(const std::vector<double>& values) {
  if (values.empty()) {
    return 0.0;
  }
  double sum = 0.0;
  for (double value : values) {
    sum += value;
  }
  return sum / static_cast<double>(values.size());
}

double StdDev(const std::vector<double>& values) {
  if (values.size() <= 1) {
    return 0.0;
  }
  const double mean = Mean(values);
  double accum = 0.0;
  for (double value : values) {
    const double delta = value - mean;
    accum += delta * delta;
  }
  return std::sqrt(accum / static_cast<double>(values.size()));
}

double Percentile(std::vector<double> values, const double ratio) {
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const double index = ratio * static_cast<double>(values.size() - 1);
  const auto lower = static_cast<size_t>(std::floor(index));
  const auto upper = static_cast<size_t>(std::ceil(index));
  if (lower == upper) {
    return values[lower];
  }
  const double weight = index - static_cast<double>(lower);
  return values[lower] * (1.0 - weight) + values[upper] * weight;
}

double PointDistance(const double x0, const double y0, const double x1,
                     const double y1) {
  return std::hypot(x1 - x0, y1 - y0);
}

double EstimateYawRateFromHistory(
    const std::vector<const TrackPoint*>& history) {
  if (history.size() < 3) {
    return 0.0;
  }

  bool initialized = false;
  double first_heading = 0.0;
  double last_heading = 0.0;
  double first_time = 0.0;
  double last_time = 0.0;
  for (size_t i = 1; i < history.size(); ++i) {
    const auto& prev_point = *history[i - 1];
    const auto& curr_point = *history[i];
    const double delta_x = curr_point.x - prev_point.x;
    const double delta_y = curr_point.y - prev_point.y;
    const double delta_t = curr_point.timestamp - prev_point.timestamp;
    if (delta_t <= 1e-3 || std::hypot(delta_x, delta_y) <= 1e-3) {
      continue;
    }

    const double segment_heading = std::atan2(delta_y, delta_x);
    if (!initialized) {
      first_heading = segment_heading;
      first_time = curr_point.timestamp;
      initialized = true;
    }
    last_heading = segment_heading;
    last_time = curr_point.timestamp;
  }
  if (!initialized || last_time - first_time <= 1e-3) {
    return 0.0;
  }
  return AngleDiff(first_heading, last_heading) / (last_time - first_time);
}

std::vector<const TrackPoint*> CollectRecentHistory(
    const std::vector<TrackPoint>& track, const double frame_time) {
  std::vector<const TrackPoint*> history;
  for (const auto& point : track) {
    if (point.timestamp > frame_time + 1e-6) {
      break;
    }
    if (frame_time - point.timestamp > kHistoryWindowSec) {
      continue;
    }
    history.push_back(&point);
  }
  return history;
}

double ComputeHistoryTime(const std::vector<const TrackPoint*>& history) {
  if (history.size() < 2) {
    return 0.0;
  }
  return std::max(0.0,
                  history.back()->timestamp - history.front()->timestamp);
}

MotionMode ClassifyMotionMode(const double abs_yaw_rate) {
  if (abs_yaw_rate < kStraightYawRateThreshold) {
    return MotionMode::kLinear;
  }
  if (abs_yaw_rate < kGentleTurnYawRateThreshold) {
    return MotionMode::kGentleTurn;
  }
  return MotionMode::kSharpTurn;
}

std::string MotionModeName(const MotionMode mode) {
  switch (mode) {
    case MotionMode::kLinear:
      return "linear_motion";
    case MotionMode::kGentleTurn:
      return "gentle_turn";
    case MotionMode::kSharpTurn:
      return "sharp_turn";
  }
  return "unknown";
}

bool InterpolateTrackPosition(const std::vector<TrackPoint>& track,
                              const double target_time, double* x, double* y) {
  auto it = std::lower_bound(
      track.begin(), track.end(), target_time,
      [](const TrackPoint& point, const double time) {
        return point.timestamp < time;
      });
  if (it != track.end() && std::fabs(it->timestamp - target_time) <= 1e-6) {
    *x = it->x;
    *y = it->y;
    return true;
  }
  if (it == track.begin() || it == track.end()) {
    return false;
  }
  const auto& right = *it;
  const auto& left = *(it - 1);
  const double gap = right.timestamp - left.timestamp;
  if (gap > kMaxInterpGapSec || gap <= 1e-6) {
    return false;
  }
  const double ratio = (target_time - left.timestamp) / gap;
  *x = left.x + (right.x - left.x) * ratio;
  *y = left.y + (right.y - left.y) * ratio;
  return true;
}

double ComputePredAverageSpeed(const std::vector<PredPoint>& trajectory,
                               const double target_time) {
  if (target_time <= 1e-6) {
    return 0.0;
  }
  double total_dist = 0.0;
  for (size_t i = 1; i < trajectory.size(); ++i) {
    const auto& left = trajectory[i - 1];
    const auto& right = trajectory[i];
    if (left.relative_time >= target_time - 1e-9) {
      break;
    }
    const double seg_end_t = std::min(right.relative_time, target_time);
    const double gap = right.relative_time - left.relative_time;
    if (gap <= 1e-6) {
      continue;
    }
    const double ratio = (seg_end_t - left.relative_time) / gap;
    const double end_x = left.x + (right.x - left.x) * ratio;
    const double end_y = left.y + (right.y - left.y) * ratio;
    total_dist += PointDistance(left.x, left.y, end_x, end_y);
    if (seg_end_t >= target_time - 1e-9) {
      break;
    }
  }
  return total_dist / target_time;
}

bool ComputeActualAverageSpeed(const std::vector<TrackPoint>& track,
                               const double frame_time,
                               const double target_time, double* out_speed) {
  if (target_time <= 1e-6) {
    *out_speed = 0.0;
    return true;
  }
  std::vector<double> times;
  times.push_back(frame_time);
  for (const auto& point : track) {
    if (point.timestamp > frame_time + 1e-9 &&
        point.timestamp < frame_time + target_time - 1e-9) {
      times.push_back(point.timestamp);
    }
  }
  times.push_back(frame_time + target_time);

  double total_dist = 0.0;
  double prev_x = 0.0;
  double prev_y = 0.0;
  bool initialized = false;
  for (double time : times) {
    double x = 0.0;
    double y = 0.0;
    if (!InterpolateTrackPosition(track, time, &x, &y)) {
      return false;
    }
    if (initialized) {
      total_dist += PointDistance(prev_x, prev_y, x, y);
    }
    prev_x = x;
    prev_y = y;
    initialized = true;
  }
  *out_speed = total_dist / target_time;
  return true;
}

double ComputeLateralStd(const std::vector<PredPoint>& points) {
  if (points.size() < 2) {
    return 0.0;
  }
  const double x0 = points.front().x;
  const double y0 = points.front().y;
  const double theta0 = points.front().theta;
  std::vector<double> offsets;
  offsets.reserve(points.size());
  for (const auto& point : points) {
    const double dx = point.x - x0;
    const double dy = point.y - y0;
    offsets.push_back(-dx * std::sin(theta0) + dy * std::cos(theta0));
  }
  return StdDev(offsets);
}

double ComputeHeadingStd(const std::vector<PredPoint>& points) {
  if (points.size() < 2) {
    return 0.0;
  }
  std::vector<double> changes;
  changes.reserve(points.size() - 1);
  for (size_t i = 1; i < points.size(); ++i) {
    changes.push_back(AngleDiff(points[i - 1].theta, points[i].theta));
  }
  return StdDev(changes);
}

bool SamplePredictionXY(const std::vector<PredPoint>& points,
                        const double target_time, double* x, double* y) {
  if (points.empty()) {
    return false;
  }
  if (target_time < points.front().relative_time - 1e-6 ||
      target_time > points.back().relative_time + 1e-6) {
    return false;
  }
  for (size_t i = 1; i < points.size(); ++i) {
    if (target_time <= points[i].relative_time + 1e-6) {
      const auto& left = points[i - 1];
      const auto& right = points[i];
      const double gap = right.relative_time - left.relative_time;
      if (gap <= 1e-6) {
        *x = right.x;
        *y = right.y;
        return true;
      }
      const double ratio = (target_time - left.relative_time) / gap;
      *x = left.x + (right.x - left.x) * ratio;
      *y = left.y + (right.y - left.y) * ratio;
      return true;
    }
  }
  *x = points.back().x;
  *y = points.back().y;
  return true;
}

std::vector<PredPoint> BuildLegacyTrajectory(const TrackPoint& current,
                                             const double total_time,
                                             const double period) {
  const double x = current.x;
  const double y = current.y;
  const double vx = current.vx;
  const double vy = current.vy;
  const double ax = common::math::Clamp(current.ax, kLegacyMinAcc, kLegacyMaxAcc);
  const double ay = common::math::Clamp(current.ay, kLegacyMinAcc, kLegacyMaxAcc);

  const size_t num_points = static_cast<size_t>(total_time / period);
  std::vector<PredPoint> points;
  points.reserve(num_points + 1);
  const double initial_theta =
      std::hypot(vx, vy) > 1e-3 ? std::atan2(vy, vx) : current.theta;
  points.push_back({0.0, x, y, initial_theta});
  for (size_t step = 1; step <= num_points; ++step) {
    const double t = static_cast<double>(step) * period;
    const double px = x + vx * t + 0.5 * ax * t * t;
    const double py = y + vy * t + 0.5 * ay * t * t;
    const double speed_x = vx + ax * t;
    const double speed_y = vy + ay * t;
    const double theta =
        std::hypot(speed_x, speed_y) > 1e-3
            ? std::atan2(speed_y, speed_x)
            : points.back().theta;
    points.push_back({t, px, py, theta});
  }
  return points;
}

std::vector<PredPoint> BuildCurrentTrajectory(Obstacle* obstacle,
                                              FreeMovePredictor* predictor,
                                              ObstaclesContainer* container,
                                              double* predict_ms) {
  std::vector<PredPoint> points;
  if (obstacle == nullptr || predictor == nullptr || container == nullptr) {
    return points;
  }
  obstacle->mutable_latest_feature()->clear_predicted_trajectory();
  const auto start = std::chrono::steady_clock::now();
  const bool ok = predictor->Predict(nullptr, obstacle, container);
  const auto end = std::chrono::steady_clock::now();
  *predict_ms = std::chrono::duration<double, std::milli>(end - start).count();
  if (!ok || obstacle->latest_feature().predicted_trajectory_size() == 0) {
    return points;
  }
  const auto& trajectory = obstacle->latest_feature().predicted_trajectory(0);
  points.reserve(static_cast<size_t>(trajectory.trajectory_point_size()));
  for (const auto& point : trajectory.trajectory_point()) {
    points.push_back({point.relative_time(), point.path_point().x(),
                      point.path_point().y(), point.path_point().theta()});
  }
  return points;
}

bool ComputeSampleMetrics(const std::vector<PredPoint>& trajectory,
                          const std::vector<TrackPoint>& track,
                          const double frame_time, SampleMetrics* metrics) {
  static const std::vector<double> horizons = {0.5, 1.0, 1.5, 2.0};
  std::vector<std::pair<double, double>> actual_positions;
  std::vector<std::pair<double, double>> predicted_positions;
  actual_positions.reserve(horizons.size());
  predicted_positions.reserve(horizons.size());
  for (double horizon : horizons) {
    double actual_x = 0.0;
    double actual_y = 0.0;
    double pred_x = 0.0;
    double pred_y = 0.0;
    if (!InterpolateTrackPosition(track, frame_time + horizon, &actual_x,
                                  &actual_y) ||
        !SamplePredictionXY(trajectory, horizon, &pred_x, &pred_y)) {
      return false;
    }
    actual_positions.emplace_back(actual_x, actual_y);
    predicted_positions.emplace_back(pred_x, pred_y);
  }

  metrics->ade_1s =
      (PointDistance(predicted_positions[0].first, predicted_positions[0].second,
                     actual_positions[0].first, actual_positions[0].second) +
       PointDistance(predicted_positions[1].first, predicted_positions[1].second,
                     actual_positions[1].first, actual_positions[1].second)) /
      2.0;
  metrics->fde_1s = PointDistance(predicted_positions[1].first,
                                  predicted_positions[1].second,
                                  actual_positions[1].first,
                                  actual_positions[1].second);

  double ade_2s_sum = 0.0;
  for (size_t i = 0; i < horizons.size(); ++i) {
    ade_2s_sum += PointDistance(predicted_positions[i].first,
                                predicted_positions[i].second,
                                actual_positions[i].first,
                                actual_positions[i].second);
  }
  metrics->ade_2s = ade_2s_sum / static_cast<double>(horizons.size());
  metrics->fde_2s = PointDistance(predicted_positions.back().first,
                                  predicted_positions.back().second,
                                  actual_positions.back().first,
                                  actual_positions.back().second);

  metrics->lateral_std = ComputeLateralStd(trajectory);
  metrics->heading_std = ComputeHeadingStd(trajectory);
  metrics->pred_speed_1s = ComputePredAverageSpeed(trajectory, 1.0);
  metrics->pred_speed_2s = ComputePredAverageSpeed(trajectory, 2.0);
  if (!ComputeActualAverageSpeed(track, frame_time, 1.0,
                                 &metrics->actual_speed_1s) ||
      !ComputeActualAverageSpeed(track, frame_time, 2.0,
                                 &metrics->actual_speed_2s)) {
    return false;
  }
  return true;
}

MetricSummary SummarizeMetrics(const std::vector<SampleMetrics>& metrics) {
  MetricSummary summary;
  summary.samples = metrics.size();
  std::vector<double> ade_1s;
  std::vector<double> fde_1s;
  std::vector<double> ade_2s;
  std::vector<double> fde_2s;
  std::vector<double> lateral_std;
  std::vector<double> heading_std;
  std::vector<double> pred_speed_1s;
  std::vector<double> actual_speed_1s;
  std::vector<double> pred_speed_2s;
  std::vector<double> actual_speed_2s;
  ade_1s.reserve(metrics.size());
  fde_1s.reserve(metrics.size());
  ade_2s.reserve(metrics.size());
  fde_2s.reserve(metrics.size());
  lateral_std.reserve(metrics.size());
  heading_std.reserve(metrics.size());
  pred_speed_1s.reserve(metrics.size());
  actual_speed_1s.reserve(metrics.size());
  pred_speed_2s.reserve(metrics.size());
  actual_speed_2s.reserve(metrics.size());
  for (const auto& metric : metrics) {
    ade_1s.push_back(metric.ade_1s);
    fde_1s.push_back(metric.fde_1s);
    ade_2s.push_back(metric.ade_2s);
    fde_2s.push_back(metric.fde_2s);
    lateral_std.push_back(metric.lateral_std);
    heading_std.push_back(metric.heading_std);
    pred_speed_1s.push_back(metric.pred_speed_1s);
    actual_speed_1s.push_back(metric.actual_speed_1s);
    pred_speed_2s.push_back(metric.pred_speed_2s);
    actual_speed_2s.push_back(metric.actual_speed_2s);
  }
  summary.ade_1s_mean = Mean(ade_1s);
  summary.fde_1s_mean = Mean(fde_1s);
  summary.ade_2s_mean = Mean(ade_2s);
  summary.fde_2s_mean = Mean(fde_2s);
  summary.lateral_std_mean = Mean(lateral_std);
  summary.lateral_std_p95 = Percentile(lateral_std, 0.95);
  summary.heading_std_mean = Mean(heading_std);
  summary.pred_speed_1s_mean = Mean(pred_speed_1s);
  summary.actual_speed_1s_mean = Mean(actual_speed_1s);
  summary.pred_speed_2s_mean = Mean(pred_speed_2s);
  summary.actual_speed_2s_mean = Mean(actual_speed_2s);
  return summary;
}

bool EvaluateIndustrialGate(const MotionMode mode, const MetricSummary& legacy,
                            const MetricSummary& current,
                            const size_t unique_obstacles) {
  if (mode == MotionMode::kLinear) {
    return current.samples >= 12 && unique_obstacles >= 4 &&
           current.lateral_std_mean <= legacy.lateral_std_mean * 0.65 &&
           current.heading_std_mean <= legacy.heading_std_mean * 0.20 &&
           current.ade_1s_mean <= legacy.ade_1s_mean * 1.10 &&
           current.fde_2s_mean <= legacy.fde_2s_mean * 1.15;
  }
  if (mode == MotionMode::kGentleTurn) {
    return current.samples >= 20 && unique_obstacles >= 4 &&
           current.ade_1s_mean <= legacy.ade_1s_mean * 1.15 &&
           current.fde_2s_mean <= legacy.fde_2s_mean * 1.30;
  }
  return current.samples >= 20 && unique_obstacles >= 4 &&
         current.ade_1s_mean <= legacy.ade_1s_mean * 1.20 &&
         current.fde_2s_mean <= legacy.fde_2s_mean * 1.35;
}

void LoadRecord(const std::string& record_path,
                std::vector<PerceptionObstacles>* frames, TrackMap* tracks) {
  RecordReader reader(record_path);
  CHECK(reader.IsValid()) << "Unable to open record: " << record_path;
  RecordMessage message;
  while (reader.ReadMessage(&message)) {
    if (message.channel_name != kPerceptionTopic) {
      continue;
    }
    PerceptionObstacles obstacles;
    if (!obstacles.ParseFromString(message.content)) {
      continue;
    }
    const double frame_time =
        obstacles.has_header() ? obstacles.header().timestamp_sec() : 0.0;
    frames->push_back(obstacles);
    for (const auto& obstacle : obstacles.perception_obstacle()) {
      const double timestamp =
          obstacle.has_timestamp() ? obstacle.timestamp() : frame_time;
      TrackPoint point;
      point.timestamp = timestamp;
      point.x = obstacle.position().x();
      point.y = obstacle.position().y();
      point.vx = obstacle.velocity().x();
      point.vy = obstacle.velocity().y();
      point.ax = obstacle.has_acceleration() ? obstacle.acceleration().x() : 0.0;
      point.ay = obstacle.has_acceleration() ? obstacle.acceleration().y() : 0.0;
      point.theta = obstacle.theta();
      point.obstacle_type = obstacle.type();

      auto& track = (*tracks)[obstacle.id()];
      if (!track.empty() &&
          std::fabs(track.back().timestamp - point.timestamp) <= 1e-6) {
        track.back() = point;
      } else {
        track.push_back(point);
      }
    }
  }
}

void PrintSummaryLine(const std::string& cohort_name,
                      const MetricSummary& legacy,
                      const MetricSummary& current, const size_t unique_obstacles,
                      const bool gate_pass) {
  std::cout << cohort_name << " samples=" << current.samples
            << " unique_obstacles=" << unique_obstacles
            << " legacy_ade_1s=" << legacy.ade_1s_mean
            << " current_ade_1s=" << current.ade_1s_mean
            << " legacy_fde_2s=" << legacy.fde_2s_mean
            << " current_fde_2s=" << current.fde_2s_mean
            << " legacy_lat_std=" << legacy.lateral_std_mean
            << " current_lat_std=" << current.lateral_std_mean
            << " legacy_heading_std=" << legacy.heading_std_mean
            << " current_heading_std=" << current.heading_std_mean
            << " legacy_pred_spd_1s=" << legacy.pred_speed_1s_mean
            << " current_pred_spd_1s=" << current.pred_speed_1s_mean
            << " legacy_act_spd_1s=" << legacy.actual_speed_1s_mean
            << " current_act_spd_1s=" << current.actual_speed_1s_mean
            << " gate_pass=" << (gate_pass ? "true" : "false") << std::endl;
}

void PrintUsage(const char* program_name) {
  std::cout
      << "Usage: " << program_name << " [--record=/path/to/file.record]\n"
      << "Purpose: evaluate the production free-move predictor offline against\n"
      << "record-derived perception tracks and compare it with the legacy CA\n"
      << "baseline for release validation.\n"
      << "Outputs: aggregate, linear, gentle-turn, sharp-turn, and runtime\n"
      << "metrics using the current production Predict() path.\n";
}

}  // namespace free_move_record_eval

}  // namespace prediction
}  // namespace apollo

int main(int argc, char** argv) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "--help" || arg == "-h") {
      apollo::prediction::free_move_record_eval::PrintUsage(argv[0]);
      return 0;
    }
  }
  apollo::cyber::Init(argv[0]);
  FLAGS_map_dir = "modules/prediction/testdata";
  FLAGS_base_map_filename = "kml_map.bin";
  FLAGS_sim_map_filename = "kml_map.bin";
  FLAGS_prediction_trajectory_time_length =
      apollo::prediction::free_move_record_eval::kHorizonSec;
  FLAGS_prediction_trajectory_time_resolution =
      apollo::prediction::free_move_record_eval::kPeriodSec;
  CHECK(apollo::hdmap::HDMapUtil::ReloadMaps());

  std::string record_path =
      apollo::prediction::free_move_record_eval::kDefaultRecordPath;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg.rfind("--record=", 0) == 0) {
      record_path = arg.substr(std::string("--record=").size());
    } else {
      std::cerr << "Unknown argument: " << arg << std::endl;
      apollo::prediction::free_move_record_eval::PrintUsage(argv[0]);
      return 1;
    }
  }

  std::vector<apollo::perception::PerceptionObstacles> frames;
  apollo::prediction::free_move_record_eval::TrackMap tracks;
  apollo::prediction::free_move_record_eval::LoadRecord(record_path, &frames,
                                                        &tracks);

  apollo::prediction::FreeMovePredictor predictor;
  apollo::prediction::ObstaclesContainer container;
  std::unordered_map<int, double> last_sample_timestamp;
  std::unordered_map<apollo::prediction::free_move_record_eval::MotionMode,
                     std::vector<apollo::prediction::free_move_record_eval::SampleMetrics>>
      legacy_metrics;
  std::unordered_map<apollo::prediction::free_move_record_eval::MotionMode,
                     std::vector<apollo::prediction::free_move_record_eval::SampleMetrics>>
      current_metrics;
  std::unordered_map<apollo::prediction::free_move_record_eval::MotionMode,
                     std::set<int>>
      cohort_obstacles;
  std::vector<apollo::prediction::free_move_record_eval::SampleMetrics>
      all_legacy_metrics;
  std::vector<apollo::prediction::free_move_record_eval::SampleMetrics>
      all_current_metrics;
  std::set<int> all_obstacles;
  std::vector<double> predict_latency_ms;

  for (const auto& frame : frames) {
    container.Insert(frame);
    const double frame_time =
        frame.has_header() ? frame.header().timestamp_sec() : 0.0;
    for (const auto& obstacle : frame.perception_obstacle()) {
      if (!apollo::prediction::free_move_record_eval::IsEvalObstacleType(
              obstacle.type())) {
        continue;
      }
      const int obstacle_id = obstacle.id();
      const double timestamp =
          obstacle.has_timestamp() ? obstacle.timestamp() : frame_time;
      if (std::hypot(obstacle.velocity().x(), obstacle.velocity().y()) <
          apollo::prediction::free_move_record_eval::kMinEvalSpeed) {
        continue;
      }
      if (timestamp - last_sample_timestamp[obstacle_id] <
          apollo::prediction::free_move_record_eval::kSampleSpacingSec) {
        continue;
      }

      const auto track_it = tracks.find(obstacle_id);
      if (track_it == tracks.end()) {
        continue;
      }
      const auto history = apollo::prediction::free_move_record_eval::
          CollectRecentHistory(track_it->second, timestamp);
      if (history.size() < 3 ||
          apollo::prediction::free_move_record_eval::ComputeHistoryTime(
              history) <
              apollo::prediction::free_move_record_eval::kMinReliableHistorySec) {
        continue;
      }
      double future_x = 0.0;
      double future_y = 0.0;
      if (!apollo::prediction::free_move_record_eval::InterpolateTrackPosition(
              track_it->second,
              timestamp + apollo::prediction::free_move_record_eval::kHorizonSec,
              &future_x, &future_y)) {
        continue;
      }

      apollo::prediction::Obstacle* obstacle_ptr =
          container.GetObstacle(obstacle_id);
      if (obstacle_ptr == nullptr || obstacle_ptr->history_size() < 3) {
        continue;
      }

      const apollo::prediction::free_move_record_eval::TrackPoint current = {
          timestamp,
          obstacle.position().x(),
          obstacle.position().y(),
          obstacle.velocity().x(),
          obstacle.velocity().y(),
          obstacle.has_acceleration() ? obstacle.acceleration().x() : 0.0,
          obstacle.has_acceleration() ? obstacle.acceleration().y() : 0.0,
          obstacle.theta(),
          obstacle.type(),
      };

      const auto legacy_trajectory =
          apollo::prediction::free_move_record_eval::BuildLegacyTrajectory(
              current, apollo::prediction::free_move_record_eval::kHorizonSec,
              apollo::prediction::free_move_record_eval::kPeriodSec);
      double predict_ms = 0.0;
      const auto current_trajectory =
          apollo::prediction::free_move_record_eval::BuildCurrentTrajectory(
              obstacle_ptr, &predictor, &container, &predict_ms);
      if (current_trajectory.empty()) {
        continue;
      }

      apollo::prediction::free_move_record_eval::SampleMetrics legacy_sample;
      apollo::prediction::free_move_record_eval::SampleMetrics current_sample;
      if (!apollo::prediction::free_move_record_eval::ComputeSampleMetrics(
              legacy_trajectory, track_it->second, timestamp, &legacy_sample) ||
          !apollo::prediction::free_move_record_eval::ComputeSampleMetrics(
              current_trajectory, track_it->second, timestamp,
              &current_sample)) {
        continue;
      }

      const apollo::prediction::free_move_record_eval::MotionMode mode =
          apollo::prediction::free_move_record_eval::ClassifyMotionMode(
              std::fabs(apollo::prediction::free_move_record_eval::
                            EstimateYawRateFromHistory(history)));
      legacy_metrics[mode].push_back(legacy_sample);
      current_metrics[mode].push_back(current_sample);
      cohort_obstacles[mode].insert(obstacle_id);
      all_legacy_metrics.push_back(legacy_sample);
      all_current_metrics.push_back(current_sample);
      all_obstacles.insert(obstacle_id);
      predict_latency_ms.push_back(predict_ms);
      last_sample_timestamp[obstacle_id] = timestamp;
    }
  }

  std::cout << std::fixed << std::setprecision(6);
  std::cout << "purpose=offline_release_validation" << std::endl;
  std::cout << "comparison=legacy_ca_vs_production_free_move" << std::endl;
  std::cout << "record=" << record_path << std::endl;
  std::cout << "tracked_obstacles=" << tracks.size() << std::endl;
  std::cout << "evaluated_samples=" << all_current_metrics.size() << std::endl;
  std::cout << "unique_obstacles=" << all_obstacles.size() << std::endl;

  const auto all_legacy_summary =
      apollo::prediction::free_move_record_eval::SummarizeMetrics(
          all_legacy_metrics);
  const auto all_current_summary =
      apollo::prediction::free_move_record_eval::SummarizeMetrics(
          all_current_metrics);
  const bool all_gate_pass =
      all_current_summary.samples >= 80 && all_obstacles.size() >= 8 &&
      all_current_summary.ade_1s_mean <= all_legacy_summary.ade_1s_mean * 1.50 &&
      all_current_summary.fde_2s_mean <= all_legacy_summary.fde_2s_mean * 1.60;
  apollo::prediction::free_move_record_eval::PrintSummaryLine(
      "all_dynamic", all_legacy_summary, all_current_summary,
      all_obstacles.size(), all_gate_pass);

  for (apollo::prediction::free_move_record_eval::MotionMode mode :
       {apollo::prediction::free_move_record_eval::MotionMode::kLinear,
        apollo::prediction::free_move_record_eval::MotionMode::kGentleTurn,
        apollo::prediction::free_move_record_eval::MotionMode::kSharpTurn}) {
    const auto legacy_summary =
        apollo::prediction::free_move_record_eval::SummarizeMetrics(
            legacy_metrics[mode]);
    const auto current_summary =
        apollo::prediction::free_move_record_eval::SummarizeMetrics(
            current_metrics[mode]);
    const bool gate_pass =
        apollo::prediction::free_move_record_eval::EvaluateIndustrialGate(
            mode, legacy_summary, current_summary,
            cohort_obstacles[mode].size());
    apollo::prediction::free_move_record_eval::PrintSummaryLine(
        apollo::prediction::free_move_record_eval::MotionModeName(mode),
        legacy_summary, current_summary, cohort_obstacles[mode].size(),
        gate_pass);
  }

  std::cout << "predict_mean_ms="
            << apollo::prediction::free_move_record_eval::Mean(
                   predict_latency_ms)
            << std::endl;
  std::cout << "predict_p95_ms="
            << apollo::prediction::free_move_record_eval::Percentile(
                   predict_latency_ms, 0.95)
            << std::endl;
  std::cout << "predict_p99_ms="
            << apollo::prediction::free_move_record_eval::Percentile(
                   predict_latency_ms, 0.99)
            << std::endl;
  std::cout << "predict_throughput_hz="
            << (predict_latency_ms.empty()
                    ? 0.0
                    : 1000.0 /
                          apollo::prediction::free_move_record_eval::Mean(
                              predict_latency_ms))
            << std::endl;
  return 0;
}
