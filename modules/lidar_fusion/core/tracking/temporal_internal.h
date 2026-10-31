// Copyright 2026 WheelOS. All Rights Reserved.

#pragma once

#include <deque>
#include <map>
#include <memory>
#include <set>
#include <utility>

#include "Eigen/Geometry"
#include "tracking/core/multi_object_tracker.h"
#include "tracking/estimation/predictor.h"

#include "modules/lidar_fusion/core/tracking/temporal.h"

namespace apollo {
namespace lidar_fusion {
namespace temporal_detail {

namespace tracking = common::tracking;
using Vector = Eigen::Vector3d;

struct TemporalPrior {
  GroundPrior ground;
  ObjectPrior objects;
};

struct Measurement {
  tracking::VectorMeasurement position;
  Box box;
  bool unknown = false;
  bool point_supported_footprint = false;
  bool conflict = false;
  ClassificationUse classification_use = CLASSIFICATION_UNASSESSED;
  UnifiedClass cls = CLASS_UNKNOWN;
  Source source = GEOMETRY;
  uint32_t candidate = 0;
  uint64_t prior_track = 0;
};

using Mot = tracking::MultiObjectTracker<tracking::GaussianState, Measurement>;
using Track = Mot::Track;

Vector V(const Vec3& point);
void Set(const Vector& value, Vec3* point);
Eigen::Isometry3d Transform(const LocalPose& pose);
Box TransformBox(const Box& box, const Eigen::Isometry3d& transform);

struct Vote {
  std::deque<UnifiedClass> classes;
  uint32_t conflicts = 0;
  UnifiedClass current = CLASS_UNKNOWN;
};

using CellKey = std::pair<int, int>;

struct CellState {
  double log_odds = 0;
  double stamp = 0;
  double ground_stamp = 0;
  double ground_z = 0;
  bool ground = false;
};

}  // namespace temporal_detail

struct StatefulFusion::Impl {
  FusionConfig config;
  SingleFrameFusion single;
  std::unique_ptr<temporal_detail::Mot> tracker;
  std::map<uint64_t, temporal_detail::Vote> votes;
  std::map<temporal_detail::CellKey, temporal_detail::CellState> history;
  double stamp = 0;
  uint64_t epoch = 0;
  bool initialized = false;
  bool active = false;
  bool diagnostic = false;
  Eigen::Isometry3d previous_pose = Eigen::Isometry3d::Identity();
  std::map<uint64_t, std::set<temporal_detail::CellKey>> last_regions;

  void Reset();
  temporal_detail::TemporalPrior Prior(double now,
                                       const Eigen::Isometry3d& pose) const;
  void Map(const Eigen::Isometry3d& pose, FusionOutput* output);
};

}  // namespace lidar_fusion
}  // namespace apollo
