// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#pragma once

#include <deque>
#include <functional>
#include <unordered_map>
#include <vector>

#include "modules/localization/core/types.h"

namespace apollo {
namespace localization {
namespace unified {

using StateLookup = std::function<Result(double, LocalState*)>;

struct DeskewedScan {
  Stamp stamp;
  Epoch epoch;
  std::vector<Eigen::Vector3d> points_in_end_base;
};

Result DeskewLidarScan(const LidarScan& scan, const StateLookup& lookup,
                       DeskewedScan* deskewed);

struct LioUpdate {
  Eigen::MatrixXd observation;
  Eigen::VectorXd innovation;
  Eigen::MatrixXd noise;
  Eigen::MatrixXd observed_pose_basis;
  uint32_t correspondences = 0;
};

class LocalLio {
 public:
  explicit LocalLio(const LocalConfig& config) : config_(config) {}

  Result Observe(const LidarScan& scan, const StateLookup& lookup,
                 LocalState* state);
  void Reset(const Epoch& epoch);
  const Eigen::MatrixXd& observed_pose_basis() const {
    return observed_pose_basis_;
  }
  uint32_t scan_count() const { return scans_.size(); }
  std::vector<uint64_t> geometry_source_sequences() const;

 private:
  struct StoredScan {
    Epoch epoch;
    uint64_t sequence = 0;
    std::vector<Eigen::Vector3d> points;
    double anchor_variance = 0.0;
  };

  struct Plane {
    Eigen::Vector3d point = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();
    double variance = 0.0;
  };

  struct VoxelKey {
    int x = 0;
    int y = 0;
    int z = 0;
    bool operator==(const VoxelKey& other) const {
      return x == other.x && y == other.y && z == other.z;
    }
  };

  struct VoxelHash {
    std::size_t operator()(const VoxelKey& key) const;
  };

  VoxelKey Key(const Eigen::Vector3d& point) const;
  void Rebuild();
  Result BuildUpdate(const DeskewedScan& scan, const LocalState& state,
                     LioUpdate* update) const;
  Result ApplyUpdate(const LioUpdate& update, LocalState* state) const;
  void Insert(const DeskewedScan& scan, const LocalState& state);

  LocalConfig config_;
  Epoch epoch_;
  std::deque<StoredScan> scans_;
  std::unordered_map<VoxelKey, Plane, VoxelHash> planes_;
  Eigen::MatrixXd observed_pose_basis_;
};

}  // namespace unified
}  // namespace localization
}  // namespace apollo
