// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include <memory>
#include <random>
#include <vector>

#include "modules/localization/map/likelihood_field.h"

namespace apollo {
namespace localization {
namespace unified {

struct ParticlePolicy {
  size_t particles = 0;
  size_t maximum_scan_points = 0;
  size_t maximum_groups = 0;
  size_t minimum_groups = 0;
  size_t maximum_modes = 0;
  size_t maximum_support_regions = 0;
  double group_resolution = 0.0;
  double distance_sigma = 0.0;
  double inlier_distance = 0.0;
  double minimum_observed_fraction = 0.0;
  double minimum_inlier_fraction = 0.0;
  double outlier_floor = 0.0;
  double score_temperature = 0.0;
  double resample_ess_fraction = 0.0;
  double mode_position_radius = 0.0;
  double mode_angle_radius = 0.0;
  // Search perturbations only, not independent Gaussian ODOM error claims.
  double translation_diffusion = 0.0;
  double rotation_diffusion = 0.0;
  double maximum_interval = 0.0;
  double maximum_motion_translation = 0.0;
  double maximum_motion_rotation = 0.0;
  double maximum_query_age = 0.0;
  double future_tolerance = 0.0;
  double solve_budget_ms = 0.0;
};

struct ParticleDomain {
  Eigen::Vector3d minimum = Eigen::Vector3d::Zero();
  Eigen::Vector3d maximum = Eigen::Vector3d::Zero();
  double yaw_minimum = 0.0;
  double yaw_maximum = 0.0;
  // Fixed conditional attitude prior, NOT marginalized attitude uncertainty.
  // No gravity or traversability is inferred from a PCD.
  Eigen::Quaterniond tilt = Eigen::Quaterniond::Identity();
};

struct ParticleQuery {
  FieldIdentity identity;
  LocalState local;
  std::string source;
  uint64_t scan_sequence = 0;
  // Already in base at local.stamp.time. Caller owns deskew/calibration.
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud;
  bool measurement_time_compensated = false;
};

struct ParticleMode {
  // Representative sample, not an average across disconnected basins.
  std::string support_region_id;
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  double search_mass = 0.0;
  double observed_fraction = 0.0;
  double inlier_fraction = 0.0;
  double unknown_fraction = 0.0;
  double outside_fraction = 0.0;
  size_t samples = 0;
};

struct ParticleSupportAllocation {
  std::string region_id;
  size_t initial_particles = 0;
  double prior_search_mass = 0.0;
};

struct ParticleSnapshot {
  FieldIdentity identity;
  Epoch epoch;
  Stamp stamp;
  std::string source;
  uint64_t scan_sequence = 0;
  bool diagnostic_only = true;
  bool search_coverage_complete = false;
  bool local_covariance_model_valid = false;
  double effective_samples = 0.0;
  bool resampled = false;
  bool modes_truncated = false;
  double unreported_mass = 0.0;
  size_t query_groups = 0;
  size_t evaluated_groups = 0;
  size_t observed_groups = 0;
  size_t unknown_groups = 0;
  size_t outside_groups = 0;
  size_t declared_support_regions = 0;
  size_t weighted_support_regions = 0;
  std::vector<ParticleSupportAllocation> initial_support_allocation;
  std::vector<ParticleMode> modes;
};

// Single-threaded diagnostic search. No TF, GlobalObservation, covariance or
// handover API: normalized search mass is not a calibrated posterior.
class ParticleRelocalizer {
 public:
  ParticleRelocalizer(const ParticlePolicy& policy,
                      std::shared_ptr<const LikelihoodField> field);
  Result Initialize(const ParticleDomain& domain, const LocalState& local,
                    uint64_t random_seed);
  // Failure leaves output, particles, frontier and random stream unchanged.
  // Callers must honor Result; an old snapshot is not a fresh success.
  Result Update(const ParticleQuery& query, ParticleSnapshot* output);
  void Reset();

 private:
  struct Particle {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    std::string support_region_id;
    double weight = 0.0;
    double observed_fraction = 0.0;
    double inlier_fraction = 0.0;
    double unknown_fraction = 0.0;
    double outside_fraction = 0.0;
  };
  Result ValidatePolicy() const;
  ParticlePolicy policy_;
  std::shared_ptr<const LikelihoodField> field_;
  std::vector<Particle> particles_;
  LocalState previous_;
  std::string source_;
  uint64_t scan_sequence_ = 0;
  std::mt19937_64 random_;
  std::vector<std::string> support_region_ids_;
  std::vector<ParticleSupportAllocation> initial_support_allocation_;
};

}  // namespace unified
}  // namespace localization
}  // namespace apollo
