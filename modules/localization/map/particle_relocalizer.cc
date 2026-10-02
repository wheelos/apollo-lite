// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "modules/localization/map/particle_relocalizer.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iterator>
#include <limits>
#include <map>
#include <numeric>
#include <tuple>
#include <utility>

namespace apollo {
namespace localization {
namespace unified {
namespace {

constexpr double kPi = 3.14159265358979323846;

bool Positive(double value) { return std::isfinite(value) && value > 0.0; }

bool ValidLocal(const LocalState& local) {
  return local.valid && !local.epoch.session.empty() &&
         local.epoch.generation != 0 && !local.stamp.clock_id.empty() &&
         local.stamp.sequence != 0 && Positive(local.stamp.time) &&
         std::isfinite(local.stamp.receive_time) &&
         local.position.allFinite() &&
         local.position.cwiseAbs().maxCoeff() <= 1e6 &&
         local.orientation.coeffs().allFinite() &&
         std::abs(local.orientation.norm() - 1.0) <= 1e-9;
}

double Angle(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b) {
  return LogRotation(Eigen::Quaterniond(
      a.linear().transpose() * b.linear())).norm();
}

}  // namespace

ParticleRelocalizer::ParticleRelocalizer(
    const ParticlePolicy& policy, std::shared_ptr<const LikelihoodField> field)
    : policy_(policy), field_(std::move(field)) {}

Result ParticleRelocalizer::ValidatePolicy() const {
  if (!field_ || policy_.particles < 2 || policy_.particles > 4096 ||
      policy_.maximum_scan_points == 0 ||
      policy_.maximum_scan_points > 100000 ||
      policy_.maximum_groups == 0 || policy_.maximum_groups > 4096 ||
      policy_.minimum_groups == 0 ||
      policy_.minimum_groups > policy_.maximum_groups ||
      policy_.maximum_groups > policy_.maximum_scan_points ||
      policy_.particles * policy_.maximum_groups > 4000000 ||
      policy_.maximum_modes == 0 || policy_.maximum_modes > 64 ||
      policy_.maximum_support_regions == 0 ||
      policy_.maximum_support_regions > 256 ||
      !Positive(policy_.group_resolution) ||
      policy_.group_resolution < 0.01 ||
      !Positive(policy_.distance_sigma) ||
      policy_.distance_sigma < field_->quantization_radius() ||
      !Positive(policy_.inlier_distance) ||
      policy_.inlier_distance <= field_->quantization_radius() ||
      policy_.inlier_distance >= field_->config().maximum_distance ||
      !Positive(policy_.mode_position_radius) ||
      !Positive(policy_.mode_angle_radius) ||
      policy_.mode_angle_radius > kPi ||
      !Positive(policy_.score_temperature) || policy_.score_temperature > 100.0 ||
      !Positive(policy_.maximum_interval) ||
      !Positive(policy_.maximum_motion_translation) ||
      !Positive(policy_.maximum_motion_rotation) ||
      policy_.maximum_motion_rotation > kPi ||
      !Positive(policy_.maximum_query_age) ||
      !Positive(policy_.solve_budget_ms)) {
    return {Reason::CONFIG_INVALID, "Invalid bounded particle search policy"};
  }
  for (double value : {policy_.minimum_observed_fraction,
                       policy_.minimum_inlier_fraction,
                       policy_.outlier_floor,
                       policy_.resample_ess_fraction}) {
    if (!Positive(value) || value > 1.0) {
      return {Reason::CONFIG_INVALID, "Invalid particle score fraction"};
    }
  }
  if (policy_.outlier_floor >= 1.0 ||
      !std::isfinite(policy_.translation_diffusion) ||
      policy_.translation_diffusion < 0.0 ||
      !std::isfinite(policy_.rotation_diffusion) ||
      policy_.rotation_diffusion < 0.0 ||
      policy_.translation_diffusion * std::sqrt(policy_.maximum_interval) >
          policy_.maximum_motion_translation ||
      policy_.rotation_diffusion * std::sqrt(policy_.maximum_interval) >
          policy_.maximum_motion_rotation ||
      !std::isfinite(policy_.future_tolerance) ||
      policy_.future_tolerance < 0.0) {
    return {Reason::CONFIG_INVALID, "Invalid search diffusion/clock policy"};
  }
  return {};
}

Result ParticleRelocalizer::Initialize(const ParticleDomain& domain,
                                      const LocalState& local,
                                      uint64_t random_seed) {
  const auto valid = ValidatePolicy();
  if (!valid.ok()) {
    return valid;
  }
  if (!ValidLocal(local) || !domain.minimum.allFinite() ||
      !domain.maximum.allFinite() ||
      (domain.maximum.array() <= domain.minimum.array()).any() ||
      (domain.minimum.array() < field_->config().minimum.array()).any() ||
      (domain.maximum.array() >= field_->config().maximum.array()).any() ||
      !std::isfinite(domain.yaw_minimum) || !std::isfinite(domain.yaw_maximum) ||
      domain.yaw_minimum < -kPi || domain.yaw_maximum > kPi ||
      domain.yaw_maximum < domain.yaw_minimum ||
      !domain.tilt.coeffs().allFinite() ||
      std::abs(domain.tilt.norm() - 1.0) > 1e-9) {
    return {Reason::INVALID_INPUT, "Invalid explicit particle domain/local reference"};
  }
  std::vector<Region3d> clipped_regions;
  std::vector<double> volumes;
  for (const auto& region : field_->config().support_regions) {
    Region3d clipped;
    clipped.id = region.id;
    clipped.minimum = region.minimum.cwiseMax(domain.minimum);
    clipped.maximum = region.maximum.cwiseMin(domain.maximum);
    if ((clipped.maximum.array() <= clipped.minimum.array()).any()) {
      continue;
    }
    clipped_regions.push_back(clipped);
    volumes.push_back((clipped.maximum - clipped.minimum).prod());
  }
  if (clipped_regions.empty() ||
      clipped_regions.size() > policy_.maximum_support_regions ||
      clipped_regions.size() > policy_.particles) {
    return {Reason::MAP_NOT_READY,
            "Search domain has no covered support regions or exceeds region budget"};
  }
  std::mt19937_64 random(random_seed);
  std::uniform_real_distribution<double> uniform(0.0, 1.0);
  std::vector<Particle> particles(policy_.particles);
  std::vector<ParticleSupportAllocation> support_allocation;
  support_allocation.reserve(clipped_regions.size());
  const size_t remaining = policy_.particles - clipped_regions.size();
  const double total_volume =
      std::accumulate(volumes.begin(), volumes.end(), 0.0);
  std::vector<size_t> quota(clipped_regions.size(), 1);
  std::vector<std::pair<double, size_t>> remainders;
  size_t assigned = 0;
  for (size_t i = 0; i < volumes.size(); ++i) {
    const double exact =
        static_cast<double>(remaining) * volumes[i] / total_volume;
    const size_t whole = static_cast<size_t>(std::floor(exact));
    quota[i] += whole;
    assigned += whole;
    remainders.emplace_back(exact - whole, i);
  }
  std::stable_sort(remainders.begin(), remainders.end(),
      [](const auto& a, const auto& b) { return a.first > b.first; });
  for (size_t i = 0; i < remaining - assigned; ++i) {
    ++quota[remainders[i].second];
  }
  size_t particle_index = 0;
  for (size_t region_index = 0; region_index < clipped_regions.size();
       ++region_index) {
    const size_t count = quota[region_index];
    support_allocation.push_back(
        {clipped_regions[region_index].id, count,
         volumes[region_index] / total_volume});
    std::array<std::vector<size_t>, 4> permutations;
    for (auto& permutation : permutations) {
      permutation.resize(count);
      std::iota(permutation.begin(), permutation.end(), 0);
      std::shuffle(permutation.begin(), permutation.end(), random);
    }
    for (size_t i = 0; i < count; ++i, ++particle_index) {
      auto& particle = particles[particle_index];
      particle.support_region_id = clipped_regions[region_index].id;
      for (int axis = 0; axis < 3; ++axis) {
        const double fraction =
            (static_cast<double>(permutations[axis][i]) + uniform(random)) /
            static_cast<double>(count);
        particle.pose.translation()(axis) =
            clipped_regions[region_index].minimum(axis) + fraction *
            (clipped_regions[region_index].maximum(axis) -
             clipped_regions[region_index].minimum(axis));
      }
      const double yaw_fraction =
          (static_cast<double>(permutations[3][i]) + uniform(random)) /
          static_cast<double>(count);
      const double yaw = domain.yaw_minimum + yaw_fraction *
          (domain.yaw_maximum - domain.yaw_minimum);
      particle.pose.linear() =
          (Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())) *
           domain.tilt).toRotationMatrix();
      particle.weight =
          volumes[region_index] /
          (total_volume * static_cast<double>(count));
    }
  }
  particles_ = std::move(particles);
  previous_ = local;
  random_ = random;
  source_.clear();
  scan_sequence_ = 0;
  support_region_ids_.clear();
  for (const auto& region : clipped_regions) {
    support_region_ids_.push_back(region.id);
  }
  initial_support_allocation_ = std::move(support_allocation);
  return {};
}

void ParticleRelocalizer::Reset() {
  particles_.clear();
  previous_ = LocalState();
  source_.clear();
  scan_sequence_ = 0;
  support_region_ids_.clear();
  initial_support_allocation_.clear();
}

Result ParticleRelocalizer::Update(const ParticleQuery& query,
                                  ParticleSnapshot* output) {
  if (output == nullptr || particles_.empty()) {
    return {Reason::HISTORY_UNAVAILABLE, "Particle search is not initialized"};
  }
  if (!(query.identity == field_->config().identity)) {
    return {Reason::MAP_MISMATCH, "Particle query map/calibration mismatch"};
  }
  if (query.local.epoch != previous_.epoch) {
    return {Reason::EPOCH_MISMATCH, "Particle query changed ODOM epoch"};
  }
  if (!ValidLocal(query.local) || query.source.empty() ||
      query.scan_sequence == 0 || !query.cloud ||
      !query.measurement_time_compensated ||
      query.cloud->size() < policy_.minimum_groups ||
      query.cloud->size() > policy_.maximum_scan_points) {
    return {Reason::INVALID_INPUT, "Invalid measurement-time particle query"};
  }
  if (query.local.stamp.clock_id != previous_.stamp.clock_id ||
      query.local.stamp.receive_time - query.local.stamp.time >
          policy_.maximum_query_age ||
      query.local.stamp.time - query.local.stamp.receive_time >
          policy_.future_tolerance) {
    return {Reason::CLOCK_INVALID, "Particle query clock/age invalid"};
  }
  if (!source_.empty() && query.source != source_) {
    return {Reason::INVALID_INPUT, "Changing scan source requires a new search"};
  }
  if (query.scan_sequence <= scan_sequence_) {
    return {Reason::DUPLICATE, "Particle scan identity did not advance"};
  }
  const double dt = query.local.stamp.time - previous_.stamp.time;
  const bool same_initial_state =
      scan_sequence_ == 0 && dt == 0.0 &&
      query.local.stamp.sequence == previous_.stamp.sequence &&
      Pose(query.local).matrix().isApprox(Pose(previous_).matrix(), 1e-12);
  if (!same_initial_state &&
      (dt <= 0.0 || query.local.stamp.sequence <= previous_.stamp.sequence)) {
    return {Reason::TIMESTAMP_REGRESSION, "Particle local reference did not advance"};
  }
  if (dt > policy_.maximum_interval) {
    return {Reason::HISTORY_UNAVAILABLE, "Particle motion interval exceeds budget"};
  }
  const Eigen::Isometry3d delta =
      Pose(previous_).inverse() * Pose(query.local);
  if (delta.translation().norm() > policy_.maximum_motion_translation ||
      Angle(Eigen::Isometry3d::Identity(), delta) >
          policy_.maximum_motion_rotation) {
    return {Reason::INVALID_INPUT, "Particle nominal motion exceeds search contract"};
  }
  using Clock = std::chrono::steady_clock;
  const auto start = Clock::now();
  const auto expired = [&]() {
    return std::chrono::duration<double, std::milli>(
        Clock::now() - start).count() > policy_.solve_budget_ms;
  };
  using Cell = std::tuple<int64_t, int64_t, int64_t>;
  std::map<Cell, Eigen::Vector3d> groups;
  for (const auto& point : *query.cloud) {
    const Eigen::Vector3d p(point.x, point.y, point.z);
    if (!p.allFinite() || p.cwiseAbs().maxCoeff() > 1e6) {
      return {Reason::INVALID_INPUT, "Nonfinite or out-of-range query point"};
    }
    const Eigen::Vector3d cell =
        (p / policy_.group_resolution).array().floor();
    const Cell key(static_cast<int64_t>(cell.x()),
                   static_cast<int64_t>(cell.y()),
                   static_cast<int64_t>(cell.z()));
    const auto inserted = groups.emplace(key, p);
    // Stable representative under input reordering and duplicated returns.
    if (!inserted.second &&
        std::make_tuple(p.x(), p.y(), p.z()) <
            std::make_tuple(inserted.first->second.x(),
                            inserted.first->second.y(),
                            inserted.first->second.z())) {
      inserted.first->second = p;
    }
  }
  if (groups.size() < policy_.minimum_groups) {
    return {Reason::MATCH_FAILED, "Insufficient distinct spatial query groups"};
  }
  std::vector<Eigen::Vector3d> points;
  const size_t count = std::min(groups.size(), policy_.maximum_groups);
  auto group = groups.begin();
  size_t cursor = 0;
  for (size_t i = 0; i < count; ++i) {
    const size_t index = i * groups.size() / count;
    while (cursor < index) {
      ++group;
      ++cursor;
    }
    points.push_back(group->second);
  }
  auto particles = particles_;
  auto random = random_;
  std::normal_distribution<double> normal(0.0, 1.0);
  std::vector<double> logs(particles.size(),
                           -std::numeric_limits<double>::infinity());
  double maximum_log = -std::numeric_limits<double>::infinity();
  const double translation_scale = policy_.translation_diffusion * std::sqrt(dt);
  const double rotation_scale = policy_.rotation_diffusion * std::sqrt(dt);
  for (size_t i = 0; i < particles.size(); ++i) {
    if (expired()) {
      return {Reason::MATCH_FAILED, "Particle computation deadline exceeded"};
    }
    auto& particle = particles[i];
    Eigen::Isometry3d perturbation = Eigen::Isometry3d::Identity();
    Eigen::Vector3d angle;
    for (int axis = 0; axis < 3; ++axis) {
      perturbation.translation()(axis) = translation_scale * normal(random);
      angle(axis) = rotation_scale * normal(random);
    }
    if (!angle.allFinite() || !std::isfinite(angle.norm()) ||
        !perturbation.translation().allFinite()) {
      return {Reason::INVALID_INPUT, "Particle search perturbation overflow"};
    }
    perturbation.linear() = ExpRotation(angle).toRotationMatrix();
    particle.pose = particle.pose * delta * perturbation;
    if (!particle.pose.matrix().allFinite()) {
      return {Reason::INVALID_INPUT, "Particle prediction overflow"};
    }
    size_t observed = 0;
    size_t unknown = 0;
    size_t outside = 0;
    size_t inliers = 0;
    double score = 0.0;
    for (const auto& point : points) {
      double distance = 0.0;
      const auto cell = field_->Query(particle.pose * point, &distance);
      if (cell == FieldCellStatus::OBSERVED) {
        ++observed;
        // Cell-center distances have bounded discretization error. This
        // geometric tolerance is not a localization covariance assertion.
        inliers += distance + field_->quantization_radius() <=
                   policy_.inlier_distance;
        const double scaled = distance / policy_.distance_sigma;
        const double likelihood =
            policy_.outlier_floor + (1.0 - policy_.outlier_floor) *
                                        std::exp(-0.5 * scaled * scaled);
        score += std::log(likelihood);
      } else if (cell == FieldCellStatus::UNKNOWN) {
        ++unknown;
      } else {
        ++outside;
      }
    }
    particle.observed_fraction =
        static_cast<double>(observed) / static_cast<double>(points.size());
    particle.inlier_fraction =
        observed == 0 ? 0.0 :
            static_cast<double>(inliers) / static_cast<double>(observed);
    particle.unknown_fraction =
        static_cast<double>(unknown) / static_cast<double>(points.size());
    particle.outside_fraction =
        static_cast<double>(outside) / static_cast<double>(points.size());
    if (observed < policy_.minimum_groups ||
        particle.observed_fraction < policy_.minimum_observed_fraction ||
        particle.inlier_fraction < policy_.minimum_inlier_fraction ||
        particle.weight <= 0.0) {
      continue;
    }
    // Mean group score caps information per scan. This remains a heuristic
    // search weight, NOT an independence claim across groups or scans.
    logs[i] = std::log(particle.weight) +
              policy_.score_temperature * score /
                  static_cast<double>(observed);
    maximum_log = std::max(maximum_log, logs[i]);
  }
  if (!std::isfinite(maximum_log)) {
    return {Reason::MATCH_FAILED, "No particle has sufficient map support"};
  }
  double sum = 0.0;
  for (size_t i = 0; i < particles.size(); ++i) {
    particles[i].weight = std::exp(logs[i] - maximum_log);
    sum += particles[i].weight;
  }
  double squared_sum = 0.0;
  for (auto& particle : particles) {
    particle.weight /= sum;
    squared_sum += particle.weight * particle.weight;
  }
  ParticleSnapshot snapshot;
  snapshot.identity = query.identity;
  snapshot.epoch = query.local.epoch;
  snapshot.stamp = query.local.stamp;
  snapshot.source = query.source;
  snapshot.scan_sequence = query.scan_sequence;
  snapshot.local_covariance_model_valid =
      query.local.covariance_model_valid && previous_.covariance_model_valid;
  snapshot.effective_samples = 1.0 / squared_sum;
  snapshot.query_groups = groups.size();
  snapshot.evaluated_groups = points.size();
  snapshot.declared_support_regions = support_region_ids_.size();
  snapshot.initial_support_allocation = initial_support_allocation_;
  for (const auto& id : support_region_ids_) {
    snapshot.weighted_support_regions += std::any_of(
        particles.begin(), particles.end(), [&id](const Particle& particle) {
          return particle.support_region_id == id && particle.weight > 0.0;
        });
  }
  std::vector<size_t> order(particles.size());
  std::iota(order.begin(), order.end(), 0);
  std::stable_sort(order.begin(), order.end(), [&particles](size_t a, size_t b) {
    return particles[a].weight > particles[b].weight;
  });
  for (size_t index : order) {
    const auto& particle = particles[index];
    if (particle.weight == 0.0) {
      continue;
    }
    auto mode = std::find_if(snapshot.modes.begin(), snapshot.modes.end(),
        [&](const ParticleMode& item) {
          return (item.pose.translation() - particle.pose.translation()).norm() <=
                     policy_.mode_position_radius &&
                 Angle(item.pose, particle.pose) <= policy_.mode_angle_radius &&
                 item.support_region_id == particle.support_region_id;
        });
    if (mode == snapshot.modes.end()) {
      if (snapshot.modes.size() >= policy_.maximum_modes) {
        snapshot.modes_truncated = true;
        snapshot.unreported_mass += particle.weight;
        continue;
      }
      ParticleMode next;
      next.pose = particle.pose;
      next.support_region_id = particle.support_region_id;
      next.observed_fraction = particle.observed_fraction;
      next.inlier_fraction = particle.inlier_fraction;
      next.unknown_fraction = particle.unknown_fraction;
      next.outside_fraction = particle.outside_fraction;
      snapshot.modes.push_back(next);
      mode = std::prev(snapshot.modes.end());
    } else {
      const double old_mass = mode->search_mass;
      const double total_mass = old_mass + particle.weight;
      mode->observed_fraction =
          (old_mass * mode->observed_fraction +
           particle.weight * particle.observed_fraction) / total_mass;
      mode->inlier_fraction =
          (old_mass * mode->inlier_fraction +
           particle.weight * particle.inlier_fraction) / total_mass;
      mode->unknown_fraction =
          (old_mass * mode->unknown_fraction +
           particle.weight * particle.unknown_fraction) / total_mass;
      mode->outside_fraction =
          (old_mass * mode->outside_fraction +
           particle.weight * particle.outside_fraction) / total_mass;
    }
    mode->search_mass += particle.weight;
    ++mode->samples;
  }
  std::stable_sort(snapshot.modes.begin(), snapshot.modes.end(),
      [](const ParticleMode& a, const ParticleMode& b) {
        return a.search_mass > b.search_mass;
      });
  if (!snapshot.modes.empty()) {
    const auto& primary = snapshot.modes.front();
    snapshot.observed_groups = static_cast<size_t>(std::llround(
        primary.observed_fraction * points.size()));
    snapshot.unknown_groups = static_cast<size_t>(std::llround(
        primary.unknown_fraction * points.size()));
    snapshot.outside_groups = points.size() - std::min(
        points.size(), snapshot.observed_groups + snapshot.unknown_groups);
  }
  if (snapshot.effective_samples <
      policy_.resample_ess_fraction * static_cast<double>(particles.size())) {
    std::vector<Particle> resampled;
    resampled.reserve(particles.size());
    std::vector<size_t> supported;
    std::vector<double> cdf;
    double cumulative = 0.0;
    for (size_t i = 0; i < particles.size(); ++i) {
      if (particles[i].weight > 0.0) {
        supported.push_back(i);
        cumulative += particles[i].weight;
        cdf.push_back(cumulative);
      }
    }
    for (auto& value : cdf) {
      value /= cumulative;
    }
    cdf.back() = 1.0;
    const double step = 1.0 / static_cast<double>(particles.size());
    std::uniform_real_distribution<double> uniform(0.0, step);
    const double offset = uniform(random);
    size_t ancestor = 0;
    for (size_t i = 0; i < particles.size(); ++i) {
      const double target = std::min(
          std::nextafter(1.0, 0.0), offset + static_cast<double>(i) * step);
      while (ancestor + 1 < cdf.size() && target >= cdf[ancestor]) {
        ++ancestor;
      }
      resampled.push_back(particles[supported[ancestor]]);
      resampled.back().weight = step;
    }
    particles = std::move(resampled);
    snapshot.resampled = true;
  }
  if (expired()) {
    return {Reason::MATCH_FAILED, "Particle computation deadline exceeded"};
  }
  particles_ = std::move(particles);
  random_ = random;
  previous_ = query.local;
  source_ = query.source;
  scan_sequence_ = query.scan_sequence;
  *output = std::move(snapshot);
  return {};
}

}  // namespace unified
}  // namespace localization
}  // namespace apollo
