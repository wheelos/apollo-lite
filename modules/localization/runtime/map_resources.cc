// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "modules/localization/runtime/map_resources.h"

#include <cmath>
#include <utility>

namespace apollo {
namespace localization {
namespace unified {

Result LoadGlobalMap(const GlobalEstimatorConfig& config,
                     std::shared_ptr<MapMatcher>* matcher,
                     std::vector<Eigen::Isometry3d>* seeds) {
  if (matcher == nullptr || seeds == nullptr) {
    return {Reason::INVALID_INPUT, "Missing map resource output"};
  }
  if (!config.enable_map_matching()) {
    matcher->reset();
    seeds->clear();
    return {};
  }
  if (!config.has_map_position_variance_floor() ||
      !config.has_map_rotation_variance_floor() ||
      config.map().recovery_seeds_size() >
          static_cast<int>(config.maximum_recovery_seeds()) ||
      config.maximum_scan_points() < config.minimum_points()) {
    return {Reason::CONFIG_INVALID, "Missing map floors or invalid recovery bounds"};
  }
  MatchConfig policy;
  policy.voxel_size = config.voxel_size();
  policy.resolution = config.ndt_resolution();
  policy.max_fitness = config.max_fitness();
  policy.correspondence_distance = config.correspondence_distance();
  policy.minimum_overlap = config.minimum_overlap();
  policy.minimum_points = config.minimum_points();
  policy.max_translation = config.max_match_translation();
  policy.max_rotation = config.max_match_rotation();
  policy.minimum_information = config.minimum_information_eigenvalue();
  policy.maximum_condition = config.max_information_condition();
  policy.position_variance_floor = config.map_position_variance_floor();
  policy.rotation_variance_floor = config.map_rotation_variance_floor();
  policy.ambiguity_margin = config.ambiguity_score_margin();
  policy.maximum_seeds = config.maximum_recovery_seeds();
  policy.rotation_length = config.geometry_rotation_length();
  auto candidate = std::make_shared<MapMatcher>(policy);
  const auto loaded = candidate->Load(config.map().pcd_path());
  if (!loaded.ok()) {
    return loaded;
  }
  std::vector<Eigen::Isometry3d> candidates;
  for (const auto& seed : config.map().recovery_seeds()) {
    Eigen::Isometry3d pose;
    const auto decoded = DecodePose(seed, &pose);
    if (!decoded.ok() || !seed.has_header() ||
        seed.header().frame_id() != config.map().frame_id()) {
      return {Reason::CONFIG_INVALID, "Invalid recovery seed/map frame"};
    }
    candidates.push_back(pose);
  }
  *matcher = std::move(candidate);
  *seeds = std::move(candidates);
  return {};
}

Result ValidateMapSwitch(const MapSwitchRequest& request,
                         const GlobalEstimatorConfig& active,
                         const Epoch& epoch, uint64_t last_request,
                         double now) {
  if (!request.IsInitialized() || !std::isfinite(now) ||
      !std::isfinite(request.request_time()) || request.request_time() <= 0.0 ||
      request.request_time() - now > active.future_tolerance() ||
      now - request.request_time() > active.max_observation_age() ||
      request.request_id() == 0) {
    return {Reason::INVALID_INPUT, "Invalid/stale map switch request"};
  }
  if (request.request_id() <= last_request) {
    return {Reason::DUPLICATE, "Map switch request sequence did not advance"};
  }
  if (request.current_epoch().producer_session() != epoch.session ||
      request.current_epoch().generation() != epoch.generation ||
      epoch.session.empty() || epoch.generation == 0) {
    return {Reason::EPOCH_MISMATCH, "Map switch targets a different local epoch"};
  }
  const auto& next = request.next_map();
  if (request.expected_map_id() != active.map().map_id() ||
      request.expected_map_version() != active.map().version()) {
    return {Reason::MAP_MISMATCH, "Active map changed before switch admission"};
  }
  if (next.map_id().empty() || next.version().empty() ||
      next.frame_id() != active.map().frame_id() ||
      next.calibration_id() != active.map().calibration_id() ||
      (next.map_id() == active.map().map_id() &&
       next.version() == active.map().version()) ||
      (active.enable_map_matching() && next.pcd_path().empty())) {
    return {Reason::MAP_MISMATCH,
            "Map switch requires a new version and unchanged TF/calibration contract"};
  }
  return {};
}

}  // namespace unified
}  // namespace localization
}  // namespace apollo
