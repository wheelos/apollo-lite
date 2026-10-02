// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include <deque>

#include "modules/localization/core/global_alignment.h"
#include "modules/localization/runtime/messages.h"
#include "wheelos_msgs/sensor_msgs/gnss_best_pose.pb.h"
#include "wheelos_msgs/sensor_msgs/heading.pb.h"

namespace apollo {
namespace localization {
namespace unified {

class GnssAdapter {
 public:
  GnssAdapter(const GlobalEstimatorConfig& config,
              const Eigen::Vector3d& antenna_in_base);
  Result ValidateConfig() const;
  Result AddHeading(const drivers::gnss::Heading& message, double now);
  Result Observe(const drivers::gnss::GnssBestPose& message, double now,
                 const LocalState& local, GlobalObservation* observation,
                 const GlobalAlignment* history = nullptr);
  const SourceCounters& heading_health() const { return heading_health_; }
  const SourceCounters& position_health() const { return position_health_; }

 private:
  struct HeadingSample {
    double time = 0.0;
    double yaw = 0.0;
    double variance = 0.0;
  };
  GlobalEstimatorConfig config_;
  Eigen::Vector3d antenna_in_base_;
  std::deque<HeadingSample> headings_;
  SourceCounters heading_health_;
  SourceCounters position_health_;
  uint64_t sequence_ = 0;
};

Result GeodeticToMap(const MapManifest& manifest, double latitude,
                     double longitude, double ellipsoid_height,
                     Eigen::Vector3d* position);

}  // namespace unified
}  // namespace localization
}  // namespace apollo
