// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#pragma once

#include <memory>
#include <optional>

#include "native_global_diagnostic.pb.h"
#include "openloc/localization/online/gnss_position_gate.h"
#include "wheelos_msgs/sensor_msgs/gnss_best_pose.pb.h"
#include "wheelos_msgs/sensor_msgs/heading.pb.h"

namespace apollo::localization::openloc_diagnostic {

struct NativeStructuredPosition {
  uint64_t sequence = 0;
  int64_t measurement_ns = 0;
  int64_t receive_ns = 0;
  ::openloc::localization::Status status;
  std::optional<NativeGlobalGnssInput> observation;
};

struct NativeStructuredHeading {
  uint64_t sequence = 0;
  int64_t measurement_ns = 0;
  int64_t receive_ns = 0;
  ::openloc::localization::Status status;
  double baseline_yaw_rad = 0;
  double variance_rad2 = 0;
};

// Serialized receiver adapter, not an estimator. Height/INS attitude are not
// synthesized into a full-pose observation.
class NativeStructuredGnssAdapter {
 public:
  explicit NativeStructuredGnssAdapter(const NativeGlobalHostConfig& config);
  ~NativeStructuredGnssAdapter();
  void Reset(const std::string& session, uint64_t generation,
             int64_t minimum_measurement_ns = 0);
  NativeStructuredPosition Position(const drivers::gnss::GnssBestPose& input,
                                   int64_t receive_ns);
  NativeStructuredHeading Heading(const drivers::gnss::Heading& input,
                                 int64_t receive_ns);

 private:
  ::openloc::localization::Status Time(double source, int64_t receive,
                                     int64_t* measurement) const;
  class Projection;
  NativeGlobalHostConfig config_;
  ::openloc::localization::GnssPositionGate position_gate_;
  std::unique_ptr<Projection> projection_;
  std::string session_;
  uint64_t generation_ = 0;
  uint64_t position_sequence_ = 0;
  uint64_t heading_sequence_ = 0;
  int64_t minimum_measurement_ns_ = 0;
  int64_t last_heading_ns_ = 0;
  int64_t heading_fixed_start_ns_ = 0;
};

}  // namespace apollo::localization::openloc_diagnostic
