// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#pragma once

#include <optional>
#include <cstddef>
#include <cstdint>
#include <string>

#include "native_joint_graph.pb.h"
#include "native_publication.h"
#include "openloc/localization/global/fixed_lag_alignment.h"

namespace apollo::localization::openloc_diagnostic {

struct NativeGraphFrame {
  NativePublication::State nominal;
  NativePublication::State joint;
  NativePublication::Result assessment;
  NativePublication::Result integrity;
  std::optional<::openloc::localization::unified::MotionIncrement> motion;
  std::optional<::openloc::localization::unified::JointLidarScan> scan;
  uint32_t previous_dimension = 0;
};

struct NativeDecodedGraph {
  ::openloc::localization::GraphOdometrySample odometry;
  std::optional<::openloc::localization::GraphMotionConnection> motion;
  std::optional<::openloc::localization::unified::JointLidarScan> scan;
};

constexpr std::size_t kMaximumNativeGraphBytes = 16 * 1024 * 1024;
constexpr uint32_t kMaximumNativeGraphDimension = 2048;

NativeJointOdometryInput EncodeNativeGraph(
    const NativeGraphFrame& frame, NativeOdometryDiagnostic odometry,
    const std::string& model_id);
NativeJointOdometryInput UnavailableNativeGraph(
    NativeOdometryDiagnostic odometry, const std::string& model_id,
    const std::string& detail);
NativeDecodedGraph DecodeNativeGraph(const NativeJointOdometryInput& input,
                                     const std::string& model_id,
                                     uint32_t maximum_dimension);

}  // namespace apollo::localization::openloc_diagnostic
