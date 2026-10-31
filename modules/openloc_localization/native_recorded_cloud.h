// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#pragma once

#include "native_recorded_input.h"

#include "wheelos_msgs/sensor_msgs/pointcloud.pb.h"

namespace apollo::localization::openloc_diagnostic::recorded {

inline apollo::drivers::PointCloud ReadCloud(
    const std::filesystem::path& input, const std::vector<std::string>& source,
    const std::string& frame, uint32_t sequence) {
  const std::filesystem::path relative(source.at(2));
  if (relative.is_absolute())
    throw std::runtime_error("absolute raw point path");
  for (const auto& part : relative)
    if (part == "..") throw std::runtime_error("raw point path escaped input");
  const auto points = Read(input / relative, "timestamp_ns,x,y,z", 4);
  apollo::drivers::PointCloud cloud;
  cloud.mutable_header()->set_timestamp_sec(Time(source.at(1)) / 1e9);
  cloud.mutable_header()->set_sequence_num(sequence);
  cloud.set_measurement_time(Time(source.at(0)) / 1e9);
  cloud.set_frame_id(frame);
  for (const auto& point : points) {
    auto* raw = cloud.add_point();
    raw->set_timestamp(Time(point[0]));
    raw->set_x(Number(point[1]));
    raw->set_y(Number(point[2]));
    raw->set_z(Number(point[3]));
  }
  return cloud;
}

}  // namespace apollo::localization::openloc_diagnostic::recorded
