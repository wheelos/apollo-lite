// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#pragma once

#include "cyber/cyber.h"
#include "native_global_worker.h"
#include "wheelos_msgs/sensor_msgs/gnss.pb.h"

namespace apollo::localization::openloc_diagnostic {

class NativeGlobalCyber {
 public:
  NativeGlobalCyber(const std::shared_ptr<cyber::Node>& node,
                    const NativeGlobalHostConfig& config);
  ~NativeGlobalCyber();
  void Check() const;
  void Finish();

 private:
  void Publish();
  void StopInputs();
  struct Shared;
  std::shared_ptr<cyber::Writer<NativeGlobalDiagnostic>> output_;
  std::shared_ptr<cyber::Writer<NativeGlobalProcessed>> processed_;
  std::shared_ptr<Shared> shared_;
  std::shared_ptr<cyber::Reader<NativeOdometryDiagnostic>> odometry_;
  std::shared_ptr<cyber::Reader<NativeJointOdometryInput>> graph_odometry_;
  std::shared_ptr<cyber::Reader<NativeGlobalGnssInput>> gnss_;
  std::shared_ptr<cyber::Reader<drivers::gnss::RawData>> raw_gga_;
  std::shared_ptr<cyber::Reader<drivers::gnss::GnssBestPose>> best_pose_;
  std::shared_ptr<cyber::Reader<drivers::gnss::Heading>> heading_;
  std::shared_ptr<cyber::Reader<NativeGlobalMapInput>> map_;
  std::shared_ptr<cyber::Reader<drivers::PointCloud>> raw_map_;
  std::shared_ptr<cyber::Reader<NativeGlobalResetRequest>> reset_;
  std::shared_ptr<cyber::Reader<NativeGlobalResetAck>> reset_ack_;
  cyber::Timer timer_;
};

}  // namespace apollo::localization::openloc_diagnostic
