// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#pragma once

#include <memory>

#include "cyber/component/timer_component.h"
#include "modules/localization/localization_runtime.h"
#include "modules/transform/transform_broadcaster.h"

namespace apollo::localization::openloc {

class OpenLocLocalizationComponent final : public cyber::TimerComponent {
 public:
  ~OpenLocLocalizationComponent() override;
  bool Init() override;

 private:
  bool Proc() override;
  std::shared_ptr<LocalizationRuntime> runtime_;
  std::shared_ptr<cyber::Reader<drivers::gnss::Imu>> imu_;
  std::shared_ptr<cyber::Reader<drivers::PointCloud>> lidar_;
  std::shared_ptr<cyber::Reader<drivers::gnss::GnssBestPose>> gnss_;
  std::shared_ptr<cyber::Reader<drivers::gnss::Heading>> heading_;
  std::shared_ptr<cyber::Writer<LocalizationEstimate>> pose_;
  std::shared_ptr<cyber::Writer<LocalizationStatus>> status_;
  std::unique_ptr<transform::TransformBroadcaster> broadcaster_;
  uint32_t sequence_ = 0;
};

CYBER_REGISTER_COMPONENT(OpenLocLocalizationComponent)

}  // namespace apollo::localization::openloc
