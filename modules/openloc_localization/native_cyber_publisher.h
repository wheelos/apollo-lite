// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#pragma once

#include <exception>
#include <memory>
#include <mutex>

#include "native_publication.h"

#include "cyber/cyber.h"

namespace apollo::localization::openloc_diagnostic {

// A single diagnostic publisher, not a production localization/TF authority.
class NativeCyberPublisher {
 public:
  explicit NativeCyberPublisher(const std::shared_ptr<cyber::Node>& node,
                                double maximum_age,
                                bool recorded_input = false);
  ~NativeCyberPublisher();
  void Update(const NativePublication::State& state,
              const NativePublication::Result& integrity,
              NativePublication::Clock::time_point arrival);
  void Invalidate(const NativePublication::Result& failure);
  void Check() const;
  void Finish();

 private:
  void Publish();
  NativePublication publication_;
  std::shared_ptr<cyber::Writer<NativeOdometryDiagnostic>> odometry_;
  std::shared_ptr<cyber::Writer<transform::TransformStampeds>> transforms_;
  mutable std::mutex error_mutex_;
  std::mutex publication_mutex_;
  std::exception_ptr error_;
  const bool recorded_input_;
  cyber::Timer timer_;
};

}  // namespace apollo::localization::openloc_diagnostic
