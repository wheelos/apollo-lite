// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>

#include "openloc/localization/local/native_types.h"

#include "native_odometry_diagnostic.pb.h"

namespace apollo::localization::openloc_diagnostic {

// One latest immutable snapshot, with no extrapolation or renewed source time.
// Only the publisher thread calls Sample; the owner may call Update
// concurrently.
class NativePublication {
 public:
  using Clock = std::chrono::steady_clock;
  using State = ::openloc::localization::unified::LocalState;
  using Result = ::openloc::localization::unified::Result;

  explicit NativePublication(double maximum_age, bool recorded_source_clock = false);
  void Update(const State& state, const Result& integrity,
              Clock::time_point arrival);
  void Invalidate(const Result& failure);
  NativeOdometryDiagnostic Sample(
      double publication_time,
      std::optional<Clock::time_point> now = std::nullopt);

 private:
  struct Snapshot {
    State state;
    Result integrity;
    Clock::time_point arrival;
  };
  const double maximum_age_;
  const bool recorded_source_clock_;
  std::mutex mutex_;
  std::shared_ptr<const Snapshot> latest_;
  std::optional<Result> terminal_failure_;
  std::shared_ptr<const Snapshot> last_emitted_;
  uint64_t publications_ = 0;
  std::optional<Clock::time_point> last_publication_;
};

}  // namespace apollo::localization::openloc_diagnostic
