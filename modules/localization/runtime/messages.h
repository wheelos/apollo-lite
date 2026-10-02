// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include <string>

#include "modules/localization/core/global_types.h"
#include "modules/localization/core/local_motion.h"
#include "modules/localization/proto/unified_localization.pb.h"

namespace apollo {
namespace localization {
namespace unified {

LocalConfig LocalPolicy(const LocalEstimatorConfig& config,
                        const std::string& session);
GlobalConfig GlobalPolicy(const GlobalEstimatorConfig& config);
void EncodeLocal(const LocalState& state, const LocalEstimatorConfig& config,
                 double now, LocalOdometry* message);
void EncodeMotion(const MotionIncrement& motion,
                  const LocalEstimatorConfig& config, double now,
                  LocalMotionIncrement* message);
// end must be the successfully decoded state of this same envelope.
Result DecodeMotion(const LocalOdometry& message, const LocalState& end,
                    MotionIncrement* motion);
Result DecodeLocal(const LocalOdometry& message,
                   const GlobalEstimatorConfig& config, double now,
                   LocalState* state);
Result DecodeObservation(const GlobalObservationMessage& message,
                         const GlobalEstimatorConfig& config, double now,
                         GlobalObservation* observation);
void EncodeObservation(const GlobalObservation& observation,
                       const GlobalEstimatorConfig& config, double now,
                       GlobalObservationMessage* message);
Result DecodePose(const LocalizationEstimate& message,
                  Eigen::Isometry3d* pose);
void EncodePose(const Eigen::Isometry3d& pose, const Matrix6d& covariance,
                double time, double now, const std::string& frame,
                LocalizationEstimate* message);
void EncodeGlobal(const GlobalState& global, const LocalState& local,
                  const GlobalEstimatorConfig& config, double now,
                  GlobalLocalization* message);
void EncodeSource(const std::string& name, const SourceCounters& source,
                  double now, double timeout, SourceAssessment* assessment);

class HealthReporter {
 public:
  LocalizationAssessment Evaluate(const std::string& owner,
                                  const LocalState* local, bool local_valid,
                                  bool global_valid, bool georeferenced,
                                  bool recovery_available, Reason reason,
                                  Recovery recovery, double now,
                                  bool global_owner,
                                  bool propagation_valid = false);
  bool TakeEvent(LocalizationHealthEvent* event);

 private:
  Availability state_ = UNKNOWN;
  Recovery recovery_ = IDLE;
  Epoch epoch_;
  uint64_t transition_ = 0;
  uint64_t sequence_ = 0;
  LocalizationHealthEvent event_;
  bool has_event_ = false;
};

}  // namespace unified
}  // namespace localization
}  // namespace apollo
