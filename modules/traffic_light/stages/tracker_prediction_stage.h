// Copyright 2026 The Apollo Authors. All Rights Reserved.
#pragma once

#include <memory>
#include <string>
#include <utility>

#include "modules/traffic_light/stages/tracker_stage.h"

namespace apollo {
namespace traffic_light {

class TrackerPredictionStage : public BaseStage {
 public:
  explicit TrackerPredictionStage(std::shared_ptr<TrackerStage> tracker)
      : tracker_(std::move(tracker)) {}

  std::string Name() const override { return "TrackerPredictionStage"; }

  bool Process(PipelineContext* context) override {
    return tracker_ != nullptr && tracker_->Predict(context);
  }

 private:
  std::shared_ptr<TrackerStage> tracker_;
};

}  // namespace traffic_light
}  // namespace apollo
