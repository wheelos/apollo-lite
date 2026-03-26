/******************************************************************************
 * Copyright 2026 The Wheel.OS Authors. All Rights Reserved.
 *****************************************************************************/
#include "modules/perception/traffic_light/infra/runtime_stage_factory.h"

#include <memory>

#include "cyber/common/log.h"
#include "modules/perception/traffic_light/algo/detector/traffic_light_detector_stage.h"
#include "modules/perception/traffic_light/algo/tracker/traffic_light_tracker_stage.h"
#include "modules/perception/traffic_light/domain/traffic_light_pipeline_config.h"

namespace apollo {
namespace perception {
namespace traffic_light {
namespace infra {

namespace {

constexpr char kModuleDetectorType[] = "TrafficLightDetectorStage";
constexpr char kModuleTrackerType[] = "TrafficLightTrackerStage";

class RuntimeTrafficLightDetectorPort final
    : public application::TrafficLightDetectorPort {
 public:
  RuntimeTrafficLightDetectorPort()
      : stage_(new algo::TrafficLightDetectorStage()) {}
  ~RuntimeTrafficLightDetectorPort() override = default;

  bool Configure(const pipeline::StageConfig& stage_config) override {
    if (stage_ == nullptr || configured_) {
      return false;
    }
    if (!stage_->ValidateStageConfig(stage_config)) {
      return false;
    }
    configured_ = stage_->Init(stage_config);
    return configured_;
  }

  bool Start() override {
    if (!configured_) {
      return false;
    }
    if (started_) {
      return true;
    }
    started_ = true;
    return true;
  }

  bool Run(const application::TrafficLightDetectorExecutionContext& context,
           camera::CameraFrame* frame) override {
    if (!started_ || stage_ == nullptr || frame == nullptr) {
      return false;
    }
    camera::TrafficLightDetectorRuntimeContext detector_context;
    detector_context.timestamp_sec = context.timestamp_sec;
    detector_context.camera_name = context.camera_name;
    return stage_->Detect(detector_context, frame);
  }

  bool Stop() override {
    if (stage_ == nullptr) {
      return false;
    }
    if (!started_) {
      return true;
    }
    started_ = false;
    return stage_->Shutdown();
  }

  std::string Name() const override { return stage_->Name(); }

 private:
  std::unique_ptr<camera::BaseTrafficLightDetector> stage_;
  bool configured_ = false;
  bool started_ = false;
};

class RuntimeTrafficLightTrackerPort final
    : public application::TrafficLightTrackerPort {
 public:
  RuntimeTrafficLightTrackerPort() : stage_(new algo::TrafficLightTrackerStage()) {}
  ~RuntimeTrafficLightTrackerPort() override = default;

  bool Configure(const pipeline::StageConfig& stage_config) override {
    if (stage_ == nullptr || configured_) {
      return false;
    }
    if (!stage_->ValidateStageConfig(stage_config)) {
      return false;
    }
    configured_ = stage_->Init(stage_config);
    return configured_;
  }

  bool Start() override {
    if (!configured_) {
      return false;
    }
    if (started_) {
      return true;
    }
    started_ = true;
    return true;
  }

  bool Run(const application::TrafficLightTrackerExecutionContext& context,
           camera::CameraFrame* frame) override {
    if (!started_ || stage_ == nullptr || frame == nullptr) {
      return false;
    }
    camera::TrafficLightTrackerRuntimeContext tracker_context;
    tracker_context.timestamp_sec = context.timestamp_sec;
    tracker_context.camera_name = context.camera_name;
    return stage_->Track(tracker_context, frame);
  }

  bool Stop() override {
    if (stage_ == nullptr) {
      return false;
    }
    if (!started_) {
      return true;
    }
    started_ = false;
    return stage_->Shutdown();
  }

  std::string Name() const override { return stage_->Name(); }

 private:
  std::unique_ptr<camera::BaseTrafficLightTracker> stage_;
  bool configured_ = false;
  bool started_ = false;
};

}  // namespace

std::unique_ptr<application::TrafficLightDetectorPort>
CreateRuntimeTrafficLightDetector(const pipeline::StageConfig& stage_config) {
  if (domain::ResolveTrafficLightDetectorType(stage_config) != kModuleDetectorType) {
    AERROR << "Unsupported traffic light detector type: "
           << domain::ResolveTrafficLightDetectorType(stage_config)
           << ", expected: " << kModuleDetectorType;
    return nullptr;
  }
  return std::unique_ptr<application::TrafficLightDetectorPort>(
      new RuntimeTrafficLightDetectorPort());
}

std::unique_ptr<application::TrafficLightTrackerPort>
CreateRuntimeTrafficLightTracker(const pipeline::StageConfig& stage_config) {
  if (domain::ResolveTrafficLightTrackerType(stage_config) != kModuleTrackerType) {
    AERROR << "Unsupported traffic light tracker type: "
           << domain::ResolveTrafficLightTrackerType(stage_config)
           << ", expected: " << kModuleTrackerType;
    return nullptr;
  }
  return std::unique_ptr<application::TrafficLightTrackerPort>(
      new RuntimeTrafficLightTrackerPort());
}

}  // namespace infra
}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
