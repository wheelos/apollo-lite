#pragma once

#include "modules/perception/traffic_light/common/context.h"

namespace apollo {
namespace perception {
namespace traffic_light {

// Data source abstraction for frame input injection.
class IDataProviderPort {
 public:
  virtual ~IDataProviderPort() = default;

  virtual bool PopulateFrameData(PipelineContext* context) = 0;
};

// Pose source abstraction for ego and camera pose injection.
class IPoseProviderPort {
 public:
  virtual ~IPoseProviderPort() = default;

  virtual bool PopulatePose(PipelineContext* context) = 0;
};

// HDMap source abstraction for candidate signal injection.
class IMapProviderPort {
 public:
  virtual ~IMapProviderPort() = default;

  virtual bool PopulateSignals(PipelineContext* context) = 0;
};

// V2X source abstraction for asynchronous signal evidence injection.
class IV2XProviderPort {
 public:
  virtual ~IV2XProviderPort() = default;

  virtual bool PopulateV2X(PipelineContext* context) = 0;
};

// Streaming frame input abstraction for online callback ingestion.
class IFrameInputPort {
 public:
  virtual ~IFrameInputPort() = default;

  virtual bool PushCameraFrame(uint64_t frame_id,
                               const CameraFrameState& frame) = 0;
};

// Streaming V2X input abstraction for asynchronous callback ingestion.
class IV2XInputPort {
 public:
  virtual ~IV2XInputPort() = default;

  virtual void PushV2XEvidence(const V2XLightEvidence& evidence) = 0;
};

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
