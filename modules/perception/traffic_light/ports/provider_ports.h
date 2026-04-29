#pragma once

#include "modules/perception/traffic_light/common/context.h"

namespace apollo {
namespace perception {
namespace traffic_light {

class IDataProviderPort {
 public:
  virtual ~IDataProviderPort() = default;
  virtual bool PopulateFrameData(PipelineContext* context) = 0;
};

class IPoseProviderPort {
 public:
  virtual ~IPoseProviderPort() = default;
  virtual bool PopulatePose(PipelineContext* context) = 0;
};

class IMapProviderPort {
 public:
  virtual ~IMapProviderPort() = default;
  virtual bool PopulateSignals(PipelineContext* context) = 0;
};

class IDetectorProviderPort {
 public:
  virtual ~IDetectorProviderPort() = default;
  virtual bool PopulateDetections(PipelineContext* context) = 0;
};

class IV2XProviderPort {
 public:
  virtual ~IV2XProviderPort() = default;
  virtual bool PopulateV2X(PipelineContext* context) = 0;
};

class IFrameInputPort {
 public:
  virtual ~IFrameInputPort() = default;
  virtual bool PushCameraFrame(uint64_t frame_id,
                               const CameraFrameState& frame) = 0;
};

class IV2XInputPort {
 public:
  virtual ~IV2XInputPort() = default;
  virtual void PushV2XEvidence(const V2XLightEvidence& evidence) = 0;
};

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
