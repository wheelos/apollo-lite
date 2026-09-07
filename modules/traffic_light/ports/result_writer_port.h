#pragma once

#include "modules/traffic_light/common/context.h"

namespace apollo {
namespace traffic_light {

// Output abstraction for final arbitration result publishing.
class IResultWriterPort {
 public:
  virtual ~IResultWriterPort() = default;

  virtual bool Write(const PipelineContext& context,
                     const TrafficLightResult& result) = 0;
};

}  // namespace traffic_light
}  // namespace apollo
