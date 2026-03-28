#pragma once

#include <vector>

#include "modules/perception/traffic_light/ports/result_writer_port.h"

namespace apollo {
namespace perception {
namespace traffic_light {

// Minimal in-memory result sink used by tests/offline demos.
class CollectingResultWriterPort : public IResultWriterPort {
 public:
  bool Write(const PipelineContext& context,
             const TrafficLightResult& result) override {
    last_frame_id_ = context.frame_id;
    last_result_ = result;
    history_.push_back(result);
    return true;
  }

  const TrafficLightResult& last_result() const { return last_result_; }

  uint64_t last_frame_id() const { return last_frame_id_; }

  const std::vector<TrafficLightResult>& history() const { return history_; }

 private:
  uint64_t last_frame_id_ = 0;
  TrafficLightResult last_result_;
  std::vector<TrafficLightResult> history_;
};

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
