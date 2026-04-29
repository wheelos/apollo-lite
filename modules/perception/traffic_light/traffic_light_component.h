#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "cyber/component/component.h"
#include "modules/perception/traffic_light/common/context.h"
#include "modules/perception/traffic_light/common/types.h"
#include "modules/perception/traffic_light/pipeline/pipeline.h"
#include "modules/perception/traffic_light/ports/provider_ports.h"
#include "modules/perception/traffic_light/ports/result_writer_port.h"
#include "modules/perception/traffic_light/stages/binder_stage.h"
#include "modules/perception/traffic_light/stages/detector_stage.h"
#include "modules/perception/traffic_light/stages/fusion_stage.h"
#include "modules/perception/traffic_light/stages/heuristic_stage.h"
#include "modules/perception/traffic_light/stages/prompter_stage.h"
#include "modules/perception/traffic_light/stages/tracker_stage.h"

namespace apollo {
namespace drivers {
class Image;
}
namespace perception {
class TrafficLightDetection;
}
namespace v2x {
class IntersectionTrafficLightData;
}
namespace perception {
namespace traffic_light {

class TrafficLightComponent : public apollo::cyber::Component<> {
 public:
  TrafficLightComponent() = default;
  ~TrafficLightComponent() override = default;

  bool Init() override;

  void SetDataProvider(std::shared_ptr<IDataProviderPort> data_provider) {
    data_provider_ = std::move(data_provider);
  }

  void SetPoseProvider(std::shared_ptr<IPoseProviderPort> pose_provider) {
    pose_provider_ = std::move(pose_provider);
  }

  void SetMapProvider(std::shared_ptr<IMapProviderPort> map_provider) {
    map_provider_ = std::move(map_provider);
  }

  void SetDetectorProvider(
      std::shared_ptr<IDetectorProviderPort> detector_provider) {
    detector_provider_ = std::move(detector_provider);
  }

  void SetV2XProvider(std::shared_ptr<IV2XProviderPort> v2x_provider) {
    v2x_provider_ = std::move(v2x_provider);
  }

  void SetResultWriter(std::shared_ptr<IResultWriterPort> result_writer) {
    result_writer_ = std::move(result_writer);
  }

  bool ProcessFrame(PipelineContext* context) {
    if (context == nullptr || pipeline_ == nullptr) {
      return false;
    }
    context->runtime_state = &runtime_state_;
    const bool ok = pipeline_->ProcessFrame(context);
    return ok && PublishDecision(*context);
  }

  bool ProcessOnceFromPorts();

 private:
  bool LoadOptions();
  bool InitDefaultPorts();
  bool InitReaders();
  bool PublishDecision(const PipelineContext& context);

  void RegisterDefaultStages() {
    pipeline_->RegisterStage(std::make_shared<PrompterStage>(options_.prompter));
    pipeline_->RegisterStage(std::make_shared<DetectorStage>(options_.detector));
    pipeline_->RegisterStage(std::make_shared<BinderStage>(options_.binder));
    pipeline_->RegisterStage(std::make_shared<TrackerStage>(options_.tracker));
    if (options_.enable_heuristic_stage) {
      pipeline_->RegisterStage(
          std::make_shared<HeuristicStage>(options_.heuristic));
    }
    pipeline_->RegisterStage(std::make_shared<FusionStage>(options_.fusion));
  }

  void OnReceiveImage(const std::shared_ptr<apollo::drivers::Image>& image,
                      const std::string& camera_name);
  void OnReceiveV2XMsg(
      const std::shared_ptr<apollo::v2x::IntersectionTrafficLightData>& v2x_msg);

  ComponentOptions options_;
  std::unique_ptr<PerceptionPipeline> pipeline_;
  RuntimeState runtime_state_;

  std::shared_ptr<IDataProviderPort> data_provider_;
  std::shared_ptr<IFrameInputPort> frame_input_port_;
  std::shared_ptr<IPoseProviderPort> pose_provider_;
  std::shared_ptr<IMapProviderPort> map_provider_;
  std::shared_ptr<IDetectorProviderPort> detector_provider_;
  std::shared_ptr<IV2XProviderPort> v2x_provider_;
  std::shared_ptr<IV2XInputPort> v2x_input_port_;
  std::shared_ptr<IResultWriterPort> result_writer_;

  std::shared_ptr<apollo::cyber::Writer<apollo::drivers::Image>>
      debug_image_writer_;

  std::mutex mutex_;
  uint64_t frame_counter_ = 0;
  double last_process_wall_ts_sec_ = 0.0;
  std::map<std::string, double> last_process_wall_ts_by_camera_sec_;
};

CYBER_REGISTER_COMPONENT(TrafficLightComponent);

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
