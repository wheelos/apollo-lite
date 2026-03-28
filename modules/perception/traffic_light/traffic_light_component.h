#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "cyber/component/component.h"
#include "modules/perception/traffic_light/common/context.h"
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

  // 默认从注入端口拉取一帧数据，适配在线组件和离线回放两种模式。
  bool ProcessOnceFromPorts() {
    if (pipeline_ == nullptr || data_provider_ == nullptr) {
      return false;
    }

    PipelineContext context;
    context.runtime_state = &runtime_state_;
    if (!data_provider_->PopulateFrameData(&context)) {
      return false;
    }
    if (pose_provider_ != nullptr && !pose_provider_->PopulatePose(&context)) {
      context.status.degrade_reason = "pose unavailable";
    }
    if (map_provider_ != nullptr && !map_provider_->PopulateSignals(&context)) {
      context.status.degrade_reason = "hdmap unavailable";
    }
    if (v2x_provider_ != nullptr && !v2x_provider_->PopulateV2X(&context)) {
      context.status.degrade_reason = "v2x unavailable";
    }
    return ProcessFrame(&context);
  }

 private:
  void OnReceiveImage(const std::shared_ptr<apollo::drivers::Image>& image,
                      const std::string& camera_name);

  void OnReceiveV2XMsg(
      const std::shared_ptr<apollo::v2x::IntersectionTrafficLightData>&
          v2x_msg);

  bool InitDefaultPorts();

  bool InitReaders();

  void RegisterDefaultStages() {
    pipeline_->RegisterStage(std::make_shared<PrompterStage>());
    pipeline_->RegisterStage(std::make_shared<DetectorStage>());
    pipeline_->RegisterStage(std::make_shared<SemanticBinderStage>());
    pipeline_->RegisterStage(std::make_shared<TrackerStage>());
    pipeline_->RegisterStage(std::make_shared<HeuristicStage>());
    pipeline_->RegisterStage(std::make_shared<FusionStage>());
  }

  bool PublishDecision(const PipelineContext& context);

  std::unique_ptr<PerceptionPipeline> pipeline_;
  RuntimeState runtime_state_;

  std::shared_ptr<IDataProviderPort> data_provider_;
  std::shared_ptr<IFrameInputPort> frame_input_port_;
  std::shared_ptr<IPoseProviderPort> pose_provider_;
  std::shared_ptr<IMapProviderPort> map_provider_;
  std::shared_ptr<IV2XProviderPort> v2x_provider_;
  std::shared_ptr<IV2XInputPort> v2x_input_port_;
  std::shared_ptr<IResultWriterPort> result_writer_;

  std::vector<std::string> camera_names_ = {"front_6mm"};
  std::vector<std::string> camera_channel_names_ = {
      "/apollo/sensor/camera/front_6mm/image"};
  std::string v2x_channel_name_ = "/apollo/v2x/traffic_light";
  std::string output_channel_name_ = "/apollo/perception/traffic_light";
  std::string debug_output_channel_name_ =
      "/apollo/perception/traffic_light/debug";
  std::string debug_image_channel_name_ =
      "/apollo/perception/traffic_light/debug/image";
  bool enable_debug_recording_ = true;
  bool enable_debug_image_stream_ = true;

  std::shared_ptr<apollo::cyber::Writer<apollo::drivers::Image>>
      debug_image_writer_;

  std::mutex mutex_;
  uint64_t frame_counter_ = 0;
};

CYBER_REGISTER_COMPONENT(TrafficLightComponent);

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
