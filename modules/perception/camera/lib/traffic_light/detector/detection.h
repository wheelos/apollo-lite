#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "torch/script.h"
#include "torch/torch.h"

#include "modules/perception/pipeline/proto/stage/detection.pb.h"

#include "cyber/common/macros.h"
#include "modules/perception/base/image_8u.h"
#include "modules/perception/camera/lib/interface/base_traffic_light_detector.h"
#include "modules/perception/camera/lib/traffic_light/detector/cropbox.h"
#include "modules/perception/camera/lib/traffic_light/detector/select.h"

namespace apollo {
namespace perception {
namespace camera {

struct LetterboxInfo {
    float scale;
    int pad_w;
    int pad_h;
};

class TrafficLightDetection : public BaseTrafficLightDetector {
 public:
  TrafficLightDetection();
  ~TrafficLightDetection();

  bool Init(const TrafficLightDetectorInitOptions &options) override;
  bool Init(const StageConfig &stage_config) override;

  bool Detect(const TrafficLightDetectorOptions &options,
              CameraFrame *frame) override;

  bool Process(DataFrame *data_frame) override;

  bool IsEnabled() const override { return enable_; }
  std::string Name() const override { return "TrafficLightDetection"; }

 private:
  bool InitInternal(const std::string &root_dir, int gpu_id);

  bool Inference(std::vector<base::TrafficLightPtr> *lights,
                 DataProvider *data_provider);

  bool ProcessYOLOOutput(
    const torch::Tensor &output_tensor,
                         const std::vector<base::RectI> &crop_box_list,
    const std::vector<LetterboxInfo> &letterbox_infos,
                         std::vector<base::TrafficLightPtr> *detected_lights);

  void ApplyNMS(std::vector<base::TrafficLightPtr> *lights,
                double iou_thresh = 0.6);

  base::TLColor ClassIdToColor(apollo::perception::base::TLDetectionClass class_id) const;

 private:
  TrafficLightDetectionConfig detection_param_;
  std::string detection_root_dir_;
  int gpu_id_ = 0;

  torch::jit::script::Module torch_model_;
  torch::Device device_;

  // Auxiliary object
  std::shared_ptr<base::Image8U> image_ = nullptr;
  std::shared_ptr<IGetBox> crop_;
  Select select_;

  std::vector<base::RectI> crop_box_list_;
  std::vector<float> resize_scale_list_;
  std::vector<base::TrafficLightPtr> detected_bboxes_;
  std::map<base::TLDetectionClass, base::TLColor> class_id_map_;

  DataProvider::ImageOptions data_provider_image_option_;

  DISALLOW_COPY_AND_ASSIGN(TrafficLightDetection);
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
