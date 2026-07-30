
#pragma once

#include <atomic>
#include <memory>
#include <string>

#include "modules/drivers/camera_gst/proto/config.pb.h"

#include "cyber/component/component.h"
#include "modules/drivers/camera_gst/camera_gst_driver.h"

namespace apollo {
namespace drivers {
namespace camera_gst {

class CameraGstComponent : public apollo::cyber::Component<> {
 public:
  ~CameraGstComponent() override;
  bool Init() override;

 private:
  bool ValidateGpuOnlyConfig() const;

  config::Config config_;
  std::unique_ptr<CameraGstDriver> driver_;
  std::atomic<uint64_t> gpu_frames_{0};
};

CYBER_REGISTER_COMPONENT(CameraGstComponent)

}  // namespace camera_gst
}  // namespace drivers
}  // namespace apollo
