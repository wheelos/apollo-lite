#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

#include <cuda_runtime.h>
#include <nvjpeg.h>

#include "cyber/cyber.h"
#include "cyber/component/component.h"
#include "cyber/base/concurrent_object_pool.h"
#include "modules/common_msgs/sensor_msgs/sensor_image.pb.h"
#include "modules/drivers/camera/proto/config.pb.h"

namespace apollo {
namespace drivers {
namespace camera {

using apollo::cyber::Component;
using apollo::cyber::Writer;
using apollo::cyber::base::CCObjectPool;
using apollo::drivers::Image;
using apollo::drivers::CompressedImage;
using apollo::drivers::camera::config::Config;

class HardwareCompressComponent : public cyber::Component<Image> {
 public:
  HardwareCompressComponent() = default;
  ~HardwareCompressComponent();

  bool Init() override;

  bool Proc(const std::shared_ptr<Image>& image) override;

 private:
  bool EnsureDeviceInputCapacity(size_t bytes);
  bool InitializeNvjpeg();
  void ReleaseNvjpeg() noexcept;

  Config config_;
  std::shared_ptr<CCObjectPool<CompressedImage>> image_pool_;
  std::shared_ptr<Writer<CompressedImage>> writer_;

  nvjpegHandle_t nvjpeg_handle_ = nullptr;
  nvjpegEncoderState_t nvjpeg_state_ = nullptr;
  nvjpegEncoderParams_t nvjpeg_params_ = nullptr;
  cudaStream_t stream_ = nullptr;
  std::mutex mutex_;

  // Reused GPU input staging buffer for NVJPEG RGBI input.
  void* dev_input_buffer_ = nullptr;
  size_t dev_input_size_ = 0;

  // Reused host buffer for encoded bitstream to reduce reallocations.
  std::vector<uint8_t> host_bitstream_buffer_;
  size_t host_bitstream_reserve_bytes_ = 0;

  bool nvjpeg_ready_ = false;
};

CYBER_REGISTER_COMPONENT(HardwareCompressComponent)

}  // namespace camera
}  // namespace drivers
}  // namespace apollo
