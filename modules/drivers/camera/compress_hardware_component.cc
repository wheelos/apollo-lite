#include "modules/drivers/camera/compress_hardware_component.h"

namespace apollo {
namespace drivers {
namespace camera {

bool HardwareCompressComponent::Init() {
  if (!GetProtoConfig(&config_)) {
    AERROR << "Parse config file failed: " << ConfigFilePath();
    return false;
  }
  AINFO << "Camera config: \n" << config_.DebugString();

  // Initialize image pool and writer
  try {
    image_pool_.reset(new CCObjectPool<CompressedImage>(
        config_.compress_conf().image_pool_size()));
    image_pool_->ConstructAll();
  } catch (const std::bad_alloc& e) {
    AERROR << e.what();
    return false;
  }

  writer_ = node_->CreateWriter<CompressedImage>(
      config_.compress_conf().output_channel());

  // 1. 初始化 CUDA 和 NvJPEG
  cudaStreamCreate(&stream_);
  nvjpegCreateSimple(&nvjpeg_handle_);
  nvjpegEncoderStateCreate(nvjpeg_handle_, &nvjpeg_state_, stream_);
  nvjpegEncoderParamsCreate(nvjpeg_handle_, &nvjpeg_params_, stream_);

  // 设置采样因子 (YUV420通常比444压缩率高)
  nvjpegEncoderParamsSetSamplingFactors(nvjpeg_params_, NVJPEG_CSS_420,
                                        stream_);
  nvjpegEncoderParamsSetQuality(nvjpeg_params_, 80, stream_);

  // 预分配 GPU 内存 (按照最大分辨率 1920*1080*3)
  size_t max_img_bytes = 1920 * 1080 * 3;
  cudaMalloc(&dev_input_buffer_, max_img_bytes);
  dev_input_size_ = max_img_bytes;

  return true;
}

bool HardwareCompressComponent::Proc(const std::shared_ptr<Image>& image) {
  // 保护共享的 GPU Buffer 资源
  // NvJPEG Handle 是线程安全的，但中间的 Buffer 不是
  std::lock_guard<std::mutex> lock(mutex_);

  size_t input_size = image->data().size();
  if (input_size > dev_input_size_) {
    if (dev_input_buffer_) cudaFree(dev_input_buffer_);
    cudaMalloc(&dev_input_buffer_, input_size);  // 考虑对齐
    dev_input_size_ = input_size;
  }

  // 1. Copy to GPU
  cudaMemcpyAsync(dev_input_buffer_, image->data().data(), input_size,
                  cudaMemcpyHostToDevice, stream_);

  // 2. Setup Input
  nvjpegImage_t nv_image;
  nv_image.channel[0] = (unsigned char*)dev_input_buffer_;
  nv_image.pitch[0] = image->width() * 3;

  // 3. Encode
  // 假设输入格式为 RGB，如果是 YUYV 需要先转码或告知 nvjpeg
  nvjpegEncodeImage(nvjpeg_handle_, nvjpeg_state_, nvjpeg_params_, &nv_image,
                    NVJPEG_INPUT_RGBI, image->width(), image->height(),
                    stream_);

  // 4. Retrieve Size
  size_t length;
  // 此函数在 stream 也就是 GPU 完成前会阻塞 CPU 吗？
  // 官方文档：RetrieveBitstream 会同步等待 stream 完成。
  nvjpegEncodeRetrieveBitstream(nvjpeg_handle_, nvjpeg_state_, NULL, &length,
                                stream_);

  // 5. Retrieve Data
  // 优化：使用 Pinned Memory (Page-locked) 加速回传
  // 如果没有 pinned memory，普通 vector 也行，但较慢
  std::vector<uint8_t> obuffer(length);
  nvjpegEncodeRetrieveBitstream(nvjpeg_handle_, nvjpeg_state_, obuffer.data(),
                                &length, stream_);

  // 显式同步（虽然 Retrieve 内部可能已同步，为了保险）
  cudaStreamSynchronize(stream_);

  // 6. Publish
  auto out_msg = image_pool_->GetObject();
  if (!out_msg) {
    // Pool empty handling
    return false;
  }
  out_msg->mutable_header()->CopyFrom(image->header());
  out_msg->set_format("jpeg");
  out_msg->set_data(obuffer.data(), length);
  writer_->Write(out_msg);

  return true;
}

HardwareCompressComponent::~HardwareCompressComponent() {
  if (dev_input_buffer_) {
    cudaFree(dev_input_buffer_);
  }
  nvjpegEncoderParamsDestroy(nvjpeg_params_);
  nvjpegEncoderStateDestroy(nvjpeg_state_);
  nvjpegDestroy(nvjpeg_handle_);
  cudaStreamDestroy(stream_);
}

}  // namespace camera
}  // namespace drivers
}  // namespace apollo
