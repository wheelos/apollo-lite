#include "modules/drivers/camera/compress_hardware_component.h"

#include <algorithm>

namespace {

constexpr int kJpegQuality = 80;
constexpr size_t kDefaultReserveBytes = 2 * 1024 * 1024;

inline bool CheckCuda(cudaError_t status, const char* call) {
  if (status == cudaSuccess) {
    return true;
  }
  AERROR << "CUDA call failed: " << call << ", error: "
         << cudaGetErrorString(status);
  return false;
}

inline bool CheckNvjpeg(nvjpegStatus_t status, const char* call) {
  if (status == NVJPEG_STATUS_SUCCESS) {
    return true;
  }
  AERROR << "NVJPEG call failed: " << call << ", status: "
         << static_cast<int>(status);
  return false;
}

}  // namespace

namespace apollo {
namespace drivers {
namespace camera {

bool HardwareCompressComponent::Init() {
  if (!GetProtoConfig(&config_)) {
    AERROR << "Parse config file failed: " << ConfigFilePath();
    return false;
  }
  AINFO << "Camera config: \n" << config_.DebugString();

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

  host_bitstream_reserve_bytes_ = kDefaultReserveBytes;
  host_bitstream_buffer_.reserve(host_bitstream_reserve_bytes_);

  if (!InitializeNvjpeg()) {
    ReleaseNvjpeg();
    return false;
  }

  // Pre-allocate for 1080p RGB frames as a baseline.
  const size_t max_img_bytes = 1920ULL * 1080ULL * 3ULL;
  if (!EnsureDeviceInputCapacity(max_img_bytes)) {
    ReleaseNvjpeg();
    return false;
  }

  return true;
}

bool HardwareCompressComponent::Proc(const std::shared_ptr<Image>& image) {
  if (!image) {
    AERROR << "Input image is null.";
    return false;
  }

  if (!nvjpeg_ready_) {
    AERROR << "HardwareCompressComponent is not initialized.";
    return false;
  }

  if (image->width() <= 0 || image->height() <= 0 || image->step() <= 0) {
    AERROR << "Invalid image metadata: width=" << image->width()
           << ", height=" << image->height() << ", step=" << image->step();
    return false;
  }

  if (image->encoding() != "rgb8") {
    AERROR << "Hardware NVJPEG path currently supports only rgb8 input. Got: "
           << image->encoding();
    return false;
  }

  const size_t expected_min_bytes =
      static_cast<size_t>(image->height()) * image->step();
  if (image->data().size() < expected_min_bytes) {
    AERROR << "Image buffer is smaller than expected. actual="
           << image->data().size() << ", expected_min=" << expected_min_bytes;
    return false;
  }

  // Protect shared encoder state and reusable buffers.
  std::lock_guard<std::mutex> lock(mutex_);

  const size_t input_size = expected_min_bytes;
  if (!EnsureDeviceInputCapacity(input_size)) {
    return false;
  }

  if (!CheckCuda(cudaMemcpyAsync(dev_input_buffer_, image->data().data(),
                                 input_size, cudaMemcpyHostToDevice, stream_),
                 "cudaMemcpyAsync(H2D)")) {
    return false;
  }

  nvjpegImage_t nv_image{};
  nv_image.channel[0] = reinterpret_cast<unsigned char*>(dev_input_buffer_);
  nv_image.pitch[0] = image->step();

  if (!CheckNvjpeg(nvjpegEncodeImage(nvjpeg_handle_, nvjpeg_state_,
                                     nvjpeg_params_, &nv_image,
                                     NVJPEG_INPUT_RGBI, image->width(),
                                     image->height(), stream_),
                   "nvjpegEncodeImage")) {
    return false;
  }

  size_t length = 0;
  if (!CheckNvjpeg(nvjpegEncodeRetrieveBitstream(
                       nvjpeg_handle_, nvjpeg_state_, nullptr, &length,
                       stream_),
                   "nvjpegEncodeRetrieveBitstream(size)")) {
    return false;
  }

  if (length == 0) {
    AERROR << "NVJPEG returned empty bitstream.";
    return false;
  }

  if (host_bitstream_buffer_.size() < length) {
    host_bitstream_buffer_.resize(length);
  }
  size_t output_length = length;
  if (!CheckNvjpeg(nvjpegEncodeRetrieveBitstream(
                       nvjpeg_handle_, nvjpeg_state_,
                       host_bitstream_buffer_.data(), &output_length, stream_),
                   "nvjpegEncodeRetrieveBitstream(data)")) {
    return false;
  }

  if (!CheckCuda(cudaStreamSynchronize(stream_), "cudaStreamSynchronize")) {
    return false;
  }

  auto out_msg = image_pool_->GetObject();
  if (!out_msg) {
    AERROR << "Compressed image pool is exhausted.";
    return false;
  }

  out_msg->mutable_header()->CopyFrom(image->header());
  out_msg->set_format("jpeg");
  out_msg->set_data(host_bitstream_buffer_.data(), output_length);
  writer_->Write(out_msg);

  return true;
}

HardwareCompressComponent::~HardwareCompressComponent() {
  std::lock_guard<std::mutex> lock(mutex_);
  ReleaseNvjpeg();
}

bool HardwareCompressComponent::EnsureDeviceInputCapacity(size_t bytes) {
  if (bytes == 0) {
    AERROR << "Invalid input size for device buffer allocation.";
    return false;
  }

  if (bytes <= dev_input_size_ && dev_input_buffer_ != nullptr) {
    return true;
  }

  if (dev_input_buffer_) {
    if (!CheckCuda(cudaFree(dev_input_buffer_), "cudaFree(dev_input_buffer_)")) {
      return false;
    }
    dev_input_buffer_ = nullptr;
    dev_input_size_ = 0;
  }

  if (!CheckCuda(cudaMalloc(&dev_input_buffer_, bytes), "cudaMalloc(dev_input_buffer_)")) {
    return false;
  }
  dev_input_size_ = bytes;
  return true;
}

bool HardwareCompressComponent::InitializeNvjpeg() {
  if (!CheckCuda(cudaStreamCreate(&stream_), "cudaStreamCreate")) {
    return false;
  }

  if (!CheckNvjpeg(nvjpegCreateSimple(&nvjpeg_handle_), "nvjpegCreateSimple")) {
    return false;
  }

  if (!CheckNvjpeg(nvjpegEncoderStateCreate(nvjpeg_handle_, &nvjpeg_state_,
                                            stream_),
                   "nvjpegEncoderStateCreate")) {
    return false;
  }

  if (!CheckNvjpeg(nvjpegEncoderParamsCreate(nvjpeg_handle_, &nvjpeg_params_,
                                             stream_),
                   "nvjpegEncoderParamsCreate")) {
    return false;
  }

  if (!CheckNvjpeg(
          nvjpegEncoderParamsSetSamplingFactors(nvjpeg_params_, NVJPEG_CSS_420,
                                                stream_),
          "nvjpegEncoderParamsSetSamplingFactors")) {
    return false;
  }

  if (!CheckNvjpeg(
          nvjpegEncoderParamsSetQuality(nvjpeg_params_, kJpegQuality, stream_),
          "nvjpegEncoderParamsSetQuality")) {
    return false;
  }

  nvjpeg_ready_ = true;
  return true;
}

void HardwareCompressComponent::ReleaseNvjpeg() noexcept {
  if (dev_input_buffer_ != nullptr) {
    if (cudaFree(dev_input_buffer_) != cudaSuccess) {
      AERROR << "Failed to free device input buffer during cleanup.";
    }
    dev_input_buffer_ = nullptr;
    dev_input_size_ = 0;
  }

  if (nvjpeg_params_ != nullptr) {
    if (nvjpegEncoderParamsDestroy(nvjpeg_params_) != NVJPEG_STATUS_SUCCESS) {
      AERROR << "Failed to destroy nvjpeg params during cleanup.";
    }
    nvjpeg_params_ = nullptr;
  }

  if (nvjpeg_state_ != nullptr) {
    if (nvjpegEncoderStateDestroy(nvjpeg_state_) != NVJPEG_STATUS_SUCCESS) {
      AERROR << "Failed to destroy nvjpeg encoder state during cleanup.";
    }
    nvjpeg_state_ = nullptr;
  }

  if (nvjpeg_handle_ != nullptr) {
    if (nvjpegDestroy(nvjpeg_handle_) != NVJPEG_STATUS_SUCCESS) {
      AERROR << "Failed to destroy nvjpeg handle during cleanup.";
    }
    nvjpeg_handle_ = nullptr;
  }

  if (stream_ != nullptr) {
    if (cudaStreamDestroy(stream_) != cudaSuccess) {
      AERROR << "Failed to destroy cuda stream during cleanup.";
    }
    stream_ = nullptr;
  }

  nvjpeg_ready_ = false;
}

}  // namespace camera
}  // namespace drivers
}  // namespace apollo
