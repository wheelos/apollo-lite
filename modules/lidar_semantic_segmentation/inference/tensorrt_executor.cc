#include "modules/lidar_semantic_segmentation/inference/tensorrt_executor.h"

#include <NvInfer.h>
#include <NvInferPlugin.h>
#include <NvInferVersion.h>

#include <cuda_runtime_api.h>

#include <fstream>
#include <numeric>
#include <string>
#include <vector>

#include "cyber/common/log.h"

namespace apollo {
namespace lidar_semantic_segmentation {

namespace {

class TensorRtLogger final : public nvinfer1::ILogger {
 public:
  void log(Severity severity, const char* message) noexcept override {
    if (severity <= Severity::kERROR) {
      AERROR << message;
    } else if (severity == Severity::kWARNING) {
      AWARN << message;
    }
  }
};

TensorRtLogger g_logger;

bool CudaSucceeded(cudaError_t error, const char* operation) {
  if (error == cudaSuccess) {
    return true;
  }
  AERROR << operation << " failed: " << cudaGetErrorString(error);
  return false;
}

template <typename TensorRtType>
void DestroyTensorRt(TensorRtType* object) {
  if (object == nullptr) {
    return;
  }
#if NV_TENSORRT_MAJOR >= 10
  delete object;
#else
  object->destroy();
#endif
}

std::size_t ElementCount(const std::vector<int64_t>& dimensions) {
  return std::accumulate(dimensions.begin(), dimensions.end(), std::size_t{1},
                         [](std::size_t product, int64_t dimension) {
                           return product * static_cast<std::size_t>(dimension);
                         });
}

nvinfer1::Dims ToDims(const std::vector<int64_t>& dimensions) {
  nvinfer1::Dims dims;
  dims.nbDims = static_cast<int>(dimensions.size());
  for (int index = 0; index < dims.nbDims; ++index) {
    dims.d[index] =
        static_cast<int>(dimensions[static_cast<std::size_t>(index)]);
  }
  return dims;
}

bool HasStaticDimensions(const nvinfer1::Dims& dimensions,
                         const std::vector<int64_t>& expected) {
  if (dimensions.nbDims != static_cast<int>(expected.size())) {
    return false;
  }
  for (int index = 0; index < dimensions.nbDims; ++index) {
    if (dimensions.d[index] >= 0 &&
        dimensions.d[index] != expected[static_cast<std::size_t>(index)]) {
      return false;
    }
  }
  return true;
}

}  // namespace

class TensorRtRangeRetExecutor::Impl {
 public:
  struct Buffer {
    std::string name;
    std::vector<int64_t> shape;
    void* device_memory = nullptr;
#if NV_TENSORRT_MAJOR < 10
    int binding_index = -1;
#endif

    std::size_t ByteCount() const { return ElementCount(shape) * sizeof(float); }
  };

  ~Impl() {
    if (initialized_) {
      cudaSetDevice(device_id_);
    }
    FreeBuffers();
    if (stream_ != nullptr) {
      cudaStreamDestroy(stream_);
    }
    DestroyTensorRt(context_);
    DestroyTensorRt(engine_);
    DestroyTensorRt(runtime_);
  }

  bool Init(const RangeRetModelOptions& options) {
    if (initialized_ || options.device_id < 0 || options.engine_path.empty() ||
        options.tensor_names.input.empty() || options.tensor_names.output.empty() ||
        options.tensor_names.input == options.tensor_names.output ||
        !CudaSucceeded(cudaSetDevice(options.device_id), "cudaSetDevice")) {
      return false;
    }
    device_id_ = options.device_id;
    input_.name = options.tensor_names.input;
    output_.name = options.tensor_names.output;
    input_.shape = {1, static_cast<int64_t>(kRangeRetInputChannels),
                    static_cast<int64_t>(options.projection.height),
                    static_cast<int64_t>(options.projection.width)};
    if (options.output_layout == "NCHW") {
      output_.shape = {1, static_cast<int64_t>(options.num_classes),
                       static_cast<int64_t>(options.projection.height),
                       static_cast<int64_t>(options.projection.width)};
    } else {
      output_.shape = {1, static_cast<int64_t>(options.projection.height),
                       static_cast<int64_t>(options.projection.width),
                       static_cast<int64_t>(options.num_classes)};
    }

    std::ifstream file(options.engine_path, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
      AERROR << "Unable to open RangeRet TensorRT engine: "
             << options.engine_path;
      return false;
    }
    const std::streamsize file_size = file.tellg();
    if (file_size <= 0) {
      AERROR << "RangeRet TensorRT engine is empty: " << options.engine_path;
      return false;
    }
    std::vector<char> engine_bytes(static_cast<std::size_t>(file_size));
    file.seekg(0, std::ios::beg);
    if (!file.read(engine_bytes.data(), file_size)) {
      AERROR << "Unable to read RangeRet TensorRT engine: "
             << options.engine_path;
      return false;
    }

    initLibNvInferPlugins(&g_logger, "");
    runtime_ = nvinfer1::createInferRuntime(g_logger);
    if (runtime_ == nullptr) {
      AERROR << "Unable to create TensorRT runtime";
      return false;
    }
#if NV_TENSORRT_MAJOR >= 10
    engine_ = runtime_->deserializeCudaEngine(engine_bytes.data(),
                                              engine_bytes.size());
#else
    engine_ = runtime_->deserializeCudaEngine(engine_bytes.data(),
                                              engine_bytes.size(), nullptr);
#endif
    if (engine_ == nullptr || !ConfigureBuffers()) {
      AERROR << "TensorRT engine does not match the RangeRet I/O contract";
      return false;
    }
    context_ = engine_->createExecutionContext();
    if (context_ == nullptr ||
        !CudaSucceeded(cudaStreamCreate(&stream_), "cudaStreamCreate") ||
        !AllocateBuffers()) {
      return false;
    }
    initialized_ = true;
    return true;
  }

  bool Run(const std::vector<float>& input, RangeRetTensor* output) {
    if (!initialized_ || output == nullptr ||
        input.size() != ElementCount(input_.shape) ||
        !CudaSucceeded(cudaSetDevice(device_id_), "cudaSetDevice") ||
        !CudaSucceeded(cudaMemcpyAsync(input_.device_memory, input.data(),
                                       input_.ByteCount(),
                                       cudaMemcpyHostToDevice, stream_),
                       "cudaMemcpyAsync input")) {
      return false;
    }
#if NV_TENSORRT_MAJOR >= 10
    if (!context_->setInputShape(input_.name.c_str(), ToDims(input_.shape)) ||
        !context_->setTensorAddress(input_.name.c_str(), input_.device_memory) ||
        !context_->setTensorAddress(output_.name.c_str(),
                                    output_.device_memory) ||
        !context_->enqueueV3(stream_)) {
      AERROR << "TensorRT enqueueV3 failed";
      return false;
    }
#else
    bindings_[static_cast<std::size_t>(input_.binding_index)] =
        input_.device_memory;
    bindings_[static_cast<std::size_t>(output_.binding_index)] =
        output_.device_memory;
    if (!context_->setBindingDimensions(input_.binding_index,
                                        ToDims(input_.shape)) ||
        !context_->allInputDimensionsSpecified() ||
        !context_->enqueueV2(bindings_.data(), stream_, nullptr)) {
      AERROR << "TensorRT enqueueV2 failed";
      return false;
    }
#endif
    output->shape = output_.shape;
    output->values.resize(ElementCount(output_.shape));
    return CudaSucceeded(cudaMemcpyAsync(output->values.data(),
                                         output_.device_memory,
                                         output_.ByteCount(),
                                         cudaMemcpyDeviceToHost, stream_),
                         "cudaMemcpyAsync output") &&
           CudaSucceeded(cudaStreamSynchronize(stream_),
                         "cudaStreamSynchronize");
  }

 private:
  bool ConfigureBuffers() {
#if NV_TENSORRT_MAJOR >= 10
    return ConfigureBuffer(input_.name, input_.shape, true) &&
           ConfigureBuffer(output_.name, output_.shape, false);
#else
    bindings_.assign(static_cast<std::size_t>(engine_->getNbBindings()),
                     nullptr);
    input_.binding_index = engine_->getBindingIndex(input_.name.c_str());
    output_.binding_index = engine_->getBindingIndex(output_.name.c_str());
    if (input_.binding_index < 0 || output_.binding_index < 0 ||
        !engine_->bindingIsInput(input_.binding_index) ||
        engine_->bindingIsInput(output_.binding_index) ||
        engine_->getBindingDataType(input_.binding_index) !=
            nvinfer1::DataType::kFLOAT ||
        engine_->getBindingDataType(output_.binding_index) !=
            nvinfer1::DataType::kFLOAT ||
        !HasStaticDimensions(engine_->getBindingDimensions(input_.binding_index),
                             input_.shape) ||
        !HasStaticDimensions(engine_->getBindingDimensions(output_.binding_index),
                             output_.shape)) {
      return false;
    }
    return true;
#endif
  }

#if NV_TENSORRT_MAJOR >= 10
  bool ConfigureBuffer(const std::string& name,
                       const std::vector<int64_t>& shape, bool expect_input) {
    for (int index = 0; index < engine_->getNbIOTensors(); ++index) {
      const char* tensor_name = engine_->getIOTensorName(index);
      if (tensor_name == nullptr || name != tensor_name) {
        continue;
      }
      const nvinfer1::TensorIOMode mode =
          engine_->getTensorIOMode(name.c_str());
      return ((expect_input && mode == nvinfer1::TensorIOMode::kINPUT) ||
              (!expect_input && mode == nvinfer1::TensorIOMode::kOUTPUT)) &&
             engine_->getTensorDataType(name.c_str()) ==
                 nvinfer1::DataType::kFLOAT &&
             HasStaticDimensions(engine_->getTensorShape(name.c_str()), shape);
    }
    return false;
  }
#endif

  bool AllocateBuffers() {
    return CudaSucceeded(cudaMalloc(&input_.device_memory, input_.ByteCount()),
                         "cudaMalloc input") &&
           CudaSucceeded(cudaMalloc(&output_.device_memory, output_.ByteCount()),
                         "cudaMalloc output");
  }

  void FreeBuffers() {
    if (input_.device_memory != nullptr) {
      cudaFree(input_.device_memory);
      input_.device_memory = nullptr;
    }
    if (output_.device_memory != nullptr) {
      cudaFree(output_.device_memory);
      output_.device_memory = nullptr;
    }
  }

  int device_id_ = 0;
  nvinfer1::IRuntime* runtime_ = nullptr;
  nvinfer1::ICudaEngine* engine_ = nullptr;
  nvinfer1::IExecutionContext* context_ = nullptr;
  cudaStream_t stream_ = nullptr;
  Buffer input_;
  Buffer output_;
#if NV_TENSORRT_MAJOR < 10
  std::vector<void*> bindings_;
#endif
  bool initialized_ = false;
};

TensorRtRangeRetExecutor::TensorRtRangeRetExecutor() : impl_(new Impl) {}

TensorRtRangeRetExecutor::~TensorRtRangeRetExecutor() = default;

bool TensorRtRangeRetExecutor::Init(const RangeRetModelOptions& options) {
  return impl_->Init(options);
}

bool TensorRtRangeRetExecutor::Run(const std::vector<float>& input,
                                   RangeRetTensor* output) {
  return impl_->Run(input, output);
}

}  // namespace lidar_semantic_segmentation
}  // namespace apollo
