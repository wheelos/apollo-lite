// Copyright 2026 WheelOS. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "modules/lidar_semantic_segmentation/inference/cylinder3d.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <ATen/ATen.h>
#include <c10/core/impl/VirtualGuardImpl.h>
#include <torch/script.h>

#include "nlohmann/json.hpp"

#include "modules/lidar_semantic_segmentation/inference/cylinder_batch_norm.h"
#include "modules/lidar_semantic_segmentation/inference/cylinder_sparse.h"

namespace apollo {
namespace lidar_semantic_segmentation {
namespace {

constexpr const char* kBackbone = "cylinder_3d_spconv_seg.";
constexpr const char* kGenerator = "cylinder_3d_generator.";

class StageProfile {
 public:
  explicit StageProfile(at::Device device = at::Device(at::kCPU))
      : device_(device),
        enabled_(std::getenv("CYLINDER3D_PROFILE_STAGES") != nullptr) {
    if (enabled_) {
      Synchronize();
      start_ = std::chrono::steady_clock::now();
    }
  }

  void Mark(const char* stage) {
    if (enabled_) {
      Synchronize();
      const auto now = std::chrono::steady_clock::now();
      std::fprintf(
          stderr, "CYLINDER_STAGE %s ms=%.3f\n", stage,
          std::chrono::duration<double, std::milli>(now - start_).count());
      start_ = now;
    }
  }

 private:
  void Synchronize() const {
    if (device_.is_cuda()) {
      c10::impl::VirtualGuardImpl(device_.type())
          .synchronizeDevice(device_.index());
    }
  }

  at::Device device_;
  bool enabled_;
  std::chrono::steady_clock::time_point start_;
};

bool Fail(const std::string& message, std::string* error) {
  if (error != nullptr) {
    *error = message;
  }
  return false;
}

}  // namespace

class Cylinder3DExecutor::Impl {
 public:
  void Init(const Cylinder3DOptions& options,
            CylinderProjectionOptions* projection) {
    TORCH_CHECK(
        projection != nullptr && !options.model_path.empty() &&
            options.device_id >= -1 &&
            options.device_id <= std::numeric_limits<c10::DeviceIndex>::max(),
        "invalid Cylinder3D model path, output or device");
    torch::jit::ExtraFilesMap extra{{"cylinder3d.json", ""}};
    const auto device = options.device_id < 0
                            ? at::Device(at::kCPU)
                            : at::Device(at::kCUDA, options.device_id);
    auto archive = torch::jit::load(options.model_path, at::kCPU, extra);
    auto metadata = nlohmann::json::parse(extra.at("cylinder3d.json"));
    const auto semantics = metadata.value("indice_semantics", "independent");
    TORCH_CHECK(semantics == "independent" || semantics == "spconv1-shared",
                "unsupported Cylinder3D indice semantics");
    legacy_indices_ = semantics == "spconv1-shared";
    constexpr std::array<int, 20> kSemanticKittiIds = {
        0,  10, 11, 15, 18, 20, 30, 31, 32, 40,
        44, 48, 49, 50, 51, 70, 71, 72, 80, 81};
    const auto class_ids = metadata.at("class_ids").get<std::array<int, 20>>();
    TORCH_CHECK(metadata.at("format") == "apollo-cylinder3d-v1" &&
                    class_ids == kSemanticKittiIds &&
                    metadata.at("batch_norm_eps") == 1.0e-5 &&
                    metadata.at("leaky_relu_slope") == 0.01,
                "unsupported Cylinder3D artifact format or class mapping");
    const auto& config = metadata.at("model");
    const int width = config.at("init_size");
    const int input_channels = config.at("num_input_features");
    const int point_channels = config.at("out_fea_dim");
    TORCH_CHECK(width > 0 && width <= 128 && input_channels > 0 &&
                    input_channels <= 256 && point_channels > 0 &&
                    point_channels <= 1024 && config.at("num_class") == 20 &&
                    config.at("fea_dim") == 9 &&
                    config.at("model_architecture") == "cylinder_asym",
                "unsupported Cylinder3D network dimensions");
    projection->grid = config.at("output_shape").get<CylinderCoordinate>();
    projection->minimum = metadata.at("minimum").get<std::array<double, 3>>();
    projection->maximum = metadata.at("maximum").get<std::array<double, 3>>();
    projection->max_points = options.max_points;
    projection->intensity_scale = options.intensity_scale;
    CylinderProjector validator;
    std::string error;
    TORCH_CHECK(validator.Init(*projection, &error), error);
    shape_ = projection->grid;
    max_points_ = options.max_points;
    for (const auto& entry : metadata.at("buffers").items()) {
      auto tensor = archive.attr(entry.value().get<std::string>()).toTensor();
      TORCH_CHECK(tensor.scalar_type() == at::kFloat &&
                      at::isfinite(tensor).all().item<bool>(),
                  "Cylinder3D weights must be finite FP32: ", entry.key());
      TORCH_CHECK(weights_.emplace(entry.key(), tensor.to(device)).second,
                  "duplicate Cylinder3D parameter");
    }
    ValidateWeights(width, input_channels, point_channels);
    device_ = device;
    TORCH_CHECK(!options.use_spconv ||
                    (device.is_cuda() && cylinder::SpconvAvailable()),
                "Cylinder3D spconv requires CUDA and "
                "--define=CYLINDER3D_SPCONV=true");
    use_spconv_ = options.use_spconv;
  }

  at::Tensor Run(const CylinderInput& input) {
    StageProfile profile(device_);
    TORCH_CHECK(!input.coordinates.empty() &&
                    input.coordinates.size() <= max_points_ &&
                    input.coordinates.size() == input.point_indices.size() &&
                    input.features.size() == input.coordinates.size() * 9,
                "invalid Cylinder3D projected input");
    const auto count = static_cast<int64_t>(input.coordinates.size());
    auto features =
        at::from_blob(const_cast<float*>(input.features.data()), {count, 9},
                      at::TensorOptions().dtype(at::kFloat))
            .to(device_);
    TORCH_CHECK(at::isfinite(features).all().item<bool>(),
                "Cylinder3D input features are non-finite");
    profile.Mark("upload");
    const std::string pp = std::string(kGenerator) + "PPmodel.";
    features = BatchNorm(features, pp + "0");
    for (int layer : {1, 4, 7}) {
      features = BatchNorm(Linear(features, pp + std::to_string(layer)),
                           pp + std::to_string(layer + 1),
                           cylinder::BatchNormActivation::kReluAfter);
    }
    features = Linear(features, pp + "10");
    profile.Mark("point_mlp");
    std::vector<std::pair<int64_t, int64_t>> ordered;
    ordered.reserve(count);
    for (int64_t row = 0; row < count; ++row) {
      const auto& coordinate = input.coordinates[row];
      for (int axis = 0; axis < 3; ++axis) {
        TORCH_CHECK(coordinate[axis] >= 0 && coordinate[axis] < shape_[axis],
                    "Cylinder3D projected coordinate is outside the grid");
      }
      const auto key = (coordinate[0] * shape_[1] + coordinate[1]) * shape_[2] +
                       coordinate[2];
      ordered.emplace_back(key, row);
    }
    std::sort(ordered.begin(), ordered.end());
    auto coordinates = std::make_shared<cylinder::Coordinates>();
    coordinates->reserve(count);
    std::vector<int64_t> inverse(count);
    int64_t previous = -1;
    for (const auto& entry : ordered) {
      if (entry.first != previous) {
        coordinates->push_back(input.coordinates[entry.second]);
        previous = entry.first;
      }
      inverse[entry.second] = static_cast<int64_t>(coordinates->size()) - 1;
    }
    profile.Mark("unique_cpu");
    auto indices =
        at::tensor(inverse, at::TensorOptions().dtype(at::kLong)).to(device_);
    auto pooled =
        at::full({static_cast<int64_t>(coordinates->size()), features.size(1)},
                 -std::numeric_limits<float>::infinity(), features.options());
    pooled.scatter_reduce_(0, indices.unsqueeze(1).expand_as(features),
                           features, "amax");
    pooled =
        at::relu(Linear(pooled, std::string(kGenerator) + "fea_compression.0"));
    profile.Mark("pool");
    cylinder::SparseIndiceCache cache;
    cylinder::SparseTensor x{pooled, coordinates, shape_, use_spconv_};
    x = Residual(x, "downCntx", &cache);
    std::array<cylinder::SparseTensor, 4> skip;
    std::array<cylinder::SparseRelation, 4> relations;
    for (int level = 0; level < 4; ++level) {
      const auto name = "resBlock" + std::to_string(level + 2);
      skip[level] = Residual(x, name, &cache);
      x = Conv(
          skip[level], name + ".pool", &cache,
          level < 2 ? CylinderCoordinate{2, 2, 2} : CylinderCoordinate{2, 2, 1},
          &relations[level]);
    }
    profile.Mark("encoder");
    for (int level = 0; level < 4; ++level) {
      const auto name = "upBlock" + std::to_string(level);
      x = ActivatedConv(x, name + ".trans_dilao", name + ".trans_bn", &cache);
      x = cylinder::InverseConvolve(
          x, Weight(std::string(kBackbone) + name + ".up_subm.weight"),
          relations[3 - level]);
      TORCH_CHECK(x.coordinates == skip[3 - level].coordinates,
                  "Cylinder3D skip coordinates do not match inverse output");
      x.features += skip[3 - level].features;
      for (int layer = 1; layer <= 3; ++layer) {
        x = ActivatedConv(x, name + ".conv" + std::to_string(layer),
                          name + ".bn" + std::to_string(layer), &cache);
      }
    }
    profile.Mark("decoder");
    at::Tensor gate;
    for (const auto& names :
         {std::pair<const char*, const char*>{"conv1", "bn0"},
          {"conv1_2", "bn0_2"},
          {"conv1_3", "bn0_3"}}) {
      auto branch = Conv(x, std::string("ReconNet.") + names.first, &cache);
      auto values = BatchNorm(
          branch.features, std::string(kBackbone) + "ReconNet." + names.second,
          cylinder::BatchNormActivation::kSigmoidAfter);
      gate = gate.defined() ? gate + values : values;
    }
    x.features = at::cat({gate * x.features, x.features}, 1);
    x = Conv(x, "logits", &cache);
    TORCH_CHECK(x.coordinates == coordinates,
                "Cylinder3D backbone changed original coordinate ordering");
    profile.Mark("reconstruction");
    auto output = x.features.index_select(0, indices);
    profile.Mark("point_gather");
    return output;
  }

 private:
  const at::Tensor& Weight(const std::string& name) const {
    const auto found = weights_.find(name);
    TORCH_CHECK(found != weights_.end(),
                "missing Cylinder3D parameter: ", name);
    return found->second;
  }

  void Require(const std::string& name, const std::vector<int64_t>& shape) {
    TORCH_CHECK(Weight(name).sizes().vec() == shape,
                "Cylinder3D parameter shape mismatch: ", name);
    expected_.insert(name);
  }

  void RequireBN(const std::string& name, int channels) {
    for (const char* suffix :
         {".weight", ".bias", ".running_mean", ".running_var"}) {
      Require(name + suffix, {channels});
    }
    TORCH_CHECK((Weight(name + ".running_var") >= 0).all().item<bool>(),
                "Cylinder3D batch normalization has negative variance");
    bn_scales_.emplace(name,
                       Weight(name + ".weight") /
                           at::sqrt(Weight(name + ".running_var") + 1.0e-5));
  }

  void RequireLinear(const std::string& name, int in, int out) {
    Require(name + ".weight", {out, in});
    Require(name + ".bias", {out});
  }

  void RequireConv(const std::string& name, int in, int out,
                   const CylinderCoordinate& kernel, bool bias = false) {
    Require(std::string(kBackbone) + name + ".weight",
            {kernel[0], kernel[1], kernel[2], in, out});
    if (bias) {
      Require(std::string(kBackbone) + name + ".bias", {out});
    }
  }

  void RequireResidual(const std::string& name, int in, int out, bool context) {
    const CylinderCoordinate short_kernel =
        context ? CylinderCoordinate{1, 3, 3} : CylinderCoordinate{3, 1, 3};
    const CylinderCoordinate long_kernel =
        context ? CylinderCoordinate{3, 1, 3} : CylinderCoordinate{1, 3, 3};
    RequireConv(name + ".conv1", in, out, short_kernel);
    RequireConv(name + ".conv1_2", out, out,
                legacy_indices_ ? short_kernel : long_kernel);
    RequireConv(name + ".conv2", in, out,
                legacy_indices_ ? short_kernel : long_kernel);
    RequireConv(name + ".conv3", out, out, short_kernel);
    for (const char* bn : {"bn0", "bn0_2", "bn1", "bn2"}) {
      RequireBN(std::string(kBackbone) + name + "." + bn, out);
    }
  }

  void ValidateWeights(int width, int input_channels, int point_channels) {
    const std::string pp = std::string(kGenerator) + "PPmodel.";
    RequireBN(pp + "0", 9);
    int in = 9;
    for (const auto& layer : {std::pair<int, int>{1, 64}, {4, 128}, {7, 256}}) {
      RequireLinear(pp + std::to_string(layer.first), in, layer.second);
      RequireBN(pp + std::to_string(layer.first + 1), layer.second);
      in = layer.second;
    }
    RequireLinear(pp + "10", 256, point_channels);
    RequireLinear(std::string(kGenerator) + "fea_compression.0", point_channels,
                  input_channels);
    RequireResidual("downCntx", input_channels, width, true);
    for (int level = 0; level < 4; ++level) {
      const int out = width * (2 << level);
      const auto name = "resBlock" + std::to_string(level + 2);
      RequireResidual(name, out / 2, out, false);
      RequireConv(name + ".pool", out, out, {3, 3, 3});
    }
    in = width * 16;
    for (int level = 0; level < 4; ++level) {
      const int out = width * (16 >> level);
      const auto name = "upBlock" + std::to_string(level);
      RequireConv(name + ".trans_dilao", in, out, {3, 3, 3});
      RequireBN(std::string(kBackbone) + name + ".trans_bn", out);
      RequireConv(name + ".up_subm", out, out, {3, 3, 3});
      for (int layer = 1; layer <= 3; ++layer) {
        RequireConv(name + ".conv" + std::to_string(layer), out, out,
                    layer == 1 || legacy_indices_ ? CylinderCoordinate{1, 3, 3}
                    : layer == 2                  ? CylinderCoordinate{3, 1, 3}
                                 : CylinderCoordinate{3, 3, 3});
        RequireBN(std::string(kBackbone) + name + ".bn" + std::to_string(layer),
                  out);
      }
      in = out;
    }
    RequireConv("ReconNet.conv1", in, in, {3, 1, 1});
    RequireConv("ReconNet.conv1_2", in, in,
                legacy_indices_ ? CylinderCoordinate{3, 1, 1}
                                : CylinderCoordinate{1, 3, 1});
    RequireConv("ReconNet.conv1_3", in, in,
                legacy_indices_ ? CylinderCoordinate{3, 1, 1}
                                : CylinderCoordinate{1, 1, 3});
    for (const char* bn : {"bn0", "bn0_2", "bn0_3"}) {
      RequireBN(std::string(kBackbone) + "ReconNet." + bn, in);
    }
    RequireConv("logits", 2 * in, 20, {3, 3, 3}, true);
    TORCH_CHECK(expected_.size() == weights_.size(),
                "unexpected Cylinder3D parameters; refusing partial loading");
  }

  at::Tensor Linear(const at::Tensor& input, const std::string& name) const {
    return at::linear(input, Weight(name + ".weight"), Weight(name + ".bias"));
  }

  at::Tensor BatchNorm(const at::Tensor& input, const std::string& name,
                       cylinder::BatchNormActivation activation =
                           cylinder::BatchNormActivation::kNone) const {
    return cylinder::BatchNorm(input, Weight(name + ".running_mean"),
                               bn_scales_.at(name), Weight(name + ".bias"),
                               activation);
  }

  cylinder::SparseTensor Conv(
      const cylinder::SparseTensor& input, const std::string& name,
      cylinder::SparseIndiceCache* cache,
      const CylinderCoordinate& stride = {1, 1, 1},
      cylinder::SparseRelation* relation = nullptr) const {
    const auto& weight = Weight(std::string(kBackbone) + name + ".weight");
    CylinderCoordinate padding{};
    for (int axis = 0; axis < 3; ++axis) {
      padding[axis] = weight.size(axis) / 2;
    }
    return cylinder::Convolve(
        input, weight,
        name == "logits" ? Weight(std::string(kBackbone) + "logits.bias")
                         : at::Tensor(),
        stride, padding, relation == nullptr, relation, cache);
  }

  cylinder::SparseTensor ActivatedConv(
      const cylinder::SparseTensor& input, const std::string& conv,
      const std::string& bn, cylinder::SparseIndiceCache* cache) const {
    auto result = Conv(input, conv, cache);
    result.features = BatchNorm(result.features, std::string(kBackbone) + bn,
                                cylinder::BatchNormActivation::kLeakyBefore);
    return result;
  }

  cylinder::SparseTensor Residual(const cylinder::SparseTensor& input,
                                  const std::string& name,
                                  cylinder::SparseIndiceCache* cache) const {
    auto shortcut = ActivatedConv(input, name + ".conv1", name + ".bn0", cache);
    shortcut =
        ActivatedConv(shortcut, name + ".conv1_2", name + ".bn0_2", cache);
    auto result = ActivatedConv(input, name + ".conv2", name + ".bn1", cache);
    result = ActivatedConv(result, name + ".conv3", name + ".bn2", cache);
    result.features += shortcut.features;
    return result;
  }

  std::map<std::string, at::Tensor> weights_;
  std::map<std::string, at::Tensor> bn_scales_;
  std::set<std::string> expected_;
  at::Device device_{at::kCPU};
  CylinderCoordinate shape_{};
  std::size_t max_points_ = 0;
  bool legacy_indices_ = false;
  bool use_spconv_ = false;
};

Cylinder3DExecutor::Cylinder3DExecutor() = default;
Cylinder3DExecutor::~Cylinder3DExecutor() = default;

bool Cylinder3DExecutor::Init(const Cylinder3DOptions& options,
                              CylinderProjectionOptions* projection,
                              std::string* error) {
  impl_.reset();
  try {
    at::NoGradGuard guard;
    auto impl = std::make_unique<Impl>();
    impl->Init(options, projection);
    impl_ = std::move(impl);
    return true;
  } catch (const std::exception& exception) {
    return Fail(exception.what(), error);
  }
}

bool Cylinder3DExecutor::Run(const CylinderInput& input,
                             std::vector<float>* point_logits,
                             std::string* error) {
  if (!impl_ || point_logits == nullptr) {
    return Fail("Cylinder3D executor is not initialized or output is null",
                error);
  }
  try {
    at::NoGradGuard guard;
    if (input.coordinates.empty()) {
      TORCH_CHECK(
          input.features.empty() && input.point_indices.empty(),
          "empty Cylinder3D coordinates have inconsistent feature rows");
      point_logits->clear();
      return true;
    }
    auto output = impl_->Run(input);
    StageProfile profile(output.device());
    TORCH_CHECK(
        output.dim() == 2 &&
            output.size(0) == static_cast<int64_t>(input.coordinates.size()) &&
            output.size(1) == 20 && at::isfinite(output).all().item<bool>(),
        "Cylinder3D returned invalid or non-finite point logits");
    output = output.to(at::kCPU).contiguous();
    point_logits->assign(output.data_ptr<float>(),
                         output.data_ptr<float>() + output.numel());
    profile.Mark("logits_readback");
    return true;
  } catch (const std::exception& exception) {
    return Fail(exception.what(), error);
  }
}

bool Cylinder3DExecutor::RunLabels(const std::vector<CylinderInput>& votes,
                                   std::vector<uint32_t>* labels,
                                   std::vector<float>* confidences,
                                   std::string* error) {
  if (!impl_ || labels == nullptr || confidences == nullptr || votes.empty() ||
      (votes.size() != 1 && votes.size() != 4)) {
    return Fail(
        "Cylinder3D decoding requires an initialized executor, "
        "one or four votes and non-null outputs",
        error);
  }
  try {
    at::NoGradGuard guard;
    const auto count = static_cast<int64_t>(votes.front().coordinates.size());
    at::Tensor fused, finite;
    for (const auto& input : votes) {
      TORCH_CHECK(
          input.point_indices == votes.front().point_indices &&
              input.coordinates.size() == static_cast<std::size_t>(count),
          "Cylinder3D flip voting changed point correspondence");
      if (count == 0) {
        TORCH_CHECK(input.features.empty() && input.point_indices.empty(),
                    "empty Cylinder3D coordinates have inconsistent rows");
        continue;
      }
      auto logits = impl_->Run(input);
      TORCH_CHECK(
          logits.dim() == 2 && logits.size(0) == count && logits.size(1) == 20,
          "Cylinder3D returned invalid point logits");
      const auto valid = at::isfinite(logits).all();
      finite = finite.defined() ? finite.logical_and(valid) : valid;
      // Match the reference's double-precision vote sum and stable softmax.
      if (fused.defined()) {
        fused.add_(logits);
      } else {
        fused = logits.to(at::kDouble);
      }
    }
    if (count == 0) {
      labels->clear();
      confidences->clear();
      return true;
    }
    StageProfile profile(fused.device());
    TORCH_CHECK(finite.item<bool>(),
                "Cylinder3D returned non-finite point logits");
    const auto maximum = fused.max(1);
    auto scores = (fused - std::get<0>(maximum).unsqueeze(1))
                      .exp()
                      .sum(1)
                      .reciprocal()
                      .to(at::kFloat);
    auto indices = std::get<1>(maximum).to(at::kInt);
    profile.Mark("device_decode");
    indices = indices.cpu().contiguous();
    scores = scores.cpu().contiguous();
    std::vector<uint32_t> decoded(indices.data_ptr<int32_t>(),
                                  indices.data_ptr<int32_t>() + count);
    std::vector<float> confidence(scores.data_ptr<float>(),
                                  scores.data_ptr<float>() + count);
    profile.Mark("labels_readback");
    labels->swap(decoded);
    confidences->swap(confidence);
    return true;
  } catch (const std::exception& exception) {
    return Fail(exception.what(), error);
  }
}

bool Cylinder3DSegmenter::Init(const Cylinder3DOptions& options,
                               std::string* error) {
  initialized_ = false;
  CylinderProjectionOptions projection;
  if (!executor_.Init(options, &projection, error) ||
      !projector_.Init(projection, error)) {
    return false;
  }
  options_ = options;
  initialized_ = true;
  return true;
}

bool Cylinder3DSegmenter::Segment(const apollo::drivers::PointCloud& cloud,
                                  LidarSemanticSegmentationResult* result,
                                  std::string* error) {
  if (!initialized_ || result == nullptr) {
    return Fail("Cylinder3D segmenter is not initialized or output is null",
                error);
  }
  CylinderInput input;
  StageProfile profile;
  if (!projector_.Project(cloud, 0, &input, error)) {
    return false;
  }
  const auto point_indices = input.point_indices;
  profile.Mark("projection");
  std::vector<uint32_t> labels;
  std::vector<float> confidences;
  if (options_.device_id >= 0) {
    std::vector<CylinderInput> votes(options_.flip_tta ? 4 : 1);
    votes[0] = std::move(input);
    for (std::size_t vote = 1; vote < votes.size(); ++vote) {
      if (!projector_.Project(cloud, vote, &votes[vote], error)) {
        return false;
      }
    }
    if (!executor_.RunLabels(votes, &labels, &confidences, error)) {
      return false;
    }
  } else {
    std::vector<double> fused(point_indices.size() * 20, 0.0);
    for (int vote = 0; vote < (options_.flip_tta ? 4 : 1); ++vote) {
      if (vote != 0 && !projector_.Project(cloud, vote, &input, error)) {
        return false;
      }
      std::vector<float> logits;
      if (!executor_.Run(input, &logits, error)) {
        return false;
      }
      if (logits.size() != fused.size() ||
          input.point_indices != point_indices) {
        return Fail("Cylinder3D flip voting changed point correspondence",
                    error);
      }
      for (std::size_t index = 0; index < fused.size(); ++index) {
        fused[index] += logits[index];
      }
    }
    labels.reserve(point_indices.size());
    confidences.reserve(point_indices.size());
    for (std::size_t row = 0; row < point_indices.size(); ++row) {
      const auto begin = fused.begin() + row * 20;
      const auto best = std::max_element(begin, begin + 20);
      double sum = 0;
      for (auto it = begin; it != begin + 20; ++it) {
        sum += std::exp(*it - *best);
      }
      labels.push_back(static_cast<uint32_t>(best - begin));
      confidences.push_back(static_cast<float>(1.0 / sum));
    }
  }
  profile.Mark("votes");
  LidarSemanticSegmentationResult output;
  if (cloud.has_header()) {
    *output.mutable_header() = cloud.header();
  }
  output.set_source_topic(options_.source_topic);
  output.set_source_frame_id(cloud.frame_id());
  output.set_sensor_name(options_.sensor_name);
  output.set_point_count(cloud.point_size());
  output.set_num_classes(20);
  for (int index = 0; index < cloud.point_size(); ++index) {
    auto* label = output.add_point_label();
    label->set_point_index(index);
    label->set_semantic_label(0);
    label->set_confidence(0.0F);
  }
  for (std::size_t row = 0; row < point_indices.size(); ++row) {
    auto* label = output.mutable_point_label(point_indices[row]);
    label->set_semantic_label(labels[row]);
    label->set_confidence(confidences[row]);
  }
  result->Swap(&output);
  profile.Mark("decode_result");
  return true;
}

}  // namespace lidar_semantic_segmentation
}  // namespace apollo
