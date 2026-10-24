// Copyright 2026 WheelOS. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "modules/bevfusion/inference/bevfusion_runner.h"
#include "modules/bevfusion/types/bevfusion_input.h"

namespace {

using Json = nlohmann::json;
using apollo::bevfusion::BevFusionInputView;
using apollo::bevfusion::BevFusionModelConfig;
using apollo::bevfusion::CameraImageView;
using apollo::bevfusion::InputLimits;
using apollo::bevfusion::PixelFormat;

constexpr const char* kUpstreamRevision =
    "7c1623f234d5a9794fd862c9456f68f87ae7b900";

bool SetError(const std::string& message, std::string* error) {
  if (error != nullptr) {
    *error = message;
  }
  return false;
}

std::filesystem::path ResolvePath(const std::filesystem::path& base,
                                  const std::string& path) {
  const std::filesystem::path value(path);
  return value.is_absolute() ? value : base / value;
}

bool ReadJson(const std::filesystem::path& path, Json* json,
              std::string* error) {
  std::ifstream input(path);
  if (!input) {
    return SetError("cannot open JSON file: " + path.string(), error);
  }
  try {
    input >> *json;
  } catch (const std::exception& exception) {
    return SetError(
        "invalid JSON in " + path.string() + ": " + exception.what(), error);
  }
  return true;
}

bool ReadExactBytes(const std::filesystem::path& path, std::size_t expected,
                    std::vector<std::uint8_t>* data, std::string* error) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    return SetError("cannot open input file: " + path.string(), error);
  }
  input.seekg(0, std::ios::end);
  const auto end = input.tellg();
  if (end < 0 || static_cast<std::uint64_t>(end) != expected) {
    return SetError("input file has the wrong byte count: " + path.string(),
                    error);
  }
  input.seekg(0, std::ios::beg);
  data->resize(expected);
  if (expected != 0 && !input.read(reinterpret_cast<char*>(data->data()),
                                   static_cast<std::streamsize>(expected))) {
    return SetError("failed to read complete input file: " + path.string(),
                    error);
  }
  return true;
}

bool ReadFloat32Points(const std::filesystem::path& path,
                       std::size_t point_count, std::vector<float>* points,
                       std::string* error) {
  if (point_count > std::numeric_limits<std::size_t>::max() /
                        apollo::bevfusion::kPointFeatureCount / sizeof(float)) {
    return SetError("LiDAR point count overflows packed input size", error);
  }
  const std::size_t byte_count =
      point_count * apollo::bevfusion::kPointFeatureCount * sizeof(float);
  std::vector<std::uint8_t> bytes;
  if (!ReadExactBytes(path, byte_count, &bytes, error)) {
    return false;
  }
#if defined(__BYTE_ORDER__) && defined(__ORDER_BIG_ENDIAN__) && \
    __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
  return SetError("offline float32 point files require a little-endian host",
                  error);
#endif
  points->resize(point_count * apollo::bevfusion::kPointFeatureCount);
  std::memcpy(points->data(), bytes.data(), byte_count);
  return true;
}

bool ReadMatrix(const Json& json, const char* key,
                std::array<float, 96>* matrix, std::string* error) {
  const auto iter = json.find(key);
  if (iter == json.end() || !iter->is_array() ||
      iter->size() != matrix->size()) {
    return SetError(std::string("calibration.") + key +
                        " must contain exactly 96 row-major float values",
                    error);
  }
  for (std::size_t index = 0; index < matrix->size(); ++index) {
    if (!(*iter)[index].is_number()) {
      return SetError(
          std::string("calibration.") + key + " contains a non-numeric value",
          error);
    }
    (*matrix)[index] = (*iter)[index].get<float>();
  }
  return true;
}

bool LoadModelConfig(const std::filesystem::path& path,
                     BevFusionModelConfig* config, std::string* error) {
  Json json;
  if (!ReadJson(path, &json, error) || !json.is_object()) {
    return error != nullptr && !error->empty()
               ? false
               : SetError("model config must be a JSON object", error);
  }
  const std::array<const char*, 7> paths = {
      "runtime_library_path",    "camera_backbone_engine_path",
      "lidar_scn_onnx_path",     "camera_vtransform_engine_path",
      "transfusion_engine_path", "bbox_engine_path",
      "model_revision"};
  for (const char* key : paths) {
    if (!json.contains(key) || !json[key].is_string() ||
        json[key].get<std::string>().empty()) {
      return SetError(std::string("model config field must be set: ") + key,
                      error);
    }
  }
  try {
    const auto base = path.parent_path();
    config->set_runtime_library_path(
        ResolvePath(base, json["runtime_library_path"].get<std::string>())
            .string());
    config->set_camera_backbone_engine_path(
        ResolvePath(base,
                    json["camera_backbone_engine_path"].get<std::string>())
            .string());
    config->set_lidar_scn_onnx_path(
        ResolvePath(base, json["lidar_scn_onnx_path"].get<std::string>())
            .string());
    config->set_camera_vtransform_engine_path(
        ResolvePath(base,
                    json["camera_vtransform_engine_path"].get<std::string>())
            .string());
    config->set_transfusion_engine_path(
        ResolvePath(base, json["transfusion_engine_path"].get<std::string>())
            .string());
    config->set_bbox_engine_path(
        ResolvePath(base, json["bbox_engine_path"].get<std::string>())
            .string());
    config->set_model_revision(json["model_revision"].get<std::string>());
    config->set_gpu_device_id(json.at("gpu_device_id").get<int>());
    config->set_confidence_threshold(
        json.at("confidence_threshold").get<float>());
    config->set_max_input_point_count(
        json.at("max_input_point_count").get<std::uint32_t>());
  } catch (const std::exception& exception) {
    return SetError(
        std::string("invalid model config field: ") + exception.what(), error);
  }
  return true;
}

bool LoadFrame(const Json& frame, const std::filesystem::path& manifest_base,
               std::size_t max_points, BevFusionInputView* input,
               std::vector<std::vector<std::uint8_t>>* image_storage,
               std::vector<float>* point_storage,
               std::string* calibration_revision, std::string* error) {
  try {
    input->timestamp_ns = frame.at("timestamp_ns").get<std::uint64_t>();
    const auto& lidar = frame.at("lidar");
    const std::uint64_t lidar_timestamp =
        lidar.at("timestamp_ns").get<std::uint64_t>();
    if (lidar_timestamp != input->timestamp_ns) {
      return SetError(
          "frame timestamp must equal the LiDAR reference "
          "timestamp",
          error);
    }
    input->point_count = lidar.at("point_count").get<std::size_t>();
    input->point_feature_count = lidar.at("feature_count").get<std::size_t>();
    if (input->point_count == 0 || input->point_count > max_points) {
      return SetError("LiDAR point count is empty or exceeds configured limit",
                      error);
    }
    if (input->point_feature_count != apollo::bevfusion::kPointFeatureCount) {
      return SetError("LiDAR feature_count must be exactly five", error);
    }
    const auto point_path =
        ResolvePath(manifest_base, lidar.at("path").get<std::string>());
    if (!ReadFloat32Points(point_path, input->point_count, point_storage,
                           error)) {
      return false;
    }
    input->lidar_points = point_storage->data();

    const auto& cameras = frame.at("cameras");
    if (!cameras.is_array() ||
        cameras.size() != apollo::bevfusion::kCameraCount) {
      return SetError("each offline frame must contain exactly six cameras",
                      error);
    }
    image_storage->resize(apollo::bevfusion::kCameraCount);
    for (std::size_t camera = 0; camera < apollo::bevfusion::kCameraCount;
         ++camera) {
      const auto& item = cameras[camera];
      const std::string name = item.at("name").get<std::string>();
      const std::uint64_t timestamp =
          item.at("timestamp_ns").get<std::uint64_t>();
      const std::string format = item.at("format").get<std::string>();
      const std::uint32_t width = item.at("width").get<std::uint32_t>();
      const std::uint32_t height = item.at("height").get<std::uint32_t>();
      const std::size_t stride = item.at("stride_bytes").get<std::size_t>();
      if (format != "RGB8_HWC") {
        return SetError("camera format must be RGB8_HWC", error);
      }
      const auto image_path =
          ResolvePath(manifest_base, item.at("path").get<std::string>());
      if (!ReadExactBytes(image_path, apollo::bevfusion::kImageByteCount,
                          &(*image_storage)[camera], error)) {
        return false;
      }
      auto& image = input->cameras[camera];
      image.camera_name = apollo::bevfusion::kCameraOrder[camera];
      if (name != image.camera_name) {
        return SetError(
            "camera manifest order must be FRONT, FRONT_RIGHT, "
            "FRONT_LEFT, BACK, BACK_LEFT, BACK_RIGHT",
            error);
      }
      image.timestamp_ns = timestamp;
      image.width = width;
      image.height = height;
      image.stride_bytes = stride;
      image.pixel_format = PixelFormat::kRgb8Hwc;
      image.data = (*image_storage)[camera].data();
      image.byte_count = (*image_storage)[camera].size();
    }

    const auto& calibration = frame.at("calibration");
    *calibration_revision = calibration.at("revision").get<std::string>();
    if (calibration_revision->empty()) {
      return SetError("calibration.revision must be nonempty", error);
    }
    if (!ReadMatrix(calibration, "camera_to_lidar", &input->camera_to_lidar,
                    error) ||
        !ReadMatrix(calibration, "camera_intrinsics", &input->camera_intrinsics,
                    error) ||
        !ReadMatrix(calibration, "lidar_to_image", &input->lidar_to_image,
                    error) ||
        !ReadMatrix(calibration, "image_augmentation",
                    &input->image_augmentation, error)) {
      return false;
    }
  } catch (const std::exception& exception) {
    return SetError(
        std::string("invalid offline frame manifest: ") + exception.what(),
        error);
  }
  return true;
}

Json MakeResultJson(const apollo::bevfusion::BevFusionInferenceResult& result,
                    const BevFusionModelConfig& config,
                    const std::string& calibration_revision,
                    std::size_t frame_index, double frame_total_ms) {
  Json output;
  output["frame_index"] = frame_index;
  output["timestamp_ns"] = result.timestamp_ns;
  output["input_point_count"] = result.input_point_count;
  output["inference_ms"] = result.inference_ms;
  output["frame_total_ms"] = frame_total_ms;
  output["upstream_revision"] = kUpstreamRevision;
  output["model_variant"] = "ResNet50_FP16";
  output["model_revision"] = config.model_revision();
  output["calibration_revision"] = calibration_revision;
  output["model_artifacts"] = {
      {"runtime_library", config.runtime_library_path()},
      {"camera_backbone_engine", config.camera_backbone_engine_path()},
      {"lidar_scn_onnx", config.lidar_scn_onnx_path()},
      {"camera_vtransform_engine", config.camera_vtransform_engine_path()},
      {"transfusion_engine", config.transfusion_engine_path()},
      {"bbox_engine", config.bbox_engine_path()}};
  output["box_semantics"] = {
      {"coordinate_frame", "input LiDAR coordinate frame"},
      {"center_x_y", "box center in the input LiDAR frame"},
      {"center_z", "bottom-center z, as decoded by the NVIDIA upstream core"},
      {"dimensions", "x, y, z order from NVIDIA decoder"},
      {"yaw_radians", "atan2(rot[0], rot[1]) from the NVIDIA decoder"},
      {"velocity", "upstream x/y values; no coordinate conversion applied"}};
  output["detections"] = Json::array();
  for (const auto& detection : result.detections) {
    output["detections"].push_back({{"center_x", detection.center_x},
                                    {"center_y", detection.center_y},
                                    {"center_z", detection.center_z},
                                    {"width", detection.width},
                                    {"length", detection.length},
                                    {"height", detection.height},
                                    {"velocity_x", detection.velocity_x},
                                    {"velocity_y", detection.velocity_y},
                                    {"yaw", detection.yaw},
                                    {"score", detection.score},
                                    {"class_id", detection.class_id}});
  }
  return output;
}

int Run(const std::filesystem::path& config_path,
        const std::filesystem::path& manifest_path,
        const std::filesystem::path& output_path) {
  std::string error;
  BevFusionModelConfig model_config;
  if (!LoadModelConfig(config_path, &model_config, &error)) {
    std::cerr << error << '\n';
    return 1;
  }
  Json manifest;
  if (!ReadJson(manifest_path, &manifest, &error)) {
    std::cerr << error << '\n';
    return 1;
  }
  if (!manifest.is_object() || !manifest.contains("frames") ||
      !manifest["frames"].is_array() || manifest["frames"].empty()) {
    std::cerr << "manifest must contain a nonempty frames array\n";
    return 1;
  }
  std::uint64_t max_skew_ns = 0;
  try {
    max_skew_ns = manifest.at("max_timestamp_skew_ns").get<std::uint64_t>();
  } catch (const std::exception& exception) {
    std::cerr << "manifest max_timestamp_skew_ns is required: "
              << exception.what() << '\n';
    return 1;
  }

  apollo::bevfusion::BevFusionRunner runner;
  if (!runner.Init(model_config, max_skew_ns, &error)) {
    std::cerr << error << '\n';
    return 1;
  }
  const std::filesystem::path partial_path = output_path.string() + ".partial";
  std::ofstream output(partial_path, std::ios::out | std::ios::trunc);
  if (!output) {
    std::cerr << "cannot create output file: " << partial_path << '\n';
    return 1;
  }
  const auto manifest_base = manifest_path.parent_path();
  InputLimits limits{max_skew_ns, model_config.max_input_point_count()};
  for (std::size_t index = 0; index < manifest["frames"].size(); ++index) {
    const auto frame_start = std::chrono::steady_clock::now();
    BevFusionInputView input;
    std::vector<std::vector<std::uint8_t>> image_storage;
    std::vector<float> point_storage;
    std::string calibration_revision;
    if (!LoadFrame(manifest["frames"][index], manifest_base,
                   limits.max_point_count, &input, &image_storage,
                   &point_storage, &calibration_revision, &error)) {
      std::cerr << "frame " << index << ": " << error << '\n';
      output.close();
      std::filesystem::remove(partial_path);
      return 1;
    }
    if (!apollo::bevfusion::ValidateInput(input, limits, &error)) {
      std::cerr << "frame " << index << ": " << error << '\n';
      output.close();
      std::filesystem::remove(partial_path);
      return 1;
    }
    apollo::bevfusion::BevFusionInferenceResult result;
    if (!runner.Run(input, &result, &error)) {
      std::cerr << "frame " << index << ": " << error << '\n';
      output.close();
      std::filesystem::remove(partial_path);
      return 1;
    }
    const double frame_total_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - frame_start)
            .count();
    output << MakeResultJson(result, model_config, calibration_revision, index,
                             frame_total_ms)
                  .dump()
           << '\n';
    if (!output) {
      std::cerr << "failed while writing output for frame " << index << '\n';
      output.close();
      std::filesystem::remove(partial_path);
      return 1;
    }
  }
  output.close();
  if (!output) {
    std::cerr << "failed to flush output file: " << partial_path << '\n';
    std::filesystem::remove(partial_path);
    return 1;
  }
  std::error_code rename_error;
  std::filesystem::rename(partial_path, output_path, rename_error);
  if (rename_error) {
    std::filesystem::remove(partial_path);
    std::cerr << "failed to publish results: " << rename_error.message()
              << '\n';
    return 1;
  }
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 7 || std::string(argv[1]) != "--config" ||
      std::string(argv[3]) != "--manifest" ||
      std::string(argv[5]) != "--output") {
    std::cerr << "usage: bevfusion_offline --config MODEL.json --manifest "
                 "INPUT.json --output RESULTS.jsonl\n";
    return 2;
  }
  return Run(argv[2], argv[4], argv[6]);
}
