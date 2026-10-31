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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>  // NOLINT(build/c++17)
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "google/protobuf/text_format.h"

#include "modules/lidar_fusion/core/runtime/single_frame_fusion.h"
#include "modules/lidar_object_detection/inference/pointpillars_runtime_api.h"
#include "modules/lidar_object_detection/types/pointpillars_input.h"
#include "modules/lidar_semantic_segmentation/inference/rangeret.h"
#include "modules/lidar_semantic_segmentation/inference/tensorrt_executor.h"

namespace fs = std::filesystem;
namespace fusion = apollo::lidar_fusion;
namespace detection = apollo::lidar_object_detection;
namespace segmentation = apollo::lidar_semantic_segmentation;

namespace {

void Require(bool success, const std::string& error) {
  if (!success) {
    throw std::runtime_error(error);
  }
}

std::string Read(const fs::path& path) {
  std::ifstream input(path, std::ios::binary);
  Require(input.good(), "cannot read " + path.string());
  return std::string(std::istreambuf_iterator<char>(input), {});
}

void Save(const fs::path& path, const google::protobuf::Message& message) {
  Require(!fs::exists(path), "refusing to overwrite " + path.string());
  std::ofstream output(path, std::ios::binary);
  Require(message.SerializeToOstream(&output), "cannot save " + path.string());
}

double Milliseconds(std::chrono::steady_clock::time_point begin) {
  return std::chrono::duration<double, std::milli>(
             std::chrono::steady_clock::now() - begin)
      .count();
}

int Run(int argc, char** argv) {
  Require(argc >= 7 && argc <= 9,
          "usage: replay INPUT_DIR OUTPUT_DIR PP_ENGINE SEG_ENGINE "
          "PP_INTENSITY_SCALE SEG_INTENSITY_SCALE [CACHED_DETECTION_DIR "
          "[CACHED_SEGMENTATION_DIR]]");
  const bool cached_detection = argc >= 8;
  const bool cached_segmentation = argc == 9;
  const fs::path input(argv[1]), output(argv[2]);
  Require(!fs::exists(output), "output directory already exists");
  fs::create_directories(output);
  fusion::FusionConfig config;
  Require(google::protobuf::TextFormat::ParseFromString(
              Read(input / "fusion.pb.txt"), &config),
          "invalid fusion config");
  Require(cached_detection ||
              config.detection_backend() ==
                  detection::LIDAR_OBJECT_DETECTION_BACKEND_POINTPILLARS,
          "offline runner requires PointPillars mapping");
  fusion::SingleFrameFusion core;
  std::string error;
  Require(core.Init(config, &error), error);
  {
    std::ofstream metadata(output / "arguments.txt");
    for (int i = 0; i < argc; ++i) {
      metadata << argv[i] << '\n';
    }
    Require(metadata.good(), "cannot freeze replay arguments");
    Save(output / "config.pb", config);
  }
  char runtime_error[2048] = {};
  Require(pointpillars_runtime_abi_version() == 1 &&
              pointpillars_runtime_detection_size() ==
                  sizeof(PointPillarsDetection),
          "incompatible PointPillars ABI");
  std::unique_ptr<void, decltype(&pointpillars_runtime_destroy)> runtime(
      cached_detection ? nullptr
                       : pointpillars_runtime_create(argv[3], 0, runtime_error,
                                                     sizeof(runtime_error)),
      pointpillars_runtime_destroy);
  Require(cached_detection || runtime != nullptr, runtime_error);
  segmentation::TensorRtRangeRetExecutor executor;
  segmentation::RangeRetSegmenter segmenter(&executor);
  segmentation::RangeRetModelOptions options;
  options.engine_path = argv[4];
  options.sensor_name = config.sensor_frame();
  options.source_topic = config.pointcloud_channel();
  options.projection.intensity_scale = std::stof(argv[6]);
  if (!cached_segmentation) {
    Require(segmenter.Init(options, &error), error);
  }
  std::vector<fs::path> paths;
  for (const auto& entry : fs::directory_iterator(input)) {
    if (entry.path().filename().string().find(".cloud.pb") !=
        std::string::npos) {
      paths.push_back(entry.path());
    }
  }
  std::sort(paths.begin(), paths.end());
  Require(!paths.empty(), "no cloud protobufs");
  std::ofstream csv(output / "summary.csv");
  csv << "frame,points,mode,detections,objects,unknowns,conflicts,"
         "observed_cells,inferred_cells,unknown_cells,free_cells,ground_points,"
         "repairs,det_ms,seg_ms,fusion_ms\n";
  std::vector<float> xyzi;
  std::vector<PointPillarsDetection> boxes(4096);
  for (const auto& path : paths) {
    apollo::drivers::PointCloud cloud;
    Require(cloud.ParseFromString(Read(path)), "invalid cloud protobuf");
    const auto raw = cloud.SerializeAsString();
    if (!cached_detection) {
      Require(
          detection::BuildPointPillarsInput(cloud, config.sensor_frame(),
                                            std::stof(argv[5]), &xyzi, &error),
          error);
    }
    auto begin = std::chrono::steady_clock::now();
    std::size_t count = 0;
    if (!cached_detection &&
        !pointpillars_runtime_infer(
            runtime.get(), xyzi.data(), cloud.point_size(), boxes.data(),
            boxes.size(), &count, runtime_error, sizeof(runtime_error))) {
      throw std::runtime_error(path.filename().string() +
                               ": PointPillars: " + runtime_error);
    }
    const double det_ms = Milliseconds(begin);
    fusion::DetectionFrame det;
    *det.mutable_header() = cloud.header();
    det.set_backend(config.detection_backend());
    det.set_sensor_frame_id(config.sensor_frame());
    det.set_model_revision(fs::path(argv[3]).filename().string());
    det.set_input_point_count(cloud.point_size());
    for (std::size_t i = 0; i < count; ++i) {
      const auto& b = boxes[i];
      auto* d = det.add_detection();
      d->set_center_x(b.x);
      d->set_center_y(b.y);
      d->set_center_z(b.z);
      d->set_size_x(b.dx);
      d->set_size_y(b.dy);
      d->set_size_z(b.dz);
      d->set_yaw(b.yaw);
      d->set_class_id(b.class_id);
      d->set_score(b.score);
    }
    if (cached_detection) {
      Require(det.ParseFromString(Read(fs::path(argv[7]) /
                                       (path.filename().string() + ".det.pb"))),
              "invalid cached detection protobuf");
    }
    fusion::SemanticFrame seg;
    begin = std::chrono::steady_clock::now();
    if (cached_segmentation) {
      Require(seg.ParseFromString(Read(fs::path(argv[8]) /
                                       (path.filename().string() + ".seg.pb"))),
              "invalid cached segmentation protobuf");
    } else {
      Require(segmenter.Segment(cloud, &seg, &error), error);
    }
    const double seg_ms = Milliseconds(begin);
    const auto det_raw = det.SerializeAsString();
    const auto seg_raw = seg.SerializeAsString();
    const auto stem = path.filename().string();
    Save(output / (stem + ".det.pb"), det);
    Save(output / (stem + ".seg.pb"), seg);
    for (int mode = fusion::NORMAL; mode <= fusion::GEO_ONLY; ++mode) {
      fusion::FusionOutput result;
      begin = std::chrono::steady_clock::now();
      Require(core.Process(
                  cloud,
                  (mode == fusion::NO_DET || mode == fusion::GEO_ONLY) ? nullptr
                                                                       : &det,
                  (mode == fusion::NO_SEG || mode == fusion::GEO_ONLY) ? nullptr
                                                                       : &seg,
                  &result, &error),
              error);
      const double fusion_ms = Milliseconds(begin);
      Require(result.health().mode() == mode, "unexpected degraded mode");
      Require(result.point_evidence_size() == cloud.point_size(),
              "point evidence index mismatch");
      uint32_t observed = 0, inferred = 0, unknown_cells = 0, free = 0,
               ground = 0, conflicts = 0;
      for (const auto& c : result.space_map().cell()) {
        observed += c.occupancy_obs();
        inferred += c.occupancy_det();
        unknown_cells += c.occupancy_unknown();
        free += c.traversability() == fusion::FREE ||
                c.traversability() == fusion::FREE_SOFT;
        Require(!(c.occupancy_obs() || c.occupancy_det() ||
                  c.occupancy_unknown()) ||
                    c.traversability() == fusion::OCCUPIED,
                "occupied/free contradiction");
      }
      for (const auto& p : result.point_evidence()) {
        Require(p.point_id() < static_cast<uint32_t>(cloud.point_size()),
                "invalid point id");
        ground += p.is_ground_geo();
      }
      for (const auto& o : result.object()) {
        conflicts += o.class_conflict();
        Require(o.consistency() != fusion::CONFLICT || o.class_conflict(),
                "missing conflict flag");
      }
      Require(!config.spatial().verified_single_origin() ? free == 0 : true,
              "approximate visibility produced FREE");
      csv << stem << ',' << cloud.point_size() << ',' << mode << ','
          << det.detection_size() << ',' << result.object_size() << ','
          << result.unknown_size() << ',' << conflicts << ',' << observed << ','
          << inferred << ',' << unknown_cells << ',' << free << ',' << ground
          << ',' << result.health().consistency_repairs() << ','
          << (cached_detection ? -1.0 : det_ms) << ','
          << (cached_segmentation ? -1.0 : seg_ms) << ',' << fusion_ms << '\n';
      csv.flush();
      if (mode == fusion::NORMAL) {
        Save(output / (stem + ".fusion.pb"), result);
      }
    }
    fusion::DetectionFrame wrong_frame = det;
    wrong_frame.mutable_header()->set_sequence_num(
        cloud.header().sequence_num() + 1);
    fusion::FusionOutput rejected;
    Require(core.Process(cloud, &wrong_frame, &seg, &rejected, &error), error);
    Require(rejected.health().mode() == fusion::NO_DET,
            "mismatched sweep accepted");
    Require(raw == cloud.SerializeAsString() &&
                det_raw == det.SerializeAsString() &&
                seg_raw == seg.SerializeAsString(),
            "input evidence mutated");
    std::cout << stem << ": dual-model + four modes + wrong sweep OK"
              << std::endl;
  }
  Require(csv.good(), "cannot write replay summary");
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    return Run(argc, argv);
  } catch (const std::exception& e) {
    std::cerr << "Replay stopped: " << e.what() << std::endl;
    return 1;
  }
}
