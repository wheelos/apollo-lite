// Copyright 2026 WheelOS. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include <filesystem>  // NOLINT(build/c++17)
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <map>
#include <stdexcept>
#include <string>

#include "google/protobuf/text_format.h"

#include "modules/lidar_fusion/core/fusion.h"

namespace fs = std::filesystem;
namespace fusion = apollo::lidar_fusion;
namespace {

void Require(bool ok, const std::string& message) {
  if (!ok) {
    throw std::runtime_error(message);
  }
}

std::string Read(const fs::path& path) {
  std::ifstream input(path, std::ios::binary);
  Require(input.good(), "cannot read " + path.string());
  return std::string(std::istreambuf_iterator<char>(input), {});
}

template <typename Message>
Message Proto(const fs::path& path) {
  Message result;
  Require(result.ParseFromString(Read(path)), "invalid " + path.string());
  return result;
}

int Run(int argc, char** argv) {
  Require(argc >= 4 && argc <= 7,
          "usage: replay_cached CLOUD_DIR MODEL_DIR NEW_OUTPUT_DIR [REPEATS] "
          "[--profile] [--shadow-partitions]");
  const fs::path clouds(argv[1]), models(argv[2]), output(argv[3]);
  int next = 4;
  const int repeats = next < argc && std::string(argv[next]).rfind("--", 0) != 0
                          ? std::stoi(argv[next++])
                          : 1;
  bool profile = false, shadow = false;
  for (; next < argc; ++next) {
    const std::string option(argv[next]);
    if (option == "--profile" && !profile) {
      profile = true;
    } else if (option == "--shadow-partitions" && !shadow) {
      shadow = true;
    } else {
      Require(false, "unknown or duplicate option " + option);
    }
  }
  Require(repeats > 0 && repeats <= 100, "repeats must be in [1,100]");
  fusion::FusionConfig config;
  Require(google::protobuf::TextFormat::ParseFromString(
              Read(clouds / "fusion.pb.txt"), &config),
          "invalid config");
  if (shadow) {
    config.mutable_candidates()->set_shadow_semantic_partitions(true);
  }
  fusion::SingleFrameFusion core;
  std::string error;
  Require(core.Init(config, &error), error);
  std::map<std::string, fs::path> paths;
  for (const auto& entry : fs::directory_iterator(clouds)) {
    const auto name = entry.path().filename().string();
    if (name.size() >= 9 && name.substr(name.size() - 9) == ".cloud.pb") {
      paths.emplace(name, entry.path());
    }
  }
  Require(!paths.empty(), "no input clouds");
  Require(!fs::exists(output), "output directory already exists");
  fs::create_directories(output);
  {
    std::ofstream metadata(output / "config.pb", std::ios::binary);
    Require(config.SerializeToOstream(&metadata), "cannot save config");
    std::ofstream arguments(output / "arguments.txt");
    for (int i = 0; i < argc; ++i) {
      arguments << argv[i] << '\n';
    }
    Require(arguments.good(), "cannot save arguments");
  }
  std::ofstream csv(output / "timing.csv");
  csv << "frame,mode,iteration,warmup,fusion_ms\n" << std::setprecision(12);
  std::ofstream summary(output / "summary.csv");
  summary << "frame,points,mode,detections,objects,unknowns,conflicts,"
             "observed_cells,inferred_cells,unknown_cells,free_cells,"
             "ground_points,repairs,det_ms,seg_ms,fusion_ms\n"
          << std::setprecision(12);
  std::ofstream stages;
  if (profile) {
    stages.open(output / "stages.csv");
    stages << "frame,mode,iteration,warmup,evidence_ms,components_ms,"
              "association_ms,candidates_ms,relations_ms,space_ms,"
              "visibility_ms,output_ms,voxels,neighbor_edges,"
              "relation_pairs,relation_point_transforms\n"
           << std::setprecision(12);
  }
  for (const auto& item : paths) {
    const auto cloud = Proto<apollo::drivers::PointCloud>(item.second);
    const auto det =
        Proto<fusion::DetectionFrame>(models / (item.first + ".det.pb"));
    const auto seg =
        Proto<fusion::SemanticFrame>(models / (item.first + ".seg.pb"));
    const auto raw_cloud = cloud.SerializeAsString();
    const auto raw_det = det.SerializeAsString();
    const auto raw_seg = seg.SerializeAsString();
    for (const auto& suffix : {".det.pb", ".seg.pb"}) {
      Require(fs::copy_file(models / (item.first + suffix),
                            output / (item.first + suffix)),
              "cannot copy cached evidence");
    }
    for (int mode = fusion::NORMAL; mode <= fusion::GEO_ONLY; ++mode) {
      std::string expected;
      for (int iteration = 0; iteration <= repeats; ++iteration) {
        fusion::FusionOutput result;
        fusion::FusionWork work;
        Require(
            core.Process(
                cloud,
                mode == fusion::NO_DET || mode == fusion::GEO_ONLY ? nullptr
                                                                   : &det,
                mode == fusion::NO_SEG || mode == fusion::GEO_ONLY ? nullptr
                                                                   : &seg,
                &result, &error, nullptr, nullptr, profile ? &work : nullptr),
            error);
        Require(result.health().mode() == mode, "wrong degraded mode");
        csv << item.first << ',' << mode << ',' << iteration << ','
            << (iteration == 0) << ',' << result.health().latency_ms() << '\n';
        if (iteration == repeats) {
          unsigned observed = 0, inferred = 0, unknown = 0, free = 0,
                   ground = 0, conflicts = 0;
          for (const auto& cell : result.space_map().cell()) {
            observed += cell.occupancy_obs();
            inferred += cell.occupancy_det();
            unknown += cell.occupancy_unknown();
            free += cell.traversability() == fusion::FREE ||
                    cell.traversability() == fusion::FREE_SOFT;
            Require(!(cell.occupancy_obs() || cell.occupancy_det() ||
                      cell.occupancy_unknown()) ||
                        cell.traversability() == fusion::OCCUPIED,
                    "occupied/free contradiction");
          }
          Require(config.spatial().verified_single_origin() || free == 0,
                  "approximate visibility emitted FREE");
          for (const auto& point : result.point_evidence()) {
            ground += point.is_ground_geo();
          }
          for (const auto& object : result.object()) {
            conflicts += object.class_conflict();
          }
          summary << item.first << ',' << cloud.point_size() << ',' << mode
                  << ',' << det.detection_size() << ',' << result.object_size()
                  << ',' << result.unknown_size() << ',' << conflicts << ','
                  << observed << ',' << inferred << ',' << unknown << ','
                  << free << ',' << ground << ','
                  << result.health().consistency_repairs() << ",-1,-1,"
                  << result.health().latency_ms() << '\n';
          if (mode == fusion::NORMAL) {
            std::ofstream normal(output / (item.first + ".fusion.pb"),
                                 std::ios::binary);
            Require(result.SerializeToOstream(&normal),
                    "cannot save normal output");
          }
        }
        if (profile) {
          stages << item.first << ',' << mode << ',' << iteration << ','
                 << (iteration == 0);
          for (double ms : work.stage_ms) {
            stages << ',' << ms;
          }
          stages << ',' << work.voxels << ',' << work.neighbor_edges << ','
                 << work.relation_pairs << ',' << work.relation_point_transforms
                 << '\n';
          Require(stages.good(), "cannot save stages");
        }
        result.mutable_health()->clear_latency_ms();
        const auto payload = result.SerializeAsString();
        if (iteration == 0) {
          expected = payload;
          std::ofstream pb(
              output / (item.first + "." + std::to_string(mode) + ".pb"),
              std::ios::binary);
          Require(result.SerializeToOstream(&pb), "cannot save result");
        } else {
          Require(payload == expected, "nondeterministic output");
        }
      }
    }
    fusion::DetectionFrame wrong = det;
    wrong.mutable_header()->set_sequence_num(cloud.header().sequence_num() + 1);
    fusion::FusionOutput rejected;
    Require(core.Process(cloud, &wrong, &seg, &rejected, &error), error);
    Require(rejected.health().mode() == fusion::NO_DET,
            "wrong sweep was accepted");
    Require(cloud.SerializeAsString() == raw_cloud &&
                det.SerializeAsString() == raw_det &&
                seg.SerializeAsString() == raw_seg,
            "input evidence changed");
    csv.flush();
    summary.flush();
    Require(csv.good() && summary.good(), "cannot save timing/summary");
    std::cout << item.first << ": four modes, determinism, immutable inputs OK"
              << std::endl;
  }
  return 0;
}
}  // namespace

int main(int argc, char** argv) {
  try {
    return Run(argc, argv);
  } catch (const std::exception& e) {
    std::cerr << "Cached replay stopped: " << e.what() << std::endl;
    return 1;
  }
}
