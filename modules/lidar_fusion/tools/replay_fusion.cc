// Copyright 2026 WheelOS. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include <chrono>
#include <cstdint>
#include <filesystem>  // NOLINT(build/c++17)
#include <fstream>
#include <iostream>
#include <iterator>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

#include "google/protobuf/text_format.h"

#include "modules/lidar_fusion/core/temporal.h"

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
  Require(result.ParseFromString(Read(path)),
          "invalid protobuf " + path.string());
  return result;
}
void Save(const fs::path& path, const google::protobuf::Message& message) {
  std::ofstream out(path, std::ios::binary);
  Require(message.SerializeToOstream(&out), "cannot write " + path.string());
}

int Run(int argc, char** argv) {
  Require(argc == 5,
          "usage: replay_fusion CLOUD_DIR MODEL_RESULT_DIR POSE_DIR "
          "NEW_OUTPUT_DIR");
  const fs::path clouds(argv[1]), models(argv[2]), poses(argv[3]),
      output(argv[4]);
  Require(!fs::exists(output), "output exists");
  fs::create_directories(output);
  fusion::FusionConfig c;
  Require(google::protobuf::TextFormat::ParseFromString(
              Read(poses / "fusion.pb.txt"), &c),
          "invalid config");
  Require(c.temporal().enabled(),
          "temporal replay needs explicit enabled config");
  fusion::StatefulFusion processor, dropout;
  fusion::SingleFrameFusion baseline;
  std::string error;
  Require(processor.Init(c, &error), error);
  Require(dropout.Init(c, &error), error);
  Require(baseline.Init(c, &error), error);
  std::map<std::string, fs::path> paths;
  for (const auto& file : fs::directory_iterator(clouds)) {
    if (file.path().filename().string().find(".cloud.pb") !=
        std::string::npos) {
      paths[file.path().filename().string()] = file.path();
    }
  }
  std::ofstream csv(output / "temporal.csv");
  csv << "frame,objects,unknowns,tracks,confirmed,confirmed_unknown,history_"
         "cells,"
         "observed_tracks,continued_ids,recovered,pose_valid,diagnostic_pose,"
         "repairs,fusion_ms,baseline_objects,baseline_unknowns,baseline_ms,"
         "dropout_objects,dropout_unknowns,dropout_recovered,dropout_ms\n";
  std::set<uint64_t> previous;
  unsigned index = 0, continued_total = 0, confirmed_total = 0,
           recovered_total = 0;
  for (const auto& item : paths) {
    const auto& name = item.first;
    const auto cloud = Proto<apollo::drivers::PointCloud>(item.second);
    const auto det = Proto<fusion::DetectionFrame>(models / (name + ".det.pb"));
    const auto seg = Proto<fusion::SemanticFrame>(models / (name + ".seg.pb"));
    fusion::LocalPose pose;
    Require(google::protobuf::TextFormat::ParseFromString(
                Read(poses / (name + ".pose.txt")), &pose),
            "invalid pose");
    const auto raw_cloud = cloud.SerializeAsString(),
               raw_det = det.SerializeAsString(),
               raw_seg = seg.SerializeAsString();
    fusion::FusionOutput out, single, missing;
    const auto start = std::chrono::steady_clock::now();
    Require(processor.Process(cloud, &det, &seg, &pose, &out, &error), error);
    const double ms = std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - start)
                          .count();
    Require(out.health().temporal_enabled() && out.health().pose_valid(),
            "temporal state failed on " + name);
    auto compare_start = std::chrono::steady_clock::now();
    Require(baseline.Process(cloud, &det, &seg, &single, &error), error);
    const double baseline_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - compare_start)
            .count();
    std::set<std::pair<unsigned, unsigned>> occupied;
    for (const auto& cell : out.space_map().cell()) {
      if (cell.occupancy_obs()) {
        occupied.insert({cell.x_index(), cell.y_index()});
      }
      Require(cell.traversability() != fusion::FREE &&
                  cell.traversability() != fusion::FREE_SOFT,
              "diagnostic replay emitted FREE");
    }
    for (const auto& cell : single.space_map().cell()) {
      Require(!cell.occupancy_obs() ||
                  occupied.count({cell.x_index(), cell.y_index()}) != 0,
              "temporal erased current observed occupancy");
    }
    // Independent trajectory receives a detector dropout on every fourth
    // sample.
    compare_start = std::chrono::steady_clock::now();
    Require(dropout.Process(cloud, index % 4 == 3 ? nullptr : &det, &seg, &pose,
                            &missing, &error),
            error);
    const double dropout_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - compare_start)
            .count();
    unsigned dropout_recovered = 0;
    for (const auto& object : missing.object()) {
      dropout_recovered += object.source() == fusion::TRACK_PRIOR;
    }
    recovered_total += dropout_recovered;
    unsigned confirmed = 0, unknown = 0, observed = 0, continued = 0,
             recovered = 0;
    std::set<uint64_t> current;
    for (const auto& track : out.track()) {
      Require(current.insert(track.id()).second, "duplicate stable track ID");
      confirmed += track.confirmed();
      unknown += track.confirmed() && track.unknown();
      observed += track.observed();
      continued += previous.count(track.id());
    }
    for (const auto& object : out.object()) {
      recovered += object.source() == fusion::TRACK_PRIOR;
      Require(current.count(object.track_id()) != 0, "dangling object track");
    }
    for (const auto& u : out.unknown()) {
      Require(current.count(u.track_id()) != 0, "dangling unknown track");
    }
    continued_total += continued;
    confirmed_total += confirmed;
    Require(raw_cloud == cloud.SerializeAsString() &&
                raw_det == det.SerializeAsString() &&
                raw_seg == seg.SerializeAsString(),
            "mutated input");
    Save(output / (name + ".temporal.pb"), out);
    Save(output / (name + ".dropout.pb"), missing);
    Save(output / (name + ".single.pb"), single);
    csv << name << ',' << out.object_size() << ',' << out.unknown_size() << ','
        << out.track_size() << ',' << confirmed << ',' << unknown << ','
        << out.history_cell_size() << ',' << observed << ',' << continued << ','
        << recovered << ',' << out.health().pose_valid() << ','
        << out.health().diagnostic_pose() << ','
        << out.health().consistency_repairs() << ',' << ms << ','
        << single.object_size() << ',' << single.unknown_size() << ','
        << baseline_ms << ',' << missing.object_size() << ','
        << missing.unknown_size() << ',' << dropout_recovered << ','
        << dropout_ms << '\n';
    csv.flush();
    previous = std::move(current);
    ++index;
    std::cout << name
              << ": temporal + independent dropout + immutable occupancy OK"
              << std::endl;
  }
  Require(!paths.empty(), "no input clouds");
  Require(continued_total > 0 && confirmed_total > 0,
          "sequence never exercised stable association/confirmation");
  Save(output / "config.pb", c);
  std::ofstream summary(output / "summary.txt");
  summary << "frames " << index << "\ncontinued_track_observations "
          << continued_total << "\nconfirmed_track_observations "
          << confirmed_total << "\ndropout_recovered_objects "
          << recovered_total
          << "\npose_contract diagnostic_only; no ODOM/accuracy acceptance\n";
  Require(summary.good() && csv.good(), "failed to persist summary");
  return 0;
}
}  // namespace
int main(int argc, char** argv) {
  try {
    return Run(argc, argv);
  } catch (const std::exception& e) {
    std::cerr << "Fusion replay stopped: " << e.what() << std::endl;
    return 1;
  }
}
