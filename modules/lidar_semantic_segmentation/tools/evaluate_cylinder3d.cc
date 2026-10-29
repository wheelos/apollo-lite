// Copyright 2026 WheelOS. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include <sys/resource.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>  // NOLINT(build/c++17)
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <ATen/Parallel.h>

#include "nlohmann/json.hpp"

#include "modules/lidar_semantic_segmentation/inference/cylinder3d.h"

namespace {

using Json = nlohmann::json;
using Clock = std::chrono::steady_clock;
constexpr std::array<const char*, 20> kNames = {
    "ignored",       "car",      "bicycle",      "motorcycle",   "truck",
    "other-vehicle", "person",   "bicyclist",    "motorcyclist", "road",
    "parking",       "sidewalk", "other-ground", "building",     "fence",
    "vegetation",    "trunk",    "terrain",      "pole",         "traffic-sign",
};

const std::array<int, 65536>& LearningMap() {
  static const auto mapping = [] {
    std::array<int, 65536> result;
    result.fill(-1);
    for (int ignored : {0, 1, 52, 99}) {
      result[ignored] = 0;
    }
    constexpr std::array<int, 19> raw = {10, 11, 15, 18, 20, 30, 31, 32, 40, 44,
                                         48, 49, 50, 51, 70, 71, 72, 80, 81};
    for (std::size_t index = 0; index < raw.size(); ++index) {
      result[raw[index]] = index + 1;
    }
    for (int vehicle : {13, 16, 256, 257, 259}) {
      result[vehicle] = 5;
    }
    result[60] = 9;
    result[252] = 1;
    result[253] = 7;
    result[254] = 6;
    result[255] = 8;
    result[258] = 4;
    return result;
  }();
  return mapping;
}

template <typename T>
std::vector<T> Read(const std::filesystem::path& path) {
  const auto bytes = std::filesystem::file_size(path);
  if (bytes % sizeof(T) != 0) {
    throw std::runtime_error("invalid binary length: " + path.string());
  }
  std::ifstream stream(path, std::ios::binary);
  std::vector<T> values(bytes / sizeof(T));
  if (!stream.read(reinterpret_cast<char*>(values.data()), bytes)) {
    throw std::runtime_error("cannot read: " + path.string());
  }
  return values;
}

double Milliseconds(Clock::time_point begin) {
  return std::chrono::duration<double, std::milli>(Clock::now() - begin)
      .count();
}

double Percentile(std::vector<double> values, double fraction) {
  if (values.empty()) {
    return 0;
  }
  std::sort(values.begin(), values.end());
  const double index = fraction * (values.size() - 1);
  const auto low = static_cast<std::size_t>(std::floor(index));
  const auto high = static_cast<std::size_t>(std::ceil(index));
  return values[low] + (values[high] - values[low]) * (index - low);
}

struct Scan {
  std::vector<float> raw;
  std::vector<uint32_t> truth;
  apollo::drivers::PointCloud cloud;
};

Scan LoadScan(const std::filesystem::path& sequence, const std::string& id) {
  Scan scan;
  scan.raw = Read<float>(sequence / "velodyne" / (id + ".bin"));
  scan.truth = Read<uint32_t>(sequence / "labels" / (id + ".label"));
  if (scan.raw.size() != scan.truth.size() * 4) {
    throw std::runtime_error("scan/label point count mismatch: " + id);
  }
  scan.cloud.set_frame_id("velodyne");
  scan.cloud.mutable_point()->Reserve(scan.truth.size());
  for (std::size_t index = 0; index < scan.truth.size(); ++index) {
    const auto* raw = scan.raw.data() + index * 4;
    if (!std::isfinite(raw[3]) || raw[3] < 0 || raw[3] > 1) {
      throw std::runtime_error("scan remission is outside [0,1]");
    }
    if (LearningMap()[scan.truth[index] & 65535] < 0) {
      throw std::runtime_error("unmapped SemanticKITTI raw label");
    }
    auto* point = scan.cloud.add_point();
    point->set_x(raw[0]);
    point->set_y(raw[1]);
    point->set_z(raw[2]);
    point->set_intensity(static_cast<uint32_t>(std::lround(raw[3] * 255.0F)));
  }
  return scan;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 9 && argc != 10) {
    std::cerr << "Usage: evaluate_cylinder3d MODEL SEQUENCE FRAME_LIST "
                 "NEW_OUTPUT_DIR DEVICE TTA WARMUP THREADS [aten|spconv]\n";
    return 2;
  }
  std::filesystem::path output(argv[4]);
  Json report = {
      {"status", "preflight"},
      {"model", argv[1]},
      {"sequence", argv[2]},
      {"frame_list", argv[3]},
      {"device", argv[5]},
      {"tta", argv[6]},
      {"intensity", "Apollo uint8 round(remission*255), scale 1/255"},
      {"timing",
       "Segment wall time, includes projection, GPU completion "
       "and result building; excludes scan IO and GT scoring"},
      {"frames", Json::array()}};
  bool created = false;
  try {
    if (!std::filesystem::create_directory(output)) {
      throw std::runtime_error("output directory must be new");
    }
    created = true;
    const int device = std::stoi(argv[5]);
    const int tta = std::stoi(argv[6]);
    const int warmup = std::stoi(argv[7]);
    const int threads = std::stoi(argv[8]);
    if ((tta != 0 && tta != 1) || warmup < 0 || threads <= 0) {
      throw std::runtime_error("invalid TTA/warmup/thread arguments");
    }
    at::set_num_threads(threads);
    at::set_num_interop_threads(1);
    report["threads"] = threads;
    std::ifstream list(argv[3]);
    if (!list) {
      throw std::runtime_error("cannot read frame list");
    }
    std::vector<std::string> frames;
    std::string id;
    while (std::getline(list, id)) {
      if (id.size() != 6 || !std::all_of(id.begin(), id.end(), [](char value) {
            return value >= '0' && value <= '9';
          })) {
        throw std::runtime_error("frame IDs must be six decimal digits");
      }
      frames.push_back(id);
    }
    if (frames.empty()) {
      throw std::runtime_error("empty frame list");
    }
    apollo::lidar_semantic_segmentation::Cylinder3DOptions options;
    options.model_path = argv[1];
    options.device_id = device;
    options.flip_tta = tta != 0;
    const std::string backend = argc == 10 ? argv[9] : "aten";
    if (backend != "aten" && backend != "spconv") {
      throw std::runtime_error("backend must be aten or spconv");
    }
    options.use_spconv = backend == "spconv";
    report["backend"] = backend;
    options.source_topic = "/offline/semantickitti";
    options.sensor_name = "HDL64";
    apollo::lidar_semantic_segmentation::Cylinder3DSegmenter segmenter;
    std::string error;
    const auto start = Clock::now();
    if (!segmenter.Init(options, &error)) {
      throw std::runtime_error(error);
    }
    report["init_ms"] = Milliseconds(start);
    report["warmup_ms"] = Json::array();
    auto first = LoadScan(argv[2], frames.front());
    apollo::lidar_semantic_segmentation::LidarSemanticSegmentationResult result;
    for (int index = 0; index < warmup; ++index) {
      const auto begin = Clock::now();
      if (!segmenter.Segment(first.cloud, &result, &error)) {
        throw std::runtime_error(error);
      }
      report["warmup_ms"].push_back(Milliseconds(begin));
    }
    std::array<std::array<uint64_t, 20>, 20> confusion{};
    std::vector<double> latency;
    uint64_t points = 0, correct = 0, evaluated = 0, ignored = 0;
    report["status"] = "running";
    for (const auto& frame : frames) {
      const auto io_start = Clock::now();
      auto scan = LoadScan(argv[2], frame);
      const double io_ms = Milliseconds(io_start);
      const auto begin = Clock::now();
      if (!segmenter.Segment(scan.cloud, &result, &error)) {
        throw std::runtime_error("frame " + frame + ": " + error);
      }
      const double elapsed = Milliseconds(begin);
      if (result.point_label_size() != static_cast<int>(scan.truth.size())) {
        throw std::runtime_error("point correspondence mismatch");
      }
      std::vector<uint32_t> predictions;
      predictions.reserve(scan.truth.size());
      for (std::size_t index = 0; index < scan.truth.size(); ++index) {
        const auto& prediction = result.point_label(index);
        if (prediction.point_index() != index ||
            prediction.semantic_label() >= 20 ||
            !std::isfinite(prediction.confidence())) {
          throw std::runtime_error("invalid point prediction");
        }
        const auto label = prediction.semantic_label();
        predictions.push_back(label);
        const int truth = LearningMap()[scan.truth[index] & 65535];
        if (truth == 0) {
          ++ignored;
          continue;
        }
        ++confusion[truth][label];
        ++evaluated;
        correct += truth == static_cast<int>(label);
      }
      std::ofstream prediction_file(output / (frame + ".learning"),
                                    std::ios::binary);
      prediction_file.write(reinterpret_cast<const char*>(predictions.data()),
                            predictions.size() * sizeof(uint32_t));
      if (!prediction_file) {
        throw std::runtime_error("failed to persist point predictions");
      }
      latency.push_back(elapsed);
      points += scan.truth.size();
      report["frames"].push_back({{"id", frame},
                                  {"points", scan.truth.size()},
                                  {"segment_ms", elapsed},
                                  {"io_ms", io_ms}});
      std::cout << frame << " points=" << scan.truth.size()
                << " segment_ms=" << elapsed << std::endl;
      std::ofstream(output / "progress.json") << report.dump(2) << '\n';
    }
    double sum_ms = 0, miou = 0;
    for (double value : latency) {
      sum_ms += value;
    }
    report["per_class"] = Json::object();
    for (int label = 1; label < 20; ++label) {
      uint64_t truth = 0, predicted = 0;
      for (int other = 0; other < 20; ++other) {
        truth += confusion[label][other];
        predicted += confusion[other][label];
      }
      const auto tp = confusion[label][label];
      const auto total = truth + predicted - tp;
      const double iou = total == 0 ? 0.0 : static_cast<double>(tp) / total;
      miou += iou / 19.0;
      report["per_class"][kNames[label]] = {{"iou", iou},
                                            {"truth_points", truth},
                                            {"predicted_points", predicted}};
    }
    struct rusage usage {};
    getrusage(RUSAGE_SELF, &usage);
    report["status"] = "complete";
    report["point_count"] = points;
    report["evaluated_points"] = evaluated;
    report["ignored_points"] = ignored;
    report["accuracy"] =
        evaluated == 0 ? 0.0 : static_cast<double>(correct) / evaluated;
    report["miou_19"] = miou;
    report["confusion_true_rows_predicted_columns"] = confusion;
    report["latency_ms"] = {{"mean", sum_ms / latency.size()},
                            {"p50", Percentile(latency, 0.5)},
                            {"p95", Percentile(latency, 0.95)},
                            {"min", Percentile(latency, 0.0)},
                            {"max", Percentile(latency, 1.0)}};
    report["sequential_fps_excluding_io"] = latency.size() * 1000.0 / sum_ms;
    report["peak_rss_kib"] = usage.ru_maxrss;
    std::ofstream(output / "report.json") << report.dump(2) << '\n';
    std::cout << report.at("latency_ms") << " miou=" << miou << '\n';
  } catch (const std::exception& exception) {
    report["status"] = "failed";
    report["error"] = exception.what();
    if (created) {
      std::ofstream(output / "report.json") << report.dump(2) << '\n';
    }
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
