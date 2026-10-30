// Copyright 2026 WheelOS. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <cstdint>
#include <filesystem>  // NOLINT(build/c++17)
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "modules/lidar_semantic_segmentation/types/range_projection.h"

namespace fs = std::filesystem;
namespace segmentation = apollo::lidar_semantic_segmentation;

namespace {

void Require(bool valid, const std::string& error) {
  if (!valid) {
    throw std::runtime_error(error);
  }
}

template <typename T>
void Save(const fs::path& path, const std::vector<T>& values) {
  Require(!fs::exists(path), "refusing to overwrite " + path.string());
  std::ofstream output(path, std::ios::binary);
  output.write(reinterpret_cast<const char*>(values.data()),
               values.size() * sizeof(T));
  output.flush();
  Require(output.good(), "cannot save " + path.string());
}

int Run(int argc, char** argv) {
  Require(argc == 3, "usage: project_semantickitti RAW_SCAN_LIST OUTPUT_DIR");
  std::ifstream paths(argv[1]);
  Require(paths.good(), "cannot read raw scan list");
  const fs::path output(argv[2]);
  Require(!fs::exists(output), "output directory already exists");
  fs::create_directories(output);
  segmentation::RangeImageProjectionOptions options;
  options.intensity_scale = 1.0F / 255.0F;
  segmentation::RangeImageProjector projector;
  std::string error;
  Require(projector.Init(options, &error), error);
  std::ofstream metadata(output / "samples.csv");
  metadata << "index,points,width,height\n";
  std::string path;
  std::size_t index = 0;
  while (std::getline(paths, path)) {
    Require(!path.empty(), "empty scan-list entry");
    std::ifstream input(path, std::ios::binary | std::ios::ate);
    Require(input.good(), "cannot read " + path);
    const std::streamoff bytes = input.tellg();
    Require(bytes > 0 && bytes % (4 * sizeof(float)) == 0 &&
                bytes / (4 * sizeof(float)) <= options.max_points,
            "invalid four-float scan size: " + path);
    std::vector<float> raw(bytes / sizeof(float));
    input.seekg(0);
    input.read(reinterpret_cast<char*>(raw.data()), bytes);
    Require(input.good(), "incomplete raw scan: " + path);
    apollo::drivers::PointCloud cloud;
    for (std::size_t i = 0; i < raw.size(); i += 4) {
      Require(std::isfinite(raw[i]) && std::isfinite(raw[i + 1]) &&
                  std::isfinite(raw[i + 2]) && std::isfinite(raw[i + 3]) &&
                  raw[i + 3] >= 0 && raw[i + 3] <= 1,
              "nonfinite scan or remission outside [0,1]: " + path);
      auto* point = cloud.add_point();
      point->set_x(raw[i]);
      point->set_y(raw[i + 1]);
      point->set_z(raw[i + 2]);
      point->set_intensity(static_cast<uint32_t>(
          std::nearbyint(static_cast<double>(raw[i + 3]) * 255)));
    }
    segmentation::RangeImage image;
    Require(projector.Project(cloud, &image, &error), error);
    std::vector<float> ranges;
    std::vector<int32_t> pixels;
    ranges.reserve(image.points.size());
    pixels.reserve(image.points.size());
    for (const auto& point : image.points) {
      ranges.push_back(point.range);
      pixels.push_back(
          point.valid ? static_cast<int32_t>(point.y * image.width + point.x)
                      : -1);
    }
    const auto prefix = output / std::to_string(index);
    Save(prefix.string() + ".input.f32", image.input_chw);
    Save(prefix.string() + ".range.f32", ranges);
    Save(prefix.string() + ".pixel.i32", pixels);
    Save(prefix.string() + ".pixel_to_point.i32", image.pixel_to_point_index);
    Save(prefix.string() + ".projected_range.f32", image.projected_range);
    metadata << index << ',' << image.points.size() << ',' << image.width << ','
             << image.height << '\n';
    metadata.flush();
    Require(metadata.good(), "cannot save projection metadata");
    ++index;
  }
  Require(paths.eof() && index > 0, "incomplete or empty scan list");
  std::cout << index << " native projections saved\n";
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    return Run(argc, argv);
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
