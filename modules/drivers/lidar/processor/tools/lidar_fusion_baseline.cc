// Copyright 2026 WheelOS All Rights Reserved.
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
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/Geometry"

#include "cyber/record/record_message.h"
#include "cyber/record/record_reader.h"
#include "modules/drivers/lidar/processor/control/time_contract.h"
#include "modules/drivers/lidar/processor/policy/cpu_lidar_policy.h"
#include "modules/transform/buffer_interface.h"

namespace apollo {
namespace drivers {
namespace lidar {
namespace {

using apollo::cyber::record::RecordMessage;
using apollo::cyber::record::RecordReader;

struct Options {
  std::string record;
  std::string primary_topic;
  std::vector<std::string> auxiliary_topics;
  size_t max_frames = 100;
  size_t max_points = 600000;
  uint32_t max_delta_ms = 80;
  size_t motion_bins = 12;
  double speed_mps = 5.0;
  double yaw_rate_rps = 0.05;
  bool enable_ego_filter = false;
  bool enable_voxel_filter = false;
  float voxel_size = 0.15F;
};

struct InputFrame {
  std::shared_ptr<PointCloud> cloud;
  TimeContract time_contract;
};

struct RunMetrics {
  size_t requested_primary_frames = 0;
  size_t processed_frames = 0;
  size_t full_match_frames = 0;
  size_t point_timestamp_frames = 0;
  size_t fallback_timestamp_frames = 0;
  uint64_t input_points = 0;
  uint64_t output_points = 0;
  uint64_t truncated_frames = 0;
  uint64_t compact_points = 0;
  uint64_t ego_filtered_points = 0;
  uint64_t voxel_filtered_points = 0;
  uint64_t checksum = 1469598103934665603ULL;
  double motion_probe_rms_m = 0.0;
  std::vector<double> latency_ms;
  std::vector<double> setup_ms;
  std::vector<double> fusion_ms;
  std::vector<double> filter_ms;
};

class NonQueryingBuffer final : public apollo::transform::BufferInterface {
 public:
  apollo::transform::TransformStamped lookupTransform(
      const std::string&, const std::string&, const cyber::Time&,
      const float) const override {
    throw std::runtime_error("baseline mock TF does not support lookup");
  }

  apollo::transform::TransformStamped lookupTransform(
      const std::string&, const cyber::Time&, const std::string&,
      const cyber::Time&, const std::string&, const float) const override {
    throw std::runtime_error("baseline mock TF does not support lookup");
  }

  bool canTransform(const std::string&, const std::string&,
                    const cyber::Time&, const float,
                    std::string*) const override {
    return false;
  }

  bool canTransform(const std::string&, const cyber::Time&,
                    const std::string&, const cyber::Time&,
                    const std::string&, const float,
                    std::string*) const override {
    return false;
  }

  bool GetLatestStaticTransform(const std::string&, const std::string&,
                                apollo::transform::TransformStamped*)
      const override {
    return false;
  }
};

std::vector<std::string> Split(const std::string& value, char separator) {
  std::vector<std::string> parts;
  std::stringstream stream(value);
  std::string part;
  while (std::getline(stream, part, separator)) {
    if (!part.empty()) {
      parts.push_back(part);
    }
  }
  return parts;
}

bool ParseUnsigned(const std::string& value, size_t* output) {
  if (output == nullptr || value.empty()) {
    return false;
  }
  char* end = nullptr;
  const unsigned long long parsed = std::strtoull(value.c_str(), &end, 10);
  if (end == value.c_str() || *end != '\0' ||
      parsed > std::numeric_limits<size_t>::max()) {
    return false;
  }
  *output = static_cast<size_t>(parsed);
  return true;
}

bool ParseDouble(const std::string& value, double* output) {
  if (output == nullptr || value.empty()) {
    return false;
  }
  char* end = nullptr;
  const double parsed = std::strtod(value.c_str(), &end);
  if (end == value.c_str() || *end != '\0' || !std::isfinite(parsed)) {
    return false;
  }
  *output = parsed;
  return true;
}

bool ParseBool(const std::string& value, bool* output) {
  if (output == nullptr) {
    return false;
  }
  if (value == "true") {
    *output = true;
    return true;
  }
  if (value == "false") {
    *output = false;
    return true;
  }
  return false;
}

bool ParseOptions(int argc, char** argv, Options* options) {
  if (options == nullptr) {
    return false;
  }
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    const size_t separator = argument.find('=');
    if (separator == std::string::npos || argument.rfind("--", 0) != 0) {
      std::cerr << "Invalid argument: " << argument << std::endl;
      return false;
    }
    const std::string name = argument.substr(2, separator - 2);
    const std::string value = argument.substr(separator + 1);
    if (name == "record") {
      options->record = value;
    } else if (name == "primary_topic") {
      options->primary_topic = value;
    } else if (name == "auxiliary_topics") {
      options->auxiliary_topics = Split(value, ',');
    } else if (name == "max_frames") {
      if (!ParseUnsigned(value, &options->max_frames)) {
        return false;
      }
    } else if (name == "max_points") {
      if (!ParseUnsigned(value, &options->max_points)) {
        return false;
      }
    } else if (name == "max_delta_ms") {
      size_t parsed = 0;
      if (!ParseUnsigned(value, &parsed) ||
          parsed > std::numeric_limits<uint32_t>::max()) {
        return false;
      }
      options->max_delta_ms = static_cast<uint32_t>(parsed);
    } else if (name == "motion_bins") {
      if (!ParseUnsigned(value, &options->motion_bins)) {
        return false;
      }
    } else if (name == "speed_mps") {
      if (!ParseDouble(value, &options->speed_mps)) {
        return false;
      }
    } else if (name == "yaw_rate_rps") {
      if (!ParseDouble(value, &options->yaw_rate_rps)) {
        return false;
      }
    } else if (name == "enable_ego_filter") {
      if (!ParseBool(value, &options->enable_ego_filter)) {
        return false;
      }
    } else if (name == "enable_voxel_filter") {
      if (!ParseBool(value, &options->enable_voxel_filter)) {
        return false;
      }
    } else if (name == "voxel_size") {
      double voxel_size = 0.0;
      if (!ParseDouble(value, &voxel_size) || voxel_size <= 1e-4) {
        return false;
      }
      options->voxel_size = static_cast<float>(voxel_size);
    } else {
      std::cerr << "Unknown argument: --" << name << std::endl;
      return false;
    }
  }
  return !options->record.empty() && !options->primary_topic.empty() &&
         options->max_frames > 0U && options->max_points > 0U &&
         options->max_delta_ms > 0U && options->motion_bins > 0U;
}

LidarUnifiedComponentConfig::TimeSettings DefaultTimeSettings() {
  LidarUnifiedComponentConfig::TimeSettings settings;
  settings.set_measurement_time_anchor(LidarUnifiedComponentConfig::SCAN_END);
  settings.set_expected_scan_duration_ms(100.0);
  settings.set_max_scan_duration_ms(200.0);
  return settings;
}

bool LoadFrames(const Options& options,
                std::vector<std::vector<InputFrame>>* frames_by_topic) {
  if (frames_by_topic == nullptr) {
    return false;
  }
  std::vector<std::string> topics = {options.primary_topic};
  topics.insert(topics.end(), options.auxiliary_topics.begin(),
                options.auxiliary_topics.end());
  frames_by_topic->assign(topics.size(), {});

  RecordReader reader(options.record);
  if (!reader.IsValid()) {
    std::cerr << "Invalid record: " << options.record << std::endl;
    return false;
  }
  for (const auto& topic : topics) {
    if (reader.GetMessageType(topic) != "apollo.drivers.PointCloud") {
      std::cerr << "Topic is not protobuf PointCloud: " << topic
                << ", type=" << reader.GetMessageType(topic) << std::endl;
      return false;
    }
  }

  const auto time_settings = DefaultTimeSettings();
  RecordMessage message;
  while (reader.ReadMessage(&message)) {
    const auto topic = std::find(topics.begin(), topics.end(),
                                 message.channel_name);
    if (topic == topics.end()) {
      continue;
    }
    const size_t topic_index =
        static_cast<size_t>(std::distance(topics.begin(), topic));
    auto& frames = (*frames_by_topic)[topic_index];
    if (frames.size() >= options.max_frames) {
      continue;
    }

    auto cloud = std::make_shared<PointCloud>();
    TimeContract time_contract;
    if (!cloud->ParseFromString(message.content) ||
        !NormalizePointCloudTime(*cloud, time_settings, &time_contract)) {
      continue;
    }
    frames.push_back(InputFrame{std::move(cloud), time_contract});

    bool complete = true;
    for (const auto& topic_frames : *frames_by_topic) {
      complete = complete && topic_frames.size() >= options.max_frames;
    }
    if (complete) {
      break;
    }
  }

  for (size_t index = 0; index < topics.size(); ++index) {
    auto& frames = (*frames_by_topic)[index];
    std::sort(frames.begin(), frames.end(),
              [](const InputFrame& lhs, const InputFrame& rhs) {
                return lhs.time_contract.canonical_anchor_ns <
                       rhs.time_contract.canonical_anchor_ns;
              });
    if (frames.empty()) {
      std::cerr << "No valid frames loaded for topic: " << topics[index]
                << std::endl;
      return false;
    }
  }
  return true;
}

Eigen::Affine3d MockMapFromBase(double timestamp_sec, double origin_sec,
                                double speed_mps, double yaw_rate_rps) {
  const double elapsed = timestamp_sec - origin_sec;
  return Eigen::Translation3d(speed_mps * elapsed, 0.0, 0.0) *
         Eigen::AngleAxisd(yaw_rate_rps * elapsed, Eigen::Vector3d::UnitZ());
}

Eigen::Affine3d MockBaseFromSensor(size_t sensor_index) {
  if (sensor_index == 0U) {
    return Eigen::Translation3d(1.5, 0.0, 1.8) *
           Eigen::Quaterniond::Identity();
  }
  const double lateral = sensor_index % 2U == 0U ? -1.0 : 1.0;
  return Eigen::Translation3d(0.0, lateral, 1.6) *
         Eigen::Quaterniond::Identity();
}

bool PrepareFrame(const InputFrame& input, size_t sensor_index,
                  const Options& options, double origin_sec,
                  SensorFrameContext* context,
                  std::vector<double>* sample_times,
                  std::vector<Eigen::Affine3d>* poses) {
  if (context == nullptr || sample_times == nullptr || poses == nullptr ||
      input.cloud == nullptr) {
    return false;
  }
  context->sensor_id = input.cloud->frame_id().empty()
                           ? "mock_lidar_" + std::to_string(sensor_index)
                           : input.cloud->frame_id();
  context->point_cloud = input.cloud;
  context->is_primary = sensor_index == 0U;
  context->min_timestamp_sec =
      static_cast<double>(input.time_contract.scan_begin_ns) / 1e9;
  context->max_timestamp_sec =
      static_cast<double>(input.time_contract.scan_end_ns) / 1e9;
  context->fallback_timestamp_sec =
      static_cast<double>(input.time_contract.canonical_anchor_ns -
                          input.time_contract.static_offset_ns) /
      1e9;
  context->timestamp_offset_sec =
      static_cast<double>(input.time_contract.static_offset_ns) / 1e9;
  context->fallback_timestamp_ns = static_cast<uint64_t>(
      input.time_contract.canonical_anchor_ns -
      input.time_contract.static_offset_ns);
  context->timestamp_offset_ns = input.time_contract.static_offset_ns;
  context->all_points_have_timestamps =
      input.time_contract.all_points_have_timestamps;

  const size_t bins =
      input.time_contract.quality == TimestampQuality::kPointTimestamps
          ? options.motion_bins
          : 1U;
  sample_times->resize(bins);
  poses->resize(bins);
  const double begin_sec =
      static_cast<double>(input.time_contract.scan_begin_ns) / 1e9;
  const double end_sec =
      static_cast<double>(input.time_contract.scan_end_ns) / 1e9;
  const Eigen::Affine3d base_from_sensor =
      MockBaseFromSensor(sensor_index);
  for (size_t index = 0; index < bins; ++index) {
    const double ratio =
        bins == 1U ? 0.0
                   : static_cast<double>(index) /
                         static_cast<double>(bins - 1U);
    const double timestamp = begin_sec + ratio * (end_sec - begin_sec);
    (*sample_times)[index] = timestamp;
    (*poses)[index] =
        MockMapFromBase(timestamp, origin_sec, options.speed_mps,
                        options.yaw_rate_rps) *
        base_from_sensor;
  }
  return true;
}

const InputFrame* FindNearestUnconsumed(
    const std::vector<InputFrame>& frames, int64_t reference_ns,
    int64_t max_delta_ns, size_t* next_index) {
  if (next_index == nullptr || *next_index >= frames.size()) {
    return nullptr;
  }
  size_t best_index = frames.size();
  int64_t best_delta = std::numeric_limits<int64_t>::max();
  for (size_t index = *next_index; index < frames.size(); ++index) {
    const int64_t timestamp = frames[index].time_contract.canonical_anchor_ns;
    const int64_t delta = std::llabs(timestamp - reference_ns);
    if (delta < best_delta) {
      best_delta = delta;
      best_index = index;
    }
    if (timestamp > reference_ns && delta > best_delta) {
      break;
    }
  }
  if (best_index == frames.size() || best_delta > max_delta_ns) {
    return nullptr;
  }
  *next_index = best_index + 1U;
  return &frames[best_index];
}

void HashPoint(const PointXYZIT& point, uint64_t* checksum) {
  const int64_t values[] = {
      static_cast<int64_t>(std::llround(point.x() * 1000.0)),
      static_cast<int64_t>(std::llround(point.y() * 1000.0)),
      static_cast<int64_t>(std::llround(point.z() * 1000.0)),
  };
  for (const int64_t value : values) {
    uint64_t bits = static_cast<uint64_t>(value);
    for (size_t byte = 0; byte < sizeof(bits); ++byte) {
      *checksum ^= bits & 0xffU;
      *checksum *= 1099511628211ULL;
      bits >>= 8U;
    }
  }
}

double Percentile(std::vector<double> values, double percentile) {
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const size_t index = static_cast<size_t>(
      std::ceil(percentile * static_cast<double>(values.size())) - 1.0);
  return values[std::min(index, values.size() - 1U)];
}

bool RunBaseline(const Options& options,
                 const std::vector<std::vector<InputFrame>>& frames_by_topic,
                 RunMetrics* metrics) {
  if (metrics == nullptr || frames_by_topic.empty()) {
    return false;
  }

  NonQueryingBuffer mock_buffer;
  LidarUnifiedComponentConfig config;
  config.set_max_full_pointcloud_points(options.max_points);
  config.set_motion_compensation_bins(
      static_cast<uint32_t>(options.motion_bins));
  config.set_enable_ego_query_filter(options.enable_ego_filter);
  config.set_ego_box_forward_x(2.8F);
  config.set_ego_box_backward_x(-2.8F);
  config.set_ego_box_forward_y(1.4F);
  config.set_ego_box_backward_y(-1.4F);
  config.set_enable_voxel_filter(options.enable_voxel_filter);
  config.set_voxel_size(options.voxel_size);

  CpuLidarFusionPolicy fusion;
  CpuLidarFilterPolicy filter;
  if (!fusion.Init(config, &mock_buffer) || !filter.Init(config)) {
    return false;
  }

  std::vector<PointXYZIT> output_points(options.max_points);
  std::vector<size_t> auxiliary_indices(frames_by_topic.size() - 1U, 0U);
  const double origin_sec =
      frames_by_topic.front().front().time_contract.CanonicalAnchorSec();
  const int64_t max_delta_ns =
      static_cast<int64_t>(options.max_delta_ms) * 1000000LL;
  metrics->requested_primary_frames = frames_by_topic.front().size();

  for (const auto& primary : frames_by_topic.front()) {
    std::vector<const InputFrame*> selected = {&primary};
    for (size_t sensor = 1; sensor < frames_by_topic.size(); ++sensor) {
      const InputFrame* auxiliary = FindNearestUnconsumed(
          frames_by_topic[sensor],
          primary.time_contract.canonical_anchor_ns, max_delta_ns,
          &auxiliary_indices[sensor - 1U]);
      if (auxiliary != nullptr) {
        selected.push_back(auxiliary);
      }
    }
    if (selected.size() != frames_by_topic.size()) {
      continue;
    }
    ++metrics->full_match_frames;

    const auto begin = std::chrono::steady_clock::now();
    std::vector<SensorFrameContext> contexts(selected.size());
    std::vector<std::vector<double>> sample_times(selected.size());
    std::vector<std::vector<Eigen::Affine3d>> poses(selected.size());
    size_t required_points = 0;
    for (size_t sensor = 0; sensor < selected.size(); ++sensor) {
      if (!PrepareFrame(*selected[sensor], sensor, options, origin_sec,
                        &contexts[sensor], &sample_times[sensor],
                        &poses[sensor])) {
        return false;
      }
      if (selected[sensor]->time_contract.quality ==
          TimestampQuality::kPointTimestamps) {
        ++metrics->point_timestamp_frames;
      } else {
        ++metrics->fallback_timestamp_frames;
      }
      required_points +=
          static_cast<size_t>(selected[sensor]->cloud->point_size());
    }

    const double reference_sec = primary.time_contract.CanonicalAnchorSec();
    const Eigen::Affine3d map2base_ref =
        MockMapFromBase(reference_sec, origin_sec, options.speed_mps,
                        options.yaw_rate_rps)
            .inverse();
    PointCloudBuffer output;
    output.data_ptr = output_points.data();
    output.capacity = output_points.size();
    output.item_size = sizeof(PointXYZIT);
    output.device_type = MemoryDeviceType::kHost;
    output.device_id = -1;
    const auto fusion_begin = std::chrono::steady_clock::now();
    if (!fusion.FuseToBaseLink(reference_sec, map2base_ref, contexts, poses,
                               sample_times, &output)) {
      return false;
    }
    const auto fusion_end = std::chrono::steady_clock::now();
    const size_t compact_points = output.valid_count;
    size_t ego_filtered_points = 0;
    size_t voxel_filtered_points = 0;
    const size_t output_points_count = filter.ApplyFilters(
        &output, &ego_filtered_points, &voxel_filtered_points);
    const auto end = std::chrono::steady_clock::now();

    if (required_points > output.capacity) {
      ++metrics->truncated_frames;
    }
    metrics->input_points += required_points;
    metrics->compact_points += compact_points;
    metrics->ego_filtered_points += ego_filtered_points;
    metrics->voxel_filtered_points += voxel_filtered_points;
    metrics->output_points += output_points_count;
    for (size_t index = 0; index < output_points_count; ++index) {
      HashPoint(output_points[index], &metrics->checksum);
    }
    metrics->setup_ms.push_back(
        std::chrono::duration<double, std::milli>(fusion_begin - begin)
            .count());
    metrics->fusion_ms.push_back(
        std::chrono::duration<double, std::milli>(fusion_end - fusion_begin)
            .count());
    metrics->filter_ms.push_back(
        std::chrono::duration<double, std::milli>(end - fusion_end).count());
    metrics->latency_ms.push_back(
        std::chrono::duration<double, std::milli>(end - begin).count());
    ++metrics->processed_frames;
  }
  return metrics->processed_frames > 0U && metrics->output_points > 0U;
}

void PrintLatencyStats(const std::vector<double>& values) {
  double sum = 0.0;
  for (const double value : values) {
    sum += value;
  }
  std::cout << "{\"mean\":"
            << sum / static_cast<double>(std::max<size_t>(1U, values.size()))
            << ",\"p50\":" << Percentile(values, 0.50)
            << ",\"p95\":" << Percentile(values, 0.95)
            << ",\"max\":" << Percentile(values, 1.0) << "}";
}

void PrintResult(const Options& options,
                 const std::vector<std::vector<InputFrame>>& frames_by_topic,
                 const RunMetrics& metrics) {
  std::cout << std::fixed << std::setprecision(6)
            << "{"
            << "\"record\":\"" << options.record << "\","
            << "\"sensor_count\":" << frames_by_topic.size() << ","
            << "\"loaded_frames\":[";
  for (size_t index = 0; index < frames_by_topic.size(); ++index) {
    if (index != 0U) {
      std::cout << ",";
    }
    std::cout << frames_by_topic[index].size();
  }
  std::cout << "],"
            << "\"requested_primary_frames\":"
            << metrics.requested_primary_frames << ","
            << "\"processed_frames\":" << metrics.processed_frames << ","
            << "\"full_match_frames\":" << metrics.full_match_frames << ","
            << "\"point_timestamp_frames\":"
            << metrics.point_timestamp_frames << ","
            << "\"fallback_timestamp_frames\":"
            << metrics.fallback_timestamp_frames << ","
            << "\"input_points\":" << metrics.input_points << ","
            << "\"compact_points\":" << metrics.compact_points << ","
            << "\"ego_filtered_points\":" << metrics.ego_filtered_points
            << ","
            << "\"voxel_filtered_points\":"
            << metrics.voxel_filtered_points << ","
            << "\"output_points\":" << metrics.output_points << ","
            << "\"truncated_frames\":" << metrics.truncated_frames << ","
            << "\"mock_speed_mps\":" << options.speed_mps << ","
            << "\"mock_yaw_rate_rps\":" << options.yaw_rate_rps << ","
            << "\"motion_bins\":" << options.motion_bins << ","
            << "\"enable_ego_filter\":"
            << (options.enable_ego_filter ? "true" : "false") << ","
            << "\"enable_voxel_filter\":"
            << (options.enable_voxel_filter ? "true" : "false") << ","
            << "\"voxel_size\":" << options.voxel_size << ","
            << "\"latency_ms\":";
  PrintLatencyStats(metrics.latency_ms);
  std::cout << ",\"stage_ms\":{\"setup\":";
  PrintLatencyStats(metrics.setup_ms);
  std::cout << ",\"fusion\":";
  PrintLatencyStats(metrics.fusion_ms);
  std::cout << ",\"filter\":";
  PrintLatencyStats(metrics.filter_ms);
  std::cout << "},"
            << "\"checksum\":\"" << std::hex << metrics.checksum << std::dec
            << "\""
            << "}" << std::endl;
}

}  // namespace
}  // namespace lidar
}  // namespace drivers
}  // namespace apollo

int main(int argc, char** argv) {
  apollo::drivers::lidar::Options options;
  if (!apollo::drivers::lidar::ParseOptions(argc, argv, &options)) {
    std::cerr
        << "Usage: lidar_fusion_baseline --record=<record> "
           "--primary_topic=<topic> [--auxiliary_topics=<topic,...>] "
           "[--max_frames=100] [--max_points=600000] [--max_delta_ms=80] "
           "[--motion_bins=12] [--speed_mps=5.0] [--yaw_rate_rps=0.05] "
           "[--enable_ego_filter=true|false] "
           "[--enable_voxel_filter=true|false] [--voxel_size=0.15]"
        << std::endl;
    return 2;
  }

  std::vector<std::vector<apollo::drivers::lidar::InputFrame>> frames;
  if (!apollo::drivers::lidar::LoadFrames(options, &frames)) {
    return 1;
  }
  apollo::drivers::lidar::RunMetrics metrics;
  if (!apollo::drivers::lidar::RunBaseline(options, frames, &metrics)) {
    std::cerr << "No valid fused frames produced" << std::endl;
    return 1;
  }
  apollo::drivers::lidar::PrintResult(options, frames, metrics);
  return 0;
}
