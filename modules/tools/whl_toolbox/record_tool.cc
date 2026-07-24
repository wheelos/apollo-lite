/******************************************************************************
 * Copyright 2026 The Wheel.OS Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <glob.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "boost/filesystem.hpp"
#include "gflags/gflags.h"
#include "pcl/io/pcd_io.h"

#include "cyber/record/record_message.h"
#include "cyber/record/record_reader.h"
#include "wheelos_msgs/localization_msgs/localization.pb.h"
#include "wheelos_msgs/sensor_msgs/pointcloud.pb.h"
#include "modules/localization/msf/common/io/pcl_point_types.h"

DEFINE_string(mode, "", "inspect | count | extract");
DEFINE_string(input_path, "", "record file path, directory, or wildcard");
DEFINE_string(topic, "", "topic for count mode");
DEFINE_string(pointcloud_topic, "", "pointcloud topic for extract mode");
DEFINE_string(pose_topic, "", "pose topic for extract mode");
DEFINE_string(output_dir, "", "output dir for extract mode");
DEFINE_double(pose_match_threshold_sec, 0.05,
              "max timestamp delta when matching pose");
DEFINE_int32(max_frames, 0, "max frames to export for extract mode");
DEFINE_string(progress_file, "", "json progress output path");

namespace apollo {
namespace whl_toolbox {

namespace bfs = boost::filesystem;
using apollo::cyber::record::RecordMessage;
using apollo::cyber::record::RecordReader;
using apollo::drivers::PointCloud;
using apollo::localization::LocalizationEstimate;
using apollo::localization::msf::velodyne::PointXYZIT;

struct ChannelInfo {
  std::string topic;
  std::string message_type;
  uint64_t message_count = 0;
};

struct LocalizationSample {
  double timestamp_sec = 0.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double qw = 1.0;
};

std::string JsonEscape(const std::string& input) {
  std::ostringstream out;
  for (unsigned char ch : input) {
    switch (ch) {
      case '\\':
        out << "\\\\";
        break;
      case '"':
        out << "\\\"";
        break;
      case '\b':
        out << "\\b";
        break;
      case '\f':
        out << "\\f";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        if (ch < 0x20) {
          out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
              << static_cast<int>(ch) << std::dec << std::setfill(' ');
        } else {
          out << static_cast<char>(ch);
        }
    }
  }
  return out.str();
}

std::vector<std::string> ExpandRecordPaths(const std::string& raw_path) {
  std::vector<std::string> results;
  if (raw_path.empty()) {
    return results;
  }

  glob_t glob_result {};
  if (glob(raw_path.c_str(), 0, nullptr, &glob_result) == 0) {
    for (size_t i = 0; i < glob_result.gl_pathc; ++i) {
      bfs::path path(glob_result.gl_pathv[i]);
      if (bfs::is_regular_file(path) &&
          path.filename().string().find(".record.") != std::string::npos) {
        results.push_back(bfs::absolute(path).string());
      }
    }
    globfree(&glob_result);
    if (!results.empty()) {
      std::sort(results.begin(), results.end());
      return results;
    }
  }

  bfs::path path(raw_path);
  if (!path.is_absolute()) {
    path = bfs::absolute(path);
  }
  if (bfs::is_regular_file(path)) {
    results.push_back(path.string());
    return results;
  }
  if (bfs::is_directory(path)) {
    for (bfs::directory_iterator it(path), end; it != end; ++it) {
      if (!bfs::is_regular_file(it->path())) {
        continue;
      }
      if (it->path().filename().string().find(".record.") == std::string::npos) {
        continue;
      }
      results.push_back(bfs::absolute(it->path()).string());
    }
    std::sort(results.begin(), results.end());
    return results;
  }
  return results;
}

double ResolvePointCloudTimestampSec(const PointCloud& message,
                                     uint64_t record_time_ns) {
  if (message.has_measurement_time() && message.measurement_time() > 0.0) {
    return message.measurement_time();
  }
  if (message.has_header() && message.header().timestamp_sec() > 0.0) {
    return message.header().timestamp_sec();
  }
  return static_cast<double>(record_time_ns) * 1e-9;
}

double ResolveLocalizationTimestampSec(const LocalizationEstimate& message,
                                       uint64_t record_time_ns) {
  if (message.has_measurement_time() && message.measurement_time() > 0.0) {
    return message.measurement_time();
  }
  if (message.has_header() && message.header().timestamp_sec() > 0.0) {
    return message.header().timestamp_sec();
  }
  return static_cast<double>(record_time_ns) * 1e-9;
}

void WriteProgress(const std::string& stage, size_t current, size_t total,
                   const std::string& message, bool done) {
  if (FLAGS_progress_file.empty()) {
    return;
  }
  std::ofstream fout(FLAGS_progress_file, std::ios::out | std::ios::trunc);
  if (!fout.is_open()) {
    return;
  }
  const double percent =
      total == 0 ? (done ? 100.0 : 0.0)
                 : 100.0 * static_cast<double>(current) /
                       static_cast<double>(std::max<size_t>(1, total));
  fout << "{"
       << "\"stage\":\"" << JsonEscape(stage) << "\","
       << "\"current\":" << current << ","
       << "\"total\":" << total << ","
       << "\"percent\":" << std::fixed << std::setprecision(2) << percent
       << ","
       << "\"done\":" << (done ? "true" : "false") << ","
       << "\"message\":\"" << JsonEscape(message) << "\""
       << "}\n";
}

std::string FilesToJson(const std::vector<std::string>& files) {
  std::ostringstream out;
  out << "[";
  for (size_t i = 0; i < files.size(); ++i) {
    if (i != 0) {
      out << ",";
    }
    out << "\"" << JsonEscape(files[i]) << "\"";
  }
  out << "]";
  return out.str();
}

std::string StringSetToJson(const std::set<std::string>& values) {
  std::ostringstream out;
  out << "[";
  size_t index = 0;
  for (const auto& value : values) {
    if (index++ != 0) {
      out << ",";
    }
    out << "\"" << JsonEscape(value) << "\"";
  }
  out << "]";
  return out.str();
}

std::string ChannelsToJson(const std::map<std::string, ChannelInfo>& channels) {
  std::ostringstream out;
  out << "[";
  size_t index = 0;
  for (const auto& item : channels) {
    if (index++ != 0) {
      out << ",";
    }
    out << "{"
        << "\"topic\":\"" << JsonEscape(item.second.topic) << "\","
        << "\"message_type\":\"" << JsonEscape(item.second.message_type)
        << "\","
        << "\"message_count\":" << item.second.message_count
        << "}";
  }
  out << "]";
  return out.str();
}

std::vector<LocalizationSample> LoadLocalizationSamples(
    const std::vector<std::string>& files, const std::string& topic) {
  std::vector<LocalizationSample> samples;
  if (topic.empty()) {
    return samples;
  }
  for (const auto& file : files) {
    RecordReader reader(file);
    RecordMessage message;
    while (reader.ReadMessage(&message)) {
      if (message.channel_name != topic) {
        continue;
      }
      LocalizationEstimate pose;
      if (!pose.ParseFromString(message.content)) {
        continue;
      }
      if (!pose.has_pose() || !pose.pose().has_position() ||
          !pose.pose().has_orientation()) {
        continue;
      }
      LocalizationSample sample;
      sample.timestamp_sec =
          ResolveLocalizationTimestampSec(pose, message.time);
      sample.x = pose.pose().position().x();
      sample.y = pose.pose().position().y();
      sample.z = pose.pose().position().z();
      sample.qx = pose.pose().orientation().qx();
      sample.qy = pose.pose().orientation().qy();
      sample.qz = pose.pose().orientation().qz();
      sample.qw = pose.pose().orientation().qw();
      samples.push_back(sample);
    }
  }
  std::sort(samples.begin(), samples.end(),
            [](const LocalizationSample& lhs, const LocalizationSample& rhs) {
              return lhs.timestamp_sec < rhs.timestamp_sec;
            });
  return samples;
}

const LocalizationSample* MatchLocalization(
    const std::vector<LocalizationSample>& samples, double timestamp_sec,
    double threshold_sec) {
  if (samples.empty()) {
    return nullptr;
  }
  auto it = std::lower_bound(
      samples.begin(), samples.end(), timestamp_sec,
      [](const LocalizationSample& sample, double value) {
        return sample.timestamp_sec < value;
      });
  const LocalizationSample* best = nullptr;
  if (it != samples.end()) {
    best = &(*it);
  }
  if (it != samples.begin()) {
    const LocalizationSample* previous = &(*(it - 1));
    if (best == nullptr ||
        std::abs(previous->timestamp_sec - timestamp_sec) <
            std::abs(best->timestamp_sec - timestamp_sec)) {
      best = previous;
    }
  }
  if (best == nullptr ||
      std::abs(best->timestamp_sec - timestamp_sec) > threshold_sec) {
    return nullptr;
  }
  return best;
}

int RunInspect() {
  const auto files = ExpandRecordPaths(FLAGS_input_path);
  if (files.empty()) {
    std::cerr << "No record files found for input_path: " << FLAGS_input_path
              << std::endl;
    return 1;
  }

  std::map<std::string, ChannelInfo> channels;
  std::set<std::string> pointcloud_topics;
  std::set<std::string> localization_topics;
  for (const auto& file : files) {
    RecordReader reader(file);
    const auto channel_list = reader.GetChannelList();
    for (const auto& channel : channel_list) {
      auto& info = channels[channel];
      info.topic = channel;
      info.message_type = reader.GetMessageType(channel);
      info.message_count += reader.GetMessageNumber(channel);
      if (info.message_type.find("PointCloud") != std::string::npos ||
          channel.find("PointCloud") != std::string::npos ||
          channel.find("pointcloud") != std::string::npos) {
        pointcloud_topics.insert(channel);
      }
      if (info.message_type.find("LocalizationEstimate") != std::string::npos ||
          channel.find("localization/pose") != std::string::npos) {
        localization_topics.insert(channel);
      }
    }
  }

  std::cout << "{"
            << "\"files\":" << FilesToJson(files) << ","
            << "\"channels\":" << ChannelsToJson(channels) << ","
            << "\"pointcloud_topics\":" << StringSetToJson(pointcloud_topics)
            << ","
            << "\"localization_topics\":"
            << StringSetToJson(localization_topics) << "}" << std::endl;
  return 0;
}

int RunCount() {
  if (FLAGS_topic.empty()) {
    std::cerr << "--topic is required for count mode" << std::endl;
    return 1;
  }
  const auto files = ExpandRecordPaths(FLAGS_input_path);
  if (files.empty()) {
    std::cerr << "No record files found for input_path: " << FLAGS_input_path
              << std::endl;
    return 1;
  }
  uint64_t total = 0;
  for (const auto& file : files) {
    RecordReader reader(file);
    total += reader.GetMessageNumber(FLAGS_topic);
  }
  std::cout << "{"
            << "\"files\":" << FilesToJson(files) << ","
            << "\"topic\":\"" << JsonEscape(FLAGS_topic) << "\","
            << "\"count\":" << total << "}" << std::endl;
  return 0;
}

int RunExtract() {
  if (FLAGS_pointcloud_topic.empty()) {
    std::cerr << "--pointcloud_topic is required for extract mode"
              << std::endl;
    return 1;
  }
  if (FLAGS_output_dir.empty()) {
    std::cerr << "--output_dir is required for extract mode" << std::endl;
    return 1;
  }

  const auto files = ExpandRecordPaths(FLAGS_input_path);
  if (files.empty()) {
    std::cerr << "No record files found for input_path: " << FLAGS_input_path
              << std::endl;
    return 1;
  }

  bfs::path output_dir(FLAGS_output_dir);
  bfs::path pcd_dir = output_dir / "pcd";
  bfs::path pose_dir = output_dir / "pose";
  bfs::create_directories(pcd_dir);
  if (!FLAGS_pose_topic.empty()) {
    bfs::create_directories(pose_dir);
  }

  const auto localization_samples =
      LoadLocalizationSamples(files, FLAGS_pose_topic);

  size_t total_frames = 0;
  for (const auto& file : files) {
    RecordReader reader(file);
    total_frames +=
        static_cast<size_t>(reader.GetMessageNumber(FLAGS_pointcloud_topic));
  }
  if (FLAGS_max_frames > 0 &&
      total_frames > static_cast<size_t>(FLAGS_max_frames)) {
    total_frames = static_cast<size_t>(FLAGS_max_frames);
  }

  size_t exported = 0;
  size_t exported_with_pose = 0;
  WriteProgress("prepare", 0, total_frames, "record files listed", false);
  for (const auto& file : files) {
    RecordReader reader(file);
    RecordMessage message;
    while (reader.ReadMessage(&message)) {
      if (message.channel_name != FLAGS_pointcloud_topic) {
        continue;
      }
      PointCloud cloud_message;
      if (!cloud_message.ParseFromString(message.content)) {
        continue;
      }
      char frame_name_buffer[64];
      std::snprintf(frame_name_buffer, sizeof(frame_name_buffer),
                    "frame_%06zu", exported);
      const std::string frame_name(frame_name_buffer);
      const bfs::path pcd_path = pcd_dir / (frame_name + ".pcd");

      pcl::PointCloud<PointXYZIT> cloud;
      cloud.width = 1;
      cloud.height = static_cast<uint32_t>(cloud_message.point_size());
      cloud.is_dense = false;
      cloud.points.reserve(cloud_message.point_size());
      for (int i = 0; i < cloud_message.point_size(); ++i) {
        const auto& pt = cloud_message.point(i);
        if (std::isnan(pt.x()) || std::isnan(pt.y()) || std::isnan(pt.z())) {
          continue;
        }
        PointXYZIT point;
        point.x = pt.x();
        point.y = pt.y();
        point.z = pt.z();
        point.intensity = static_cast<unsigned char>(
            std::max(0, std::min(255, static_cast<int>(pt.intensity()))));
        point.timestamp = static_cast<double>(pt.timestamp()) * 1e-9;
        cloud.points.push_back(point);
      }
      cloud.height = 1;
      cloud.width = static_cast<uint32_t>(cloud.points.size());
      pcl::io::savePCDFileBinaryCompressed(pcd_path.string(), cloud);

      const double timestamp_sec =
          ResolvePointCloudTimestampSec(cloud_message, message.time);
      if (!FLAGS_pose_topic.empty()) {
        const auto* matched = MatchLocalization(
            localization_samples, timestamp_sec,
            FLAGS_pose_match_threshold_sec);
        if (matched != nullptr) {
          const bfs::path pose_path = pose_dir / (frame_name + ".pose");
          std::ofstream pose_out(pose_path.string());
          pose_out << exported << " " << std::fixed << std::setprecision(9)
                   << timestamp_sec << " " << matched->x << " " << matched->y
                   << " " << matched->z << " " << matched->qx << " "
                   << matched->qy << " " << matched->qz << " " << matched->qw
                   << "\n";
          ++exported_with_pose;
        }
      }

      ++exported;
      WriteProgress("extract", exported, total_frames, frame_name,
                    exported == total_frames);
      if (FLAGS_max_frames > 0 &&
          exported >= static_cast<size_t>(FLAGS_max_frames)) {
        break;
      }
    }
    if (FLAGS_max_frames > 0 &&
        exported >= static_cast<size_t>(FLAGS_max_frames)) {
      break;
    }
  }

  std::ostringstream summary;
  summary << "{"
          << "\"record_files\":" << FilesToJson(files) << ","
          << "\"pointcloud_topic\":\"" << JsonEscape(FLAGS_pointcloud_topic)
          << "\","
          << "\"pose_topic\":\"" << JsonEscape(FLAGS_pose_topic) << "\","
          << "\"total_frames\":" << exported << ","
          << "\"frames_with_pose\":" << exported_with_pose << ","
          << "\"pcd_dir\":\"" << JsonEscape(pcd_dir.string()) << "\","
          << "\"pose_dir\":\"" << JsonEscape(FLAGS_pose_topic.empty()
                                                 ? ""
                                                 : pose_dir.string())
          << "\""
          << "}";
  std::ofstream summary_out((output_dir / "extract_summary.json").string());
  summary_out << summary.str() << "\n";
  std::cout << summary.str() << std::endl;
  return 0;
}

}  // namespace whl_toolbox
}  // namespace apollo

int main(int argc, char** argv) {
  google::SetUsageMessage("record_tool --mode=inspect|count|extract ...");
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (FLAGS_mode == "inspect") {
    return apollo::whl_toolbox::RunInspect();
  }
  if (FLAGS_mode == "count") {
    return apollo::whl_toolbox::RunCount();
  }
  if (FLAGS_mode == "extract") {
    return apollo::whl_toolbox::RunExtract();
  }

  std::cerr << "Unsupported mode: " << FLAGS_mode << std::endl;
  return 1;
}
