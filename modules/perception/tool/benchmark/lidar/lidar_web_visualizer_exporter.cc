#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <set>
#include <string>
#include <vector>

#include <boost/program_options.hpp>
#include "nlohmann/json.hpp"

#include "cyber/common/file.h"
#include "modules/perception/tool/benchmark/lidar/base/frame.h"
#include "modules/perception/tool/benchmark/lidar/eval/frame_statistics.h"
#include "modules/perception/tool/benchmark/lidar/eval/lidar_option.h"
#include "modules/perception/tool/benchmark/lidar/loader/sequence_data_loader.h"

namespace apollo {
namespace perception {
namespace benchmark {

namespace internal {

using Json = nlohmann::json;

struct ExportOptions {
  std::string cloud;
  std::string result;
  std::string groundtruth;
  std::string output;
  std::string reserve;
  bool is_folder = false;
  std::size_t max_points = 30000;
};

struct MatchMetadata {
  std::vector<std::string> det_status;
  std::vector<std::string> gt_status;
  std::vector<int> det_match;
  std::vector<int> gt_match;
  std::vector<double> det_best_jaccard;
  std::vector<double> gt_best_jaccard;
  std::vector<unsigned int> det_best_overlap;
  std::vector<unsigned int> gt_best_overlap;
  int strict_match_count = 0;
  int fp_count = 0;
  int fn_count = 0;
  int underseg_count = 0;
};

std::string GetFrameLabel(const std::string& path) {
  return cyber::common::GetFileName(path, true);
}

Json Vec3(double x, double y, double z) {
  return Json::array({x, y, z});
}

Json ToPointArray(const PointCloudConstPtr& cloud, std::size_t max_points) {
  Json points = Json::array();
  if (cloud == nullptr || cloud->empty() || max_points == 0) {
    return points;
  }
  const std::size_t sample_count = std::min<std::size_t>(cloud->size(), max_points);
  const std::size_t stride =
      std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(
                                   static_cast<double>(cloud->size()) /
                                   static_cast<double>(sample_count))));
  for (std::size_t i = 0; i < cloud->size() && points.size() < sample_count;
       i += stride) {
    const auto& point = cloud->at(i);
    points.push_back(Json::array({point.x, point.y, point.z}));
  }
  return points;
}

Json SerializeObject(const ObjectPtr& object, const std::string& role,
                     const std::string& status, int matched_index,
                     double jaccard, unsigned int overlap) {
  Json json = {
      {"role", role},
      {"id", object->id},
      {"track_id", object->track_id},
      {"type", translate_type_to_string(object->type)},
      {"confidence", object->confidence},
      {"center", Vec3(object->center(0), object->center(1), object->center(2))},
      {"size", Vec3(object->length, object->width, object->height)},
      {"yaw", object->yaw},
      {"velocity",
       Vec3(object->velocity(0), object->velocity(1), object->velocity(2))},
      {"status", status},
      {"matched_index", matched_index},
      {"jaccard", jaccard},
      {"matched_points", overlap},
      {"is_background", object->is_background},
      {"is_in_roi", object->is_in_roi},
      {"is_in_main_lanes", object->is_in_main_lanes},
  };
  return json;
}

MatchMetadata BuildMatchMetadata(const FrameStatistics& frame) {
  MatchMetadata meta;
  const auto& detections = frame.get_objects();
  const auto& groundtruths = frame.get_gt_objects();
  const auto& matches = frame.get_matches();
  const auto& strict_match_indices = frame.get_strict_match_indices();
  const auto& isolated_det = frame.get_isolated_object_indices_2017();
  const auto& isolated_gt = frame.get_isolated_gt_object_indices_2017();
  const auto& underseg_gt = frame.get_underseg_gt_object_indices_2017();

  meta.det_status.assign(detections.size(), "fp");
  meta.gt_status.assign(groundtruths.size(), "fn");
  meta.det_match.assign(detections.size(), -1);
  meta.gt_match.assign(groundtruths.size(), -1);
  meta.det_best_jaccard.assign(detections.size(), 0.0);
  meta.gt_best_jaccard.assign(groundtruths.size(), 0.0);
  meta.det_best_overlap.assign(detections.size(), 0);
  meta.gt_best_overlap.assign(groundtruths.size(), 0);

  for (const auto& match : matches) {
    if (match.second < meta.det_best_jaccard.size() &&
        match.jaccard_index >= meta.det_best_jaccard[match.second]) {
      meta.det_best_jaccard[match.second] = match.jaccard_index;
      meta.det_best_overlap[match.second] = match.matched_point_num;
      meta.det_match[match.second] = static_cast<int>(match.first);
    }
    if (match.first < meta.gt_best_jaccard.size() &&
        match.jaccard_index >= meta.gt_best_jaccard[match.first]) {
      meta.gt_best_jaccard[match.first] = match.jaccard_index;
      meta.gt_best_overlap[match.first] = match.matched_point_num;
      meta.gt_match[match.first] = static_cast<int>(match.second);
    }
  }

  for (auto match_index : strict_match_indices) {
    if (match_index >= matches.size()) {
      continue;
    }
    const auto& match = matches[match_index];
    if (match.second < meta.det_status.size()) {
      meta.det_status[match.second] = "tp";
      meta.det_match[match.second] = static_cast<int>(match.first);
      meta.det_best_jaccard[match.second] = match.jaccard_index;
      meta.det_best_overlap[match.second] = match.matched_point_num;
    }
    if (match.first < meta.gt_status.size()) {
      meta.gt_status[match.first] = "matched";
      meta.gt_match[match.first] = static_cast<int>(match.second);
      meta.gt_best_jaccard[match.first] = match.jaccard_index;
      meta.gt_best_overlap[match.first] = match.matched_point_num;
    }
  }

  std::set<unsigned int> underseg_set(underseg_gt.begin(), underseg_gt.end());
  for (auto gt_index : isolated_gt) {
    if (gt_index < meta.gt_status.size() && underseg_set.count(gt_index) > 0) {
      meta.gt_status[gt_index] = "underseg";
    }
  }

  meta.strict_match_count = static_cast<int>(strict_match_indices.size());
  meta.fp_count = static_cast<int>(isolated_det.size());
  meta.underseg_count = static_cast<int>(underseg_gt.size());
  meta.fn_count =
      static_cast<int>(isolated_gt.size()) - static_cast<int>(underseg_gt.size());
  return meta;
}

Json BuildFrameJson(const Frame& frame, std::size_t frame_index,
                    std::size_t max_points) {
  Json frame_json = {
      {"frame_index", frame_index},
      {"frame_label", GetFrameLabel(frame.get_name())},
      {"has_groundtruth", false},
      {"cloud", ToPointArray(frame.get_point_cloud(), max_points)},
      {"cloud_total_points",
       frame.get_point_cloud() == nullptr
           ? 0
           : static_cast<int>(frame.get_point_cloud()->size())},
      {"detections", Json::array()},
      {"groundtruths", Json::array()},
      {"stats",
       {{"detections", static_cast<int>(frame.get_objects().size())},
        {"groundtruths", 0},
        {"strict_matches", 0},
        {"false_positives", 0},
        {"false_negatives", 0},
        {"underseg", 0},
        {"precision", 0.0},
        {"recall", 0.0}}}};

  for (const auto& object : frame.get_objects()) {
    frame_json["detections"].push_back(
        SerializeObject(object, "detection", "detection", -1, 0.0, 0));
  }
  return frame_json;
}

Json BuildFrameJson(const FrameStatistics& frame, std::size_t frame_index,
                    std::size_t max_points) {
  const MatchMetadata meta = BuildMatchMetadata(frame);
  Json frame_json = {
      {"frame_index", frame_index},
      {"frame_label", GetFrameLabel(frame.get_name())},
      {"has_groundtruth", true},
      {"cloud", ToPointArray(frame.get_point_cloud(), max_points)},
      {"cloud_total_points",
       frame.get_point_cloud() == nullptr
           ? 0
           : static_cast<int>(frame.get_point_cloud()->size())},
      {"detections", Json::array()},
      {"groundtruths", Json::array()},
      {"stats",
       {{"detections", static_cast<int>(frame.get_objects().size())},
        {"groundtruths", static_cast<int>(frame.get_gt_objects().size())},
        {"strict_matches", meta.strict_match_count},
        {"false_positives", meta.fp_count},
        {"false_negatives", meta.fn_count},
        {"underseg", meta.underseg_count},
        {"precision",
         (meta.strict_match_count + meta.fp_count) > 0
             ? static_cast<double>(meta.strict_match_count) /
                   static_cast<double>(meta.strict_match_count + meta.fp_count)
             : 0.0},
        {"recall",
         !frame.get_gt_objects().empty()
             ? static_cast<double>(meta.strict_match_count) /
                   static_cast<double>(frame.get_gt_objects().size())
             : 0.0}}}};

  for (std::size_t i = 0; i < frame.get_objects().size(); ++i) {
    frame_json["detections"].push_back(SerializeObject(
        frame.get_objects()[i], "detection", meta.det_status[i],
        meta.det_match[i], meta.det_best_jaccard[i], meta.det_best_overlap[i]));
  }
  for (std::size_t i = 0; i < frame.get_gt_objects().size(); ++i) {
    frame_json["groundtruths"].push_back(SerializeObject(
        frame.get_gt_objects()[i], "groundtruth", meta.gt_status[i],
        meta.gt_match[i], meta.gt_best_jaccard[i], meta.gt_best_overlap[i]));
  }
  return frame_json;
}

bool WriteJsonFile(const std::string& path, const Json& json) {
  std::ofstream out(path);
  if (!out.is_open()) {
    std::cerr << "Failed to open json output: " << path << std::endl;
    return false;
  }
  out << std::setw(2) << json << std::endl;
  return true;
}

template <typename FrameType>
bool InitLoader(SequenceDataLoader<FrameType>* loader,
                const ExportOptions& options) {
  std::vector<std::string> inputs = {options.cloud, options.result};
  if (!options.groundtruth.empty()) {
    inputs.push_back(options.groundtruth);
  }
  return options.is_folder ? loader->init_loader_with_folder(inputs)
                           : loader->init_loader_with_list(inputs);
}

bool ExportWithoutGroundTruth(const ExportOptions& options) {
  SequenceDataLoader<Frame> loader;
  if (!InitLoader(&loader, options)) {
    std::cerr << "Failed to initialize loader for detection-only export"
              << std::endl;
    return false;
  }

  Json manifest = {
      {"title", "Apollo LiDAR Offline Viewer"},
      {"has_groundtruth", false},
      {"frame_count", static_cast<int>(loader.size())},
      {"max_points_per_frame", static_cast<int>(options.max_points)},
      {"frames", Json::array()},
  };

  std::shared_ptr<Frame> frame;
  std::size_t frame_index = 0;
  while (loader.query_next(frame)) {
    Json frame_json = BuildFrameJson(*frame, frame_index, options.max_points);
    const std::string frame_file =
        "frames/frame_" + std::to_string(frame_index) + ".json";
    manifest["frames"].push_back(
        {{"index", static_cast<int>(frame_index)},
         {"label", frame_json["frame_label"]},
         {"file", frame_file},
         {"stats", frame_json["stats"]}});
    if (!WriteJsonFile(
            cyber::common::GetAbsolutePath(options.output, frame_file),
            frame_json)) {
      return false;
    }
    frame->release();
    ++frame_index;
  }

  return WriteJsonFile(
      cyber::common::GetAbsolutePath(options.output, "manifest.json"), manifest);
}

bool ExportWithGroundTruth(const ExportOptions& options) {
  LidarOption lidar_option;
  lidar_option.parse_from_string(options.reserve);
  lidar_option.set_options();

  SequenceDataLoader<FrameStatistics> loader;
  if (!InitLoader(&loader, options)) {
    std::cerr << "Failed to initialize loader for visualization export"
              << std::endl;
    return false;
  }

  Json manifest = {
      {"title", "Apollo LiDAR Offline Viewer"},
      {"has_groundtruth", true},
      {"frame_count", static_cast<int>(loader.size())},
      {"max_points_per_frame", static_cast<int>(options.max_points)},
      {"reserve", options.reserve},
      {"frames", Json::array()},
  };

  std::shared_ptr<FrameStatistics> frame;
  std::size_t frame_index = 0;
  while (loader.query_next(frame)) {
    frame->find_association();
    frame->cal_meta_statistics();

    Json frame_json = BuildFrameJson(*frame, frame_index, options.max_points);
    const std::string frame_file =
        "frames/frame_" + std::to_string(frame_index) + ".json";
    manifest["frames"].push_back(
        {{"index", static_cast<int>(frame_index)},
         {"label", frame_json["frame_label"]},
         {"file", frame_file},
         {"stats", frame_json["stats"]}});
    if (!WriteJsonFile(
            cyber::common::GetAbsolutePath(options.output, frame_file),
            frame_json)) {
      return false;
    }
    frame->release();
    ++frame_index;
  }

  return WriteJsonFile(
      cyber::common::GetAbsolutePath(options.output, "manifest.json"), manifest);
}

}  // namespace internal

}  // namespace benchmark
}  // namespace perception
}  // namespace apollo

int main(int argc, char** argv) {
  namespace po = boost::program_options;

  po::options_description desc("Allowed options");
  desc.add_options()("help", "produce help message")(
      "cloud", po::value<std::string>(), "point cloud folder or list")(
      "result", po::value<std::string>(), "result folder or list")(
      "groundtruth", po::value<std::string>()->default_value(""),
      "groundtruth folder or list")(
      "is_folder", po::value<bool>()->default_value(false),
      "interpret input paths as folders instead of lists")(
      "output", po::value<std::string>(), "viewer output directory")(
      "reserve", po::value<std::string>()->default_value(""),
      "benchmark reserve string, e.g. JACCARD:0.7|CONFIDENCE:0.2")(
      "max_points", po::value<std::size_t>()->default_value(30000),
      "maximum point count exported for each frame");

  po::variables_map args;
  po::store(po::parse_command_line(argc, argv, desc), args);
  po::notify(args);

  if (args.count("help") == 1 || args.count("cloud") == 0 ||
      args.count("result") == 0 || args.count("output") == 0) {
    std::cout << desc << std::endl;
    return 0;
  }

  apollo::perception::benchmark::internal::ExportOptions options;
  options.cloud = args["cloud"].as<std::string>();
  options.result = args["result"].as<std::string>();
  options.groundtruth = args["groundtruth"].as<std::string>();
  options.output = args["output"].as<std::string>();
  options.reserve = args["reserve"].as<std::string>();
  options.is_folder = args["is_folder"].as<bool>();
  options.max_points = args["max_points"].as<std::size_t>();

  if (!apollo::cyber::common::EnsureDirectory(options.output) ||
      !apollo::cyber::common::EnsureDirectory(
          apollo::cyber::common::GetAbsolutePath(options.output, "frames"))) {
    std::cerr << "Failed to create output directory: " << options.output
              << std::endl;
    return 1;
  }

  const bool ok = options.groundtruth.empty()
                      ? apollo::perception::benchmark::internal::
                            ExportWithoutGroundTruth(options)
                      : apollo::perception::benchmark::internal::
                            ExportWithGroundTruth(options);
  return ok ? 0 : 1;
}
