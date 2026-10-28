// Copyright 2026 The Apollo Authors. All Rights Reserved.
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

#include "modules/traffic_light/pipeline/traffic_light_pipeline.h"
#include "modules/traffic_light/ports/default_provider_ports.h"

namespace apollo {
namespace traffic_light {
namespace {

void Require(bool valid, const std::string& message) {
  if (!valid) throw std::runtime_error(message);
}

std::vector<std::string> Row(std::istream& input, size_t count) {
  std::string line, field;
  Require(static_cast<bool>(std::getline(input, line)), "truncated TSV");
  std::istringstream stream(line);
  std::vector<std::string> fields;
  while (std::getline(stream, field, '\t')) fields.push_back(field);
  Require(fields.size() == count && !line.empty() && line.back() != '\t',
          "invalid TSV column count");
  return fields;
}

template <typename T>
T Number(const std::string& text) {
  Require(!text.empty() && text.front() != '-' &&
              text.find_first_of(" \r\n") == std::string::npos,
          "invalid unsigned/geometry field");
  std::istringstream stream(text);
  T value{};
  stream >> value;
  Require(!stream.fail() && stream.eof() &&
              std::isfinite(static_cast<double>(value)),
          "invalid number");
  return value;
}

bool Geometry(const Rect2f& box, int width = 0, int height = 0) {
  return std::isfinite(box.x) && std::isfinite(box.y) &&
         std::isfinite(box.Area()) && box.width > 0 && box.height > 0 &&
         (width == 0 ||
          (box.x >= 0 && box.y >= 0 && box.x + box.width <= width + 0.001f &&
           box.y + box.height <= height + 0.001f));
}

float Intersection(const Rect2f& a, const Rect2f& b) {
  return std::max(0.0f,
                  std::min(a.x + a.width, b.x + b.width) - std::max(a.x, b.x)) *
         std::max(0.0f, std::min(a.y + a.height, b.y + b.height) -
                            std::max(a.y, b.y));
}

Rect2f Expand(Rect2f box, float sigma, float ratio, int w, int h) {
  const float cx = box.x + box.width / 2, cy = box.y + box.height / 2;
  const float hw = (box.width + 4 * sigma) * ratio / 2;
  const float hh = (box.height + 4 * sigma) * ratio / 2;
  const float x = std::max(0.0f, cx - hw), y = std::max(0.0f, cy - hh);
  return {x, y, std::min(static_cast<float>(w), cx + hw) - x,
          std::min(static_cast<float>(h), cy + hh) - y};
}

struct Truth {
  std::string id;
  Rect2f box;
  LightColor color;
};

struct Metrics {
  int frames = 0, observations = 0, matches = 0, switches = 0, ambiguous = 0;
  int history_covered = 0, static_covered = 0, history_rois = 0,
      static_rois = 0;
  int exploration_frames = 0, prediction_frames = 0;
  double history_area = 0, static_area = 0, image_area = 0;
};

void Evaluate(const PipelineContext& context, const std::vector<Truth>& truth,
              const std::map<int, Rect2f>& previous, float ratio,
              std::map<std::string, int>* identities, Metrics* metrics) {
  const auto& image = context.camera_frames.front().image;
  std::vector<Rect2f> history, baseline;
  for (const auto& prompt : context.prompts) {
    Require(Geometry(prompt.roi_box, image.cols, image.rows), "invalid prompt");
    if (prompt.source == PromptSource::HISTORY)
      history.push_back(prompt.roi_box);
  }
  for (const auto& prediction : context.track_predictions) {
    Require(Geometry(prediction.bbox) &&
                std::isfinite(prediction.position_stddev_px) &&
                prediction.position_stddev_px >= 0,
            "invalid forecast");
    const auto found = previous.find(prediction.track_id);
    if (found != previous.end()) {
      baseline.push_back(Expand(found->second, prediction.position_stddev_px,
                                ratio, image.cols, image.rows));
    }
  }
  metrics->history_rois += history.size();
  metrics->static_rois += baseline.size();
  for (const auto& box : history) metrics->history_area += box.Area();
  for (const auto& box : baseline) metrics->static_area += box.Area();
  metrics->image_area += static_cast<double>(image.cols) * image.rows;
  metrics->prediction_frames += !history.empty();
  metrics->exploration_frames += context.metrics.prompter.fallback_used;
  for (const auto& gt : truth) {
    auto covered = [&](const std::vector<Rect2f>& boxes) {
      return std::any_of(boxes.begin(), boxes.end(), [&](const Rect2f& box) {
        return box.x <= gt.box.x && box.y <= gt.box.y &&
               box.x + box.width >= gt.box.x + gt.box.width &&
               box.y + box.height >= gt.box.y + gt.box.height;
      });
    };
    metrics->history_covered += covered(history);
    metrics->static_covered += covered(baseline);
  }
  std::vector<std::tuple<float, size_t, size_t>> edges;
  for (size_t t = 0; t < context.tracked_lights.size(); ++t) {
    const auto& track = context.tracked_lights[t];
    const auto& visual = track.current_state.visual_light;
    Require(
        Geometry(visual.bbox) && track.track_id > 0 && visual.signal_id.empty(),
        "invalid/unproven track identity");
    if (track.lost_frames != 0) continue;
    for (size_t g = 0; g < truth.size(); ++g) {
      if (truth[g].id == "-") continue;
      float inter = Intersection(visual.bbox, truth[g].box);
      float iou = inter / (visual.bbox.Area() + truth[g].box.Area() - inter);
      if (iou >= 0.5f) edges.emplace_back(-iou, t, g);
    }
  }
  std::sort(edges.begin(), edges.end());
  std::set<size_t> tracks, labels;
  for (const auto& edge : edges) {
    const size_t t = std::get<1>(edge), g = std::get<2>(edge);
    if (tracks.count(t) || labels.count(g)) continue;
    tracks.insert(t);
    labels.insert(g);
    int id = context.tracked_lights[t].track_id;
    auto found = identities->find(truth[g].id);
    metrics->switches += found != identities->end() && found->second != id;
    (*identities)[truth[g].id] = id;
    ++metrics->matches;
  }
  ++metrics->frames;
  metrics->observations += truth.size();
  for (const auto& gt : truth) metrics->ambiguous += gt.id == "-";
}

int Replay(const std::string& path) {
  std::ifstream input(path);
  Require(input.good(), "cannot open TSV");
  Require(Row(input, 3) ==
              std::vector<std::string>{"openlane-replay-v1", "OpenLaneV2_V0.1",
                                       "local-tl-prefix-diagnostic"},
          "unsupported version/profile");
  std::set<std::string> completed;
  std::string sequence;
  uint64_t timestamp = 0;
  int width = 0, height = 0;
  Metrics metrics;
  std::unique_ptr<PerceptionPipeline> pipeline;
  auto provider = std::make_shared<StaticDetectorProviderPort>();
  std::map<std::string, int> identities;
  std::map<int, Rect2f> previous;
  ComponentOptions options;
  options.enable_heuristic_stage = false;
  options.prompter.always_add_full_frame_fallback = false;
  while (input.peek() != std::char_traits<char>::eof()) {
    const auto row = Row(input, 7);
    Require(row[0] == "F" && (row[1] == "00492" || row[1] == "00556") &&
                row[2] == "ring_front_center",
            "unsupported frame identity");
    if (row[1] != sequence) {
      Require(completed.insert(row[1]).second, "noncontiguous sequence");
      sequence = row[1];
      timestamp = 0;
      width = height = 0;
      identities.clear();
      previous.clear();
      pipeline = CreateTrafficLightPipeline(options, provider);
      Require(pipeline->InitAll(), "pipeline initialization failed");
    }
    PipelineContext context;
    context.timestamp = Number<uint64_t>(row[3]);
    Require(context.timestamp > timestamp, "nonmonotonic timestamp");
    timestamp = context.timestamp;
    CameraFrameState camera;
    camera.camera_name = context.primary_camera_name = row[2];
    camera.timestamp_ns = timestamp;
    camera.image.cols = Number<int>(row[4]);
    camera.image.rows = Number<int>(row[5]);
    Require(camera.image.cols > 0 && camera.image.rows > 0,
            "invalid dimensions");
    if (width == 0) {
      width = camera.image.cols;
      height = camera.image.rows;
    }
    Require(width == camera.image.cols && height == camera.image.rows,
            "dimensions changed");
    context.camera_frames = {camera};
    std::vector<Truth> truth;
    std::vector<YoloLightCandidate> detections;
    std::set<std::string> ids;
    const size_t count = Number<size_t>(row[6]);
    Require(count <= static_cast<size_t>(width) * height, "absurd row count");
    for (size_t i = 0; i < count; ++i) {
      const auto label = Row(input, 7);
      Require(label[0] == "G" &&
                  (label[1] == "-" ||
                   (label[1].find("tl_") == 0 && ids.insert(label[1]).second)),
              "invalid/duplicate evaluation id");
      const std::map<std::string, LightColor> colors = {
          {"red", LightColor::RED},
          {"green", LightColor::GREEN},
          {"yellow", LightColor::YELLOW},
          {"unknown", LightColor::UNKNOWN}};
      Require(colors.count(label[6]), "invalid color");
      Truth gt{label[1],
               {Number<float>(label[2]), Number<float>(label[3]),
                Number<float>(label[4]), Number<float>(label[5])},
               colors.at(label[6])};
      Require(Geometry(gt.box, width, height), "invalid observation box");
      truth.push_back(gt);
      YoloLightCandidate candidate;
      candidate.bbox = gt.box;
      candidate.camera_name = camera.camera_name;
      candidate.objectness = candidate.semantic_confidence = 0.9f;
      candidate.color = gt.color;
      candidate.shape = LightShape::UNKNOWN;
      detections.push_back(candidate);
    }
    provider->SetDetections(detections);
    Require(pipeline->ProcessFrame(&context), "pipeline failed");
    Require(context.bound_lights.empty() && context.final_lights.empty(),
            "fabricated binding/control without topology/map runtime input");
    Evaluate(context, truth, previous, options.prompter.history_expand_ratio,
             &identities, &metrics);
    for (const auto& light : context.visual_lights)
      if (light.track_id > 0) previous[light.track_id] = light.bbox;
  }
  Require(metrics.frames > 0 && metrics.observations > 0, "empty diagnostic");
  std::cout << "{\"diagnostic\":\"annotation-injected-no-pixels\","
            << "\"profile\":\"local-tl-prefix-not-official-category\","
            << "\"roi_metrics\":\"all-history-proposals-excludes-full-frame\","
            << "\"detector_recall_or_cost_evidence\":false,"
            << "\"production_tracking_quality_validated\":false,"
            << "\"sequences\":" << completed.size()
            << ",\"frames\":" << metrics.frames
            << ",\"observations\":" << metrics.observations
            << ",\"ambiguous_identity_observations\":" << metrics.ambiguous
            << ",\"matches_iou_ge_0_5\":" << metrics.matches
            << ",\"identity_switches\":" << metrics.switches
            << ",\"history_covered\":" << metrics.history_covered
            << ",\"static_covered\":" << metrics.static_covered
            << ",\"history_rois\":" << metrics.history_rois
            << ",\"static_rois\":" << metrics.static_rois
            << ",\"history_area_px2_sum\":" << metrics.history_area
            << ",\"static_area_px2_sum\":" << metrics.static_area
            << ",\"image_area_px2_sum\":" << metrics.image_area
            << ",\"prediction_frames\":" << metrics.prediction_frames
            << ",\"exploration_frames\":" << metrics.exploration_frames
            << ",\"bound_lights\":0,\"final_lights\":0}\n";
  return 0;
}
}  // namespace
}  // namespace traffic_light
}  // namespace apollo

int main(int argc, char** argv) {
  try {
    if (argc != 2) throw std::runtime_error("usage: openlane_replay INPUT.tsv");
    return apollo::traffic_light::Replay(argv[1]);
  } catch (const std::exception& error) {
    std::cerr << "replay failed: " << error.what() << '\n';
    return 1;
  }
}
