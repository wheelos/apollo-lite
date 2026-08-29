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

//  Created Date: 2026-08-28
//  Author: daohu527

#include "modules/lane/inference/ufldv2.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace apollo {
namespace lane {

namespace {

constexpr int kRowGridCount = 200;
constexpr int kColumnGridCount = 100;
constexpr int kRowAnchorCount = 72;
constexpr int kColumnAnchorCount = 81;
constexpr int kLaneCandidateCount = 4;
constexpr float kMergeYDistancePixels = 2.5F;
constexpr float kMinimumLanePoints = 3.0F;

bool UsesRowHead(int lane) { return lane == 1 || lane == 2; }

bool UsesColumnHead(int lane) { return lane == 0 || lane == 3; }

struct DecodedPoint {
  ImagePoint point;
  float confidence = 0.0F;
};

struct Candidate {
  uint32_t id = 0;
  float confidence_sum = 0.0F;
  size_t confidence_count = 0;
  std::vector<DecodedPoint> points;
  float bottom_x = 0.0F;
  bool has_bottom_x = false;
};

bool CheckedTensorSize(const std::vector<int64_t>& shape, size_t* size) {
  if (size == nullptr || shape.empty()) {
    return false;
  }
  size_t product = 1;
  for (const int64_t dimension : shape) {
    if (dimension <= 0 || product > std::numeric_limits<size_t>::max() /
                                        static_cast<size_t>(dimension)) {
      return false;
    }
    product *= static_cast<size_t>(dimension);
  }
  *size = product;
  return true;
}

bool HasShape(const Ufldv2Tensor& tensor,
              const std::array<int64_t, 4>& expected_shape) {
  if (tensor.shape.size() != expected_shape.size() ||
      !std::equal(tensor.shape.begin(), tensor.shape.end(),
                  expected_shape.begin())) {
    return false;
  }
  size_t expected_size = 0;
  return CheckedTensorSize(tensor.shape, &expected_size) &&
         tensor.values.size() == expected_size;
}

bool SoftmaxExpectation(const std::vector<float>& values, size_t offset,
                        int class_count, float* expectation,
                        float* confidence) {
  if (expectation == nullptr || confidence == nullptr || class_count <= 0 ||
      offset > values.size() ||
      values.size() - offset < static_cast<size_t>(class_count)) {
    return false;
  }
  float maximum = -std::numeric_limits<float>::infinity();
  for (int index = 0; index < class_count; ++index) {
    const float value = values[offset + static_cast<size_t>(index)];
    if (!std::isfinite(value)) {
      return false;
    }
    maximum = std::max(maximum, value);
  }
  double sum = 0.0;
  double weighted_sum = 0.0;
  double peak = 0.0;
  for (int index = 0; index < class_count; ++index) {
    const double weight = std::exp(static_cast<double>(
        values[offset + static_cast<size_t>(index)] - maximum));
    sum += weight;
    weighted_sum += static_cast<double>(index) * weight;
    peak = std::max(peak, weight);
  }
  if (!std::isfinite(sum) || sum <= 0.0) {
    return false;
  }
  *expectation = static_cast<float>(weighted_sum / sum);
  *confidence = static_cast<float>(peak / sum);
  return true;
}

bool Exists(const Ufldv2Tensor& tensor, int anchor, int lane, int anchor_count,
            float* confidence) {
  const size_t class_zero =
      (static_cast<size_t>(anchor) * kLaneCandidateCount) +
      static_cast<size_t>(lane);
  const size_t class_one =
      (static_cast<size_t>(anchor_count) * kLaneCandidateCount) + class_zero;
  if (confidence == nullptr || class_one >= tensor.values.size()) {
    return false;
  }
  const float absent = tensor.values[class_zero];
  const float present = tensor.values[class_one];
  if (!std::isfinite(absent) || !std::isfinite(present)) {
    return false;
  }
  const float maximum = std::max(absent, present);
  const double absent_weight = std::exp(static_cast<double>(absent - maximum));
  const double present_weight =
      std::exp(static_cast<double>(present - maximum));
  *confidence =
      static_cast<float>(present_weight / (absent_weight + present_weight));
  return present_weight > absent_weight;
}

size_t RowOffset(int grid, int anchor, int lane) {
  return ((static_cast<size_t>(grid) * kRowAnchorCount +
           static_cast<size_t>(anchor)) *
              kLaneCandidateCount +
          static_cast<size_t>(lane));
}

size_t ColumnOffset(int grid, int anchor, int lane) {
  return ((static_cast<size_t>(grid) * kColumnAnchorCount +
           static_cast<size_t>(anchor)) *
              kLaneCandidateCount +
          static_cast<size_t>(lane));
}

float ModelToSourceX(float x, const ImageView& image) {
  return x * static_cast<float>(image.width - 1U) /
         static_cast<float>(kUfldv2ModelWidth - 1);
}

float NormalizedToSourceY(float normalized_y, const ImageView& image) {
  return normalized_y * static_cast<float>(image.height - 1U);
}

void MergePoints(std::vector<DecodedPoint>* points) {
  std::sort(points->begin(), points->end(),
            [](const DecodedPoint& left, const DecodedPoint& right) {
              return left.point.y < right.point.y;
            });
  std::vector<DecodedPoint> merged;
  merged.reserve(points->size());
  for (const DecodedPoint& point : *points) {
    if (!merged.empty() && std::fabs(point.point.y - merged.back().point.y) <=
                               kMergeYDistancePixels) {
      const float total_confidence =
          merged.back().confidence + point.confidence;
      if (total_confidence > 0.0F) {
        merged.back().point.x =
            (merged.back().point.x * merged.back().confidence +
             point.point.x * point.confidence) /
            total_confidence;
        merged.back().point.y =
            (merged.back().point.y * merged.back().confidence +
             point.point.y * point.confidence) /
            total_confidence;
        merged.back().confidence = total_confidence * 0.5F;
      }
    } else {
      merged.push_back(point);
    }
  }
  *points = std::move(merged);
}

bool ComputeBottomIntersection(const std::vector<DecodedPoint>& points,
                               uint32_t image_height, float* bottom_x) {
  if (bottom_x == nullptr || points.size() < 2U) {
    return false;
  }
  const float lower_limit = static_cast<float>(image_height - 1U) * 0.6F;
  double sum_y = 0.0;
  double sum_x = 0.0;
  double sum_yy = 0.0;
  double sum_yx = 0.0;
  size_t count = 0;
  for (const DecodedPoint& point : points) {
    if (point.point.y < lower_limit) {
      continue;
    }
    const double y = point.point.y;
    const double x = point.point.x;
    sum_y += y;
    sum_x += x;
    sum_yy += y * y;
    sum_yx += y * x;
    ++count;
  }
  if (count < 2U) {
    return false;
  }
  const double denominator =
      static_cast<double>(count) * sum_yy - sum_y * sum_y;
  if (std::fabs(denominator) <= std::numeric_limits<double>::epsilon()) {
    return false;
  }
  const double slope =
      (static_cast<double>(count) * sum_yx - sum_y * sum_x) / denominator;
  const double intercept = (sum_x - slope * sum_y) / count;
  const double intersection =
      slope * static_cast<double>(image_height - 1U) + intercept;
  if (!std::isfinite(intersection)) {
    return false;
  }
  *bottom_x = static_cast<float>(intersection);
  return true;
}

LanePosition PositionForRank(bool left, size_t rank) {
  if (left) {
    if (rank == 0U) return LanePosition::kEgoLeft;
    if (rank == 1U) return LanePosition::kAdjacentLeft;
    if (rank == 2U) return LanePosition::kThirdLeft;
  } else {
    if (rank == 0U) return LanePosition::kEgoRight;
    if (rank == 1U) return LanePosition::kAdjacentRight;
    if (rank == 2U) return LanePosition::kThirdRight;
  }
  return LanePosition::kUnknown;
}

}  // namespace

bool Ufldv2Decoder::ValidateOutputShapes(const Ufldv2TensorOutputs& outputs) {
  const auto all_finite = [](const Ufldv2Tensor& tensor) {
    return std::all_of(tensor.values.begin(), tensor.values.end(),
                       [](float value) { return std::isfinite(value); });
  };
  return HasShape(outputs.loc_row, {1, 200, 72, 4}) &&
         HasShape(outputs.loc_col, {1, 100, 81, 4}) &&
         HasShape(outputs.exist_row, {1, 2, 72, 4}) &&
         HasShape(outputs.exist_col, {1, 2, 81, 4}) &&
         all_finite(outputs.loc_row) && all_finite(outputs.loc_col) &&
         all_finite(outputs.exist_row) && all_finite(outputs.exist_col);
}

bool Ufldv2Decoder::Decode(const ImageView& image,
                           const Ufldv2TensorOutputs& outputs,
                           LaneDetectionResult* result) const {
  std::string error;
  if (result == nullptr || !image.Validate(&error) ||
      !ValidateOutputShapes(outputs)) {
    return false;
  }

  std::vector<Candidate> candidates(kLaneCandidateCount);
  for (int lane = 0; lane < kLaneCandidateCount; ++lane) {
    candidates[static_cast<size_t>(lane)].id = static_cast<uint32_t>(lane);
    if (UsesRowHead(lane)) {
      for (int anchor = 0; anchor < kRowAnchorCount; ++anchor) {
        float existence_confidence = 0.0F;
        if (!Exists(outputs.exist_row, anchor, lane, kRowAnchorCount,
                    &existence_confidence)) {
          continue;
        }
        std::vector<float> logits;
        logits.reserve(kRowGridCount);
        for (int grid = 0; grid < kRowGridCount; ++grid) {
          logits.push_back(
              outputs.loc_row.values[RowOffset(grid, anchor, lane)]);
        }
        float location = 0.0F;
        float location_confidence = 0.0F;
        if (!SoftmaxExpectation(logits, 0U, kRowGridCount, &location,
                                &location_confidence)) {
          return false;
        }
        const float model_x = location *
                              static_cast<float>(kUfldv2ModelWidth - 1) /
                              static_cast<float>(kRowGridCount - 1);
        const float normalized_y =
            0.42F + 0.58F * static_cast<float>(anchor) /
                        static_cast<float>(kRowAnchorCount - 1);
        Candidate& candidate = candidates[static_cast<size_t>(lane)];
        candidate.points.push_back(
            {{ModelToSourceX(model_x, image),
              NormalizedToSourceY(normalized_y, image)},
             existence_confidence * location_confidence});
        candidate.confidence_sum += existence_confidence;
        ++candidate.confidence_count;
      }
    }

    if (UsesColumnHead(lane)) {
      for (int anchor = 0; anchor < kColumnAnchorCount; ++anchor) {
        float existence_confidence = 0.0F;
        if (!Exists(outputs.exist_col, anchor, lane, kColumnAnchorCount,
                    &existence_confidence)) {
          continue;
        }
        std::vector<float> logits;
        logits.reserve(kColumnGridCount);
        for (int grid = 0; grid < kColumnGridCount; ++grid) {
          logits.push_back(
              outputs.loc_col.values[ColumnOffset(grid, anchor, lane)]);
        }
        float location = 0.0F;
        float location_confidence = 0.0F;
        if (!SoftmaxExpectation(logits, 0U, kColumnGridCount, &location,
                                &location_confidence)) {
          return false;
        }
        const float model_x = static_cast<float>(anchor) *
                              static_cast<float>(kUfldv2ModelWidth - 1) /
                              static_cast<float>(kColumnAnchorCount - 1);
        const float normalized_y =
            location / static_cast<float>(kColumnGridCount - 1);
        Candidate& candidate = candidates[static_cast<size_t>(lane)];
        candidate.points.push_back(
            {{ModelToSourceX(model_x, image),
              NormalizedToSourceY(normalized_y, image)},
             existence_confidence * location_confidence});
        candidate.confidence_sum += existence_confidence;
        ++candidate.confidence_count;
      }
    }
    MergePoints(&candidates[static_cast<size_t>(lane)].points);
    Candidate& candidate = candidates[static_cast<size_t>(lane)];
    candidate.has_bottom_x = ComputeBottomIntersection(
        candidate.points, image.height, &candidate.bottom_x);
  }

  std::vector<Candidate> kept;
  kept.reserve(candidates.size());
  const float duplicate_distance =
      std::max(8.0F, static_cast<float>(image.width) * 0.0125F);
  for (Candidate& candidate : candidates) {
    if (candidate.points.size() < static_cast<size_t>(kMinimumLanePoints)) {
      continue;
    }
    bool duplicate = false;
    for (Candidate& existing : kept) {
      if (candidate.has_bottom_x && existing.has_bottom_x &&
          std::fabs(candidate.bottom_x - existing.bottom_x) <
              duplicate_distance) {
        duplicate = true;
        if (candidate.confidence_sum > existing.confidence_sum) {
          existing = std::move(candidate);
        }
        break;
      }
    }
    if (!duplicate) {
      kept.push_back(std::move(candidate));
    }
  }

  std::vector<size_t> left_indices;
  std::vector<size_t> right_indices;
  const float image_center = static_cast<float>(image.width - 1U) * 0.5F;
  for (size_t index = 0; index < kept.size(); ++index) {
    if (kept[index].has_bottom_x) {
      (kept[index].bottom_x < image_center ? left_indices : right_indices)
          .push_back(index);
    }
  }
  std::sort(left_indices.begin(), left_indices.end(),
            [&kept](size_t left, size_t right) {
              return kept[left].bottom_x > kept[right].bottom_x;
            });
  std::sort(right_indices.begin(), right_indices.end(),
            [&kept](size_t left, size_t right) {
              return kept[left].bottom_x < kept[right].bottom_x;
            });

  result->timestamp_sec = image.timestamp_sec;
  result->camera_timestamp_ns = image.camera_timestamp_ns;
  result->sequence_num = image.sequence_num;
  result->frame_id = image.frame_id;
  result->lanes.clear();
  result->lanes.reserve(kept.size());
  for (const Candidate& candidate : kept) {
    LaneLineResult lane;
    lane.candidate_id = candidate.id;
    lane.confidence = candidate.confidence_count == 0U
                          ? 0.0F
                          : candidate.confidence_sum /
                                static_cast<float>(candidate.confidence_count);
    lane.image_points.reserve(candidate.points.size());
    for (const DecodedPoint& point : candidate.points) {
      lane.image_points.push_back(point.point);
    }
    result->lanes.push_back(std::move(lane));
  }
  for (size_t rank = 0; rank < left_indices.size(); ++rank) {
    result->lanes[left_indices[rank]].position = PositionForRank(true, rank);
  }
  for (size_t rank = 0; rank < right_indices.size(); ++rank) {
    result->lanes[right_indices[rank]].position = PositionForRank(false, rank);
  }
  return true;
}

bool Ufldv2Detector::Preprocess(const ImageView& image,
                                std::vector<float>* input) {
  std::string error;
  if (input == nullptr || !image.Validate(&error)) {
    return false;
  }
  const uint32_t crop_y = image.height * 2U / 5U;
  const uint32_t crop_height = image.height - crop_y;
  if (crop_height < 2U || image.width < 2U) {
    return false;
  }
  input->assign(kUfldv2InputElementCount, 0.0F);
  constexpr std::array<float, 3> kMean = {0.485F, 0.456F, 0.406F};
  constexpr std::array<float, 3> kStd = {0.229F, 0.224F, 0.225F};
  cv::Mat source(
      static_cast<int>(image.height), static_cast<int>(image.width), CV_8UC3,
      const_cast<uint8_t*>(image.bytes));
  cv::Mat resized;
  cv::resize(
      source(cv::Rect(0, static_cast<int>(crop_y),
                      static_cast<int>(image.width),
                      static_cast<int>(crop_height))),
      resized, cv::Size(kUfldv2ModelWidth, kUfldv2ModelHeight), 0.0, 0.0,
      cv::INTER_LINEAR);
  std::vector<cv::Mat> source_channels;
  cv::split(resized, source_channels);
  const size_t plane_size =
      static_cast<size_t>(kUfldv2ModelHeight) * kUfldv2ModelWidth;
  for (size_t channel = 0; channel < 3U; ++channel) {
    const size_t source_channel =
        image.encoding == ImageEncoding::kRgb8 ? channel : 2U - channel;
    cv::Mat destination(kUfldv2ModelHeight, kUfldv2ModelWidth, CV_32FC1,
                        input->data() + channel * plane_size);
    source_channels[source_channel].convertTo(
        destination, CV_32FC1, 1.0 / (255.0 * kStd[channel]),
        -kMean[channel] / kStd[channel]);
  }
  return true;
}

bool Ufldv2Detector::Detect(const ImageView& image,
                            LaneDetectionResult* result) const {
  if (executor_ == nullptr || result == nullptr) {
    return false;
  }
  std::vector<float> input;
  Ufldv2TensorOutputs outputs;
  return Preprocess(image, &input) && executor_->Run(input, &outputs) &&
         decoder_.Decode(image, outputs, result);
}

}  // namespace lane
}  // namespace apollo
