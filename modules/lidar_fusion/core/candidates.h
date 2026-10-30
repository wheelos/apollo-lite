// Copyright 2026 WheelOS. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/lidar_fusion/proto/lidar_fusion.pb.h"

namespace apollo {
namespace lidar_fusion {

using PointCell = std::pair<int, int>;

struct CellHash {
  std::size_t operator()(const PointCell& key) const {
    const auto x = std::hash<int>{}(key.first);
    return x ^
           (std::hash<int>{}(key.second) + 0x9e3779b9 + (x << 6) + (x >> 2));
  }
};

using PointIndex = std::unordered_map<PointCell, std::vector<int>, CellHash>;

struct CandidateWork {
  uint64_t pairs = 0;
  uint64_t point_transforms = 0;
};

struct CandidateResult {
  google::protobuf::RepeatedPtrField<ObjectCandidate> objects;
  google::protobuf::RepeatedPtrField<UnknownCandidate> unknowns;
};

bool ValidateCandidates(const FusionConfig& config);
std::vector<int> CandidateComponents(
    const FusionConfig& config,
    const google::protobuf::RepeatedPtrField<PointEvidence>& evidence,
    const google::protobuf::RepeatedPtrField<PointDecision>& decisions);
void PartitionUnknowns(
    const FusionConfig& config,
    const google::protobuf::RepeatedPtrField<PointEvidence>& evidence,
    const google::protobuf::RepeatedPtrField<UnknownCandidate>& unknowns,
    google::protobuf::RepeatedPtrField<ShadowPartition>* partitions);
void RefineCandidates(
    const FusionConfig& config,
    const google::protobuf::RepeatedPtrField<PointEvidence>& evidence,
    CandidateResult* candidates, CandidateWork* work = nullptr,
    const PointIndex* index = nullptr);

}  // namespace lidar_fusion
}  // namespace apollo
