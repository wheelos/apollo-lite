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
