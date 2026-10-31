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
#include <memory>
#include <string>
#include <vector>

#include "modules/lidar_fusion/proto/lidar_fusion.pb.h"

#include "cyber/node/writer.h"
#include "cyber/transport/iceoryx_chunk.h"

namespace apollo {
namespace cyber {
struct ReaderConfig;
}  // namespace cyber

namespace lidar_fusion {

struct FusionPodBuffer {
  uint8_t* data;
  std::size_t size;
};

constexpr uint32_t kFusionPodSchema = 0x46555331;  // FUS1, not a checksum.
constexpr std::size_t kFusionPodFragmentBytes{
    cyber::transport::kIceoryxChunkPayloadCapacity -
        sizeof(cyber::transport::PodChunkHeader),
};

// Each buffer is exactly header + its canonical fragment size. No payload
// allocation or intermediate serialized string is needed.
bool EncodeFusionPod(const FusionOutput& frame, uint64_t epoch,
                     uint64_t message_id,
                     const std::vector<FusionPodBuffer>& buffers,
                     std::string* error);

enum class PodAssemblyResult { INCOMPLETE, COMPLETE, REJECTED };

// One ordered publisher stream per instance. Held shared_ptrs keep borrowed
// chunks alive; parsing reads their payloads without a reassembly buffer.
class FusionPodAssembler {
 public:
  explicit FusionPodAssembler(std::size_t max_frame_bytes)
      : max_frame_bytes_(max_frame_bytes) {}
  PodAssemblyResult Push(
      const std::shared_ptr<cyber::transport::PodMessage>& fragment,
      FusionOutput* output, std::string* error);
  void Reset();

 private:
  std::size_t max_frame_bytes_;
  uint64_t epoch_ = 0;
  uint64_t message_id_ = 0;
  uint64_t timestamp_ns_ = 0;
  std::size_t total_bytes_ = 0;
  std::vector<std::shared_ptr<cyber::transport::PodMessage>> fragments_;
};

bool ConfigureFusionPodReader(const std::string& channel,
                              std::size_t max_frame_bytes,
                              cyber::ReaderConfig* config, std::string* error);

enum class PodPublishResult { PUBLISHED, NO_READERS, REJECTED };

struct PodPublicationWork {
  double size_ms = 0;
  double loan_ms = 0;
  double encode_ms = 0;
  double publish_ms = 0;
  std::size_t bytes = 0;
  std::size_t fragments = 0;
};

class FusionPodPublisher {
 public:
  FusionPodPublisher(
      std::shared_ptr<cyber::Writer<cyber::transport::PodMessage>> writer,
      std::size_t max_frame_bytes);
  bool HasReader() const;
  // Serialized calls, like the owning component's process_mutex_. No copied
  // transport fallback; a failed/partial submission is not a delivered frame.
  PodPublishResult Publish(const FusionOutput& frame, std::string* error,
                           PodPublicationWork* work = nullptr);

 private:
  std::shared_ptr<cyber::Writer<cyber::transport::PodMessage>> writer_;
  std::size_t max_frame_bytes_;
  uint64_t epoch_;
  uint64_t message_id_ = 0;
};

}  // namespace lidar_fusion
}  // namespace apollo
