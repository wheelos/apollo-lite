// Copyright 2026 WheelOS. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "modules/lidar_fusion/publication/pod.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "google/protobuf/io/coded_stream.h"
#include "google/protobuf/io/zero_copy_stream_impl.h"
#include "google/protobuf/io/zero_copy_stream_impl_lite.h"

#include "cyber/common/log.h"
#include "cyber/node/node.h"

namespace apollo {
namespace lidar_fusion {
namespace {

using cyber::transport::PodChunkHeader;
using cyber::transport::PodChunkView;
using cyber::transport::PodMessage;
using Clock = std::chrono::steady_clock;

bool Fail(const char* reason, std::string* error) {
  if (error != nullptr) {
    *error = reason;
  }
  return false;
}

bool ValidLimit(std::size_t bytes) {
  return bytes > 0 &&
         bytes <= static_cast<std::size_t>(std::numeric_limits<int>::max());
}

std::size_t FragmentCount(std::size_t bytes) {
  return (bytes + kFusionPodFragmentBytes - 1) / kFusionPodFragmentBytes;
}

std::size_t FragmentSize(std::size_t bytes, std::size_t index) {
  return std::min(kFusionPodFragmentBytes,
                  bytes - index * kFusionPodFragmentBytes);
}

double ElapsedMs(Clock::time_point begin) {
  return std::chrono::duration<double, std::milli>(Clock::now() - begin)
      .count();
}

class FragmentOutputStream final
    : public google::protobuf::io::ZeroCopyOutputStream {
 public:
  explicit FragmentOutputStream(const std::vector<FusionPodBuffer>& buffers)
      : buffers_(buffers) {}

  bool Next(void** data, int* size) override {
    if (backup_ > 0) {
      *data = buffers_[index_ - 1].data + buffers_[index_ - 1].size - backup_;
      *size = backup_;
      count_ += backup_;
      backup_ = 0;
      return true;
    }
    if (index_ == buffers_.size()) {
      return false;
    }
    const auto& buffer = buffers_[index_++];
    *data = buffer.data + sizeof(PodChunkHeader);
    *size = static_cast<int>(buffer.size - sizeof(PodChunkHeader));
    count_ += *size;
    return true;
  }

  void BackUp(int count) override {
    backup_ = count;
    count_ -= count;
  }

  int64_t ByteCount() const override { return count_; }

 private:
  const std::vector<FusionPodBuffer>& buffers_;
  std::size_t index_ = 0;
  int backup_ = 0;
  int64_t count_ = 0;
};

bool Encode(const FusionOutput& frame, std::size_t bytes, uint64_t epoch,
            uint64_t message_id, const std::vector<FusionPodBuffer>& buffers,
            std::string* error) {
  if (!ValidLimit(bytes) || epoch == 0 || message_id == 0 ||
      buffers.size() != FragmentCount(bytes)) {
    return Fail("invalid CORE_POD frame size, identity or fragment count",
                error);
  }
  for (std::size_t i = 0; i < buffers.size(); ++i) {
    if (buffers[i].data == nullptr ||
        buffers[i].size != sizeof(PodChunkHeader) + FragmentSize(bytes, i)) {
      return Fail("CORE_POD buffer does not match canonical fragment size",
                  error);
    }
  }
  for (std::size_t i = 0; i < buffers.size(); ++i) {
    PodChunkHeader header;
    // The aligned SDK header has padding; never publish uninitialized bytes.
    std::memset(static_cast<void*>(&header), 0, sizeof(header));
    header.magic = PodChunkHeader::kMagic;
    header.version = PodChunkHeader::kVersion;
    header.header_size = sizeof(header);
    header.payload_kind =
        static_cast<uint32_t>(cyber::transport::PodPayloadKind::UNKNOWN);
    header.timestamp_ns = frame.header().lidar_timestamp();
    header.frame_id = message_id;
    header.payload_size = static_cast<uint32_t>(FragmentSize(bytes, i));
    header.schema_hash = kFusionPodSchema;
    header.reserved[0] = epoch;
    header.reserved[1] = bytes;
    header.reserved[2] = i * kFusionPodFragmentBytes;
    header.reserved[3] = (static_cast<uint64_t>(buffers.size()) << 32) | i;
    std::memcpy(buffers[i].data, &header, sizeof(header));
  }
  FragmentOutputStream stream(buffers);
  bool failed = false;
  {
    google::protobuf::io::CodedOutputStream coded(&stream);
    frame.SerializeWithCachedSizes(&coded);
    failed = coded.HadError();
  }
  if (failed || stream.ByteCount() != static_cast<int64_t>(bytes)) {
    return Fail("CORE_POD direct protobuf encoding failed", error);
  }
  return true;
}

}  // namespace

bool EncodeFusionPod(const FusionOutput& frame, uint64_t epoch,
                     uint64_t message_id,
                     const std::vector<FusionPodBuffer>& buffers,
                     std::string* error) {
  return Encode(frame, frame.ByteSizeLong(), epoch, message_id, buffers, error);
}

void FusionPodAssembler::Reset() {
  fragments_.clear();
  total_bytes_ = 0;
  timestamp_ns_ = 0;
}

PodAssemblyResult FusionPodAssembler::Push(
    const std::shared_ptr<PodMessage>& fragment, FusionOutput* output,
    std::string* error) {
  const auto reject = [&](const char* reason) {
    Reset();
    Fail(reason, error);
    return PodAssemblyResult::REJECTED;
  };
  PodChunkView view;
  if (output == nullptr || fragment == nullptr ||
      !ValidLimit(max_frame_bytes_) ||
      !cyber::transport::ParsePodChunk(fragment->data(), fragment->size(),
                                       &view) ||
      fragment->size() != view.total_size ||
      view.header.schema_hash != kFusionPodSchema ||
      view.header.payload_kind !=
          static_cast<uint32_t>(cyber::transport::PodPayloadKind::UNKNOWN) ||
      view.header.width != 0 || view.header.height != 0 ||
      view.header.stride_bytes != 0 || view.header.pixel_format != 0) {
    return reject("invalid CORE_POD envelope, schema or reader limit");
  }
  const auto& header = view.header;
  const uint64_t bytes = header.reserved[1];
  const uint64_t index = header.reserved[3] & 0xffffffffu;
  const uint64_t count = header.reserved[3] >> 32;
  if (!ValidLimit(bytes) || bytes > max_frame_bytes_ ||
      header.reserved[0] == 0 || header.frame_id == 0 ||
      count != FragmentCount(bytes) || index >= count ||
      header.reserved[2] != index * kFusionPodFragmentBytes ||
      view.payload_size != FragmentSize(bytes, index)) {
    return reject("invalid CORE_POD fragment bounds or identity");
  }
  if (index == 0) {
    if (header.reserved[0] == epoch_ && header.frame_id <= message_id_) {
      return reject("duplicate or stale CORE_POD frame");
    }
    if (!fragments_.empty()) {
      AWARN << "Discarding incomplete CORE_POD fusion frame";
    }
    Reset();
    epoch_ = header.reserved[0];
    message_id_ = header.frame_id;
    timestamp_ns_ = header.timestamp_ns;
    total_bytes_ = bytes;
    fragments_.reserve(count);
  }
  if (header.reserved[0] != epoch_ || header.frame_id != message_id_ ||
      header.timestamp_ns != timestamp_ns_ || bytes != total_bytes_ ||
      index != fragments_.size()) {
    return reject("missing, reordered or mismatched CORE_POD fragment");
  }
  fragments_.push_back(fragment);
  if (fragments_.size() != count) {
    return PodAssemblyResult::INCOMPLETE;
  }

  using google::protobuf::io::ArrayInputStream;
  std::vector<std::unique_ptr<ArrayInputStream>> streams;
  std::vector<google::protobuf::io::ZeroCopyInputStream*> inputs;
  streams.reserve(count);
  inputs.reserve(count);
  for (const auto& item : fragments_) {
    PodChunkView part;
    if (!cyber::transport::ParsePodChunk(item->data(), item->size(), &part)) {
      return reject("CORE_POD chunk changed while held");
    }
    streams.push_back(std::make_unique<ArrayInputStream>(
        part.payload, static_cast<int>(part.payload_size)));
    inputs.push_back(streams.back().get());
  }
  google::protobuf::io::ConcatenatingInputStream input(inputs.data(),
                                                       inputs.size());
  FusionOutput decoded;
  if (!decoded.ParseFromZeroCopyStream(&input) ||
      input.ByteCount() != static_cast<int64_t>(total_bytes_) ||
      decoded.header().lidar_timestamp() != timestamp_ns_) {
    return reject("invalid CORE_POD protobuf payload or sweep timestamp");
  }
  *output = std::move(decoded);
  Reset();
  return PodAssemblyResult::COMPLETE;
}

bool ConfigureFusionPodReader(const std::string& channel,
                              std::size_t max_frame_bytes,
                              cyber::ReaderConfig* config, std::string* error) {
  if (channel.empty() || !ValidLimit(max_frame_bytes) || config == nullptr) {
    return Fail("invalid CORE_POD reader channel or byte limit", error);
  }
  cyber::ReaderConfig prepared;
  prepared.channel_name = channel;
  prepared.pending_queue_size = FragmentCount(max_frame_bytes);
  prepared.qos_profile.set_depth(prepared.pending_queue_size);
  *config = prepared;
  return true;
}

FusionPodPublisher::FusionPodPublisher(
    std::shared_ptr<cyber::Writer<PodMessage>> writer,
    std::size_t max_frame_bytes)
    : writer_(std::move(writer)), max_frame_bytes_(max_frame_bytes) {
  std::random_device random;
  do {
    epoch_ = (static_cast<uint64_t>(random()) << 32) ^ random();
  } while (epoch_ == 0);
}

bool FusionPodPublisher::HasReader() const {
  return writer_ != nullptr && writer_->HasReader();
}

PodPublishResult FusionPodPublisher::Publish(const FusionOutput& frame,
                                             std::string* error,
                                             PodPublicationWork* work) {
  if (work != nullptr) {
    *work = {};
  }
  const auto reject = [&](const char* reason) {
    Fail(reason, error);
    return PodPublishResult::REJECTED;
  };
  if (writer_ == nullptr || !ValidLimit(max_frame_bytes_) ||
      message_id_ == std::numeric_limits<uint64_t>::max()) {
    return reject("invalid CORE_POD publisher, byte limit or sequence");
  }
  if (!HasReader()) {
    return PodPublishResult::NO_READERS;
  }
  auto begin = work != nullptr ? Clock::now() : Clock::time_point{};
  const auto bytes = frame.ByteSizeLong();
  if (bytes == 0 || bytes > max_frame_bytes_) {
    return reject("CORE_POD frame exceeds the configured byte limit");
  }
  if (work != nullptr) {
    work->size_ms = ElapsedMs(begin);
    work->bytes = bytes;
    work->fragments = FragmentCount(bytes);
    begin = Clock::now();
  }
  std::vector<cyber::Writer<PodMessage>::LoanedMessage> loans(
      FragmentCount(bytes));
  std::vector<FusionPodBuffer> buffers;
  buffers.reserve(loans.size());
  // Acquire the whole frame first. A loan failure must publish no fragment.
  for (std::size_t i = 0; i < loans.size(); ++i) {
    const auto size = sizeof(PodChunkHeader) + FragmentSize(bytes, i);
    if (!writer_->Loan(size, &loans[i]) || loans[i].capacity() < size ||
        !loans[i].set_size(size)) {
      return reject(
          "CORE_POD loan unavailable: require exclusive local ICEORYX/SHM "
          "readers and sufficient pool capacity; no copied fallback");
    }
    buffers.push_back({loans[i].data(), size});
  }
  if (work != nullptr) {
    work->loan_ms = ElapsedMs(begin);
    begin = Clock::now();
  }
  if (!Encode(frame, bytes, epoch_, ++message_id_, buffers, error)) {
    return PodPublishResult::REJECTED;
  }
  if (work != nullptr) {
    work->encode_ms = ElapsedMs(begin);
    begin = Clock::now();
  }
  for (auto& loan : loans) {
    if (!writer_->Publish(std::move(loan))) {
      return reject(
          "CORE_POD publish rejected; partial frames must be discarded");
    }
  }
  if (work != nullptr) {
    work->publish_ms = ElapsedMs(begin);
  }
  return PodPublishResult::PUBLISHED;
}

}  // namespace lidar_fusion
}  // namespace apollo
