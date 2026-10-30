// Copyright 2026 WheelOS. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "modules/lidar_fusion/publication/pod.h"

#include <algorithm>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "google/protobuf/unknown_field_set.h"

namespace apollo {
namespace lidar_fusion {
namespace {

using cyber::transport::PodChunkHeader;
using cyber::transport::PodMessage;
constexpr std::size_t kLimit = 64 * 1024 * 1024;
using Storage = std::vector<uint8_t>;

FusionOutput Frame(OutputDetail detail = FULL_EVIDENCE) {
  FusionOutput frame;
  frame.mutable_header()->set_frame_id("base_link");
  frame.mutable_header()->set_lidar_timestamp(123456789);
  frame.mutable_health()->set_mode(GEO_ONLY);
  frame.set_odom_frame("fixture_odom");
  frame.add_risk_event(INPUT_QUEUE_OVERFLOW);
  if (detail == FUNCTIONAL_SCENE) {
    frame.mutable_publication()->set_detail(detail);
    frame.mutable_publication()->set_source_point_count(123);
  }
  return frame;
}

FusionOutput LargeFrame() {
  auto frame = Frame();
  frame.GetReflection()->MutableUnknownFields(&frame)->AddLengthDelimited(
      900, std::string(kFusionPodFragmentBytes + 1024, 'x'));
  return frame;
}

struct Encoded {
  std::vector<std::shared_ptr<Storage>> storage;
  std::vector<std::shared_ptr<PodMessage>> parts;
};

Encoded Pack(const FusionOutput& frame, uint64_t epoch = 1,
             uint64_t message_id = 1) {
  Encoded packed;
  std::vector<FusionPodBuffer> buffers;
  const auto bytes = frame.ByteSizeLong();
  for (std::size_t offset = 0; offset < bytes;
       offset += kFusionPodFragmentBytes) {
    const auto size = sizeof(PodChunkHeader) +
                      std::min(kFusionPodFragmentBytes, bytes - offset);
    packed.storage.push_back(std::make_shared<Storage>(size));
    buffers.push_back({packed.storage.back()->data(), size});
  }
  std::string error;
  EXPECT_TRUE(EncodeFusionPod(frame, epoch, message_id, buffers, &error))
      << error;
  for (const auto& storage : packed.storage) {
    auto message = std::make_shared<PodMessage>();
    EXPECT_TRUE(
        message->BorrowFromArray(storage->data(), storage->size(), storage));
    EXPECT_TRUE(message->is_borrowed());
    EXPECT_EQ(message->data(), storage->data());
    packed.parts.push_back(std::move(message));
  }
  return packed;
}

void ChangeHeader(Encoded* packed, std::size_t index,
                  const std::function<void(PodChunkHeader*)>& change) {
  PodChunkHeader header;
  std::memcpy(&header, packed->storage[index]->data(), sizeof(header));
  change(&header);
  std::memcpy(packed->storage[index]->data(), &header, sizeof(header));
}

TEST(FusionPodTest, PreservesLegacyFullAndFunctionalWireBytes) {
  for (auto detail : {FULL_EVIDENCE, FUNCTIONAL_SCENE}) {
    const auto original = Frame(detail);
    const auto expected = original.SerializeAsString();
    auto packed = Pack(original);
    ASSERT_EQ(packed.parts.size(), 1);
    FusionPodAssembler assembler(kLimit);
    FusionOutput decoded;
    std::string error;
    EXPECT_EQ(assembler.Push(packed.parts[0], &decoded, &error),
              PodAssemblyResult::COMPLETE)
        << error;
    EXPECT_EQ(decoded.SerializeAsString(), expected);
    EXPECT_EQ(original.SerializeAsString(), expected);
    EXPECT_EQ(assembler.Push(packed.parts[0], &decoded, &error),
              PodAssemblyResult::REJECTED);
    EXPECT_EQ(decoded.SerializeAsString(), expected);
  }
}

TEST(FusionPodTest, SpansRealChunkLimitAndRetainsBorrowedOwnersUntilParsing) {
  const auto original = LargeFrame();
  auto packed = Pack(original);
  ASSERT_EQ(packed.parts.size(), 2);
  std::weak_ptr<Storage> first = packed.storage[0];
  FusionPodAssembler assembler(kLimit);
  FusionOutput decoded = Frame(FUNCTIONAL_SCENE);
  const auto before = decoded.SerializeAsString();
  std::string error;
  ASSERT_EQ(assembler.Push(packed.parts[0], &decoded, &error),
            PodAssemblyResult::INCOMPLETE)
      << error;
  EXPECT_EQ(decoded.SerializeAsString(), before);
  packed.parts[0].reset();
  packed.storage[0].reset();
  EXPECT_FALSE(first.expired());
  ASSERT_EQ(assembler.Push(packed.parts[1], &decoded, &error),
            PodAssemblyResult::COMPLETE)
      << error;
  EXPECT_EQ(decoded.SerializeAsString(), original.SerializeAsString());
  EXPECT_TRUE(first.expired());
}

TEST(FusionPodTest, RejectsBadEnvelopeAndProtobufWithoutChangingOutput) {
  const std::vector<std::function<void(PodChunkHeader*)>> mutations = {
      [](auto* h) { h->schema_hash ^= 1; },
      [](auto* h) { h->payload_kind = 1; },
      [](auto* h) { h->width = 1; },
      [](auto* h) { h->reserved[0] = 0; },
      [](auto* h) { h->reserved[1] += 1; },
      [](auto* h) { h->reserved[2] = 1; },
      [](auto* h) { h->reserved[3] += uint64_t{1} << 32; },
      [](auto* h) { h->frame_id = 0; },
      [](auto* h) { h->timestamp_ns += 1; },
  };
  for (std::size_t i = 0; i < mutations.size() + 2; ++i) {
    SCOPED_TRACE(i);
    auto packed = Pack(Frame());
    if (i < mutations.size()) {
      ChangeHeader(&packed, 0, mutations[i]);
    } else if (i == mutations.size()) {
      packed.storage[0]->push_back(0);
      ASSERT_TRUE(packed.parts[0]->BorrowFromArray(packed.storage[0]->data(),
                                                   packed.storage[0]->size(),
                                                   packed.storage[0]));
    } else {
      (*packed.storage[0])[sizeof(PodChunkHeader)] = 0;
    }
    FusionPodAssembler assembler(kLimit);
    FusionOutput decoded = Frame(FUNCTIONAL_SCENE);
    const auto before = decoded.SerializeAsString();
    std::string error;
    EXPECT_EQ(assembler.Push(packed.parts[0], &decoded, &error),
              PodAssemblyResult::REJECTED);
    EXPECT_FALSE(error.empty());
    EXPECT_EQ(decoded.SerializeAsString(), before);
  }
}

TEST(FusionPodTest, RejectsMissingAndMixedFragmentsAndRecoversOnNewFrame) {
  const auto original = LargeFrame();
  auto first = Pack(original);
  auto second = Pack(original, 1, 2);
  FusionPodAssembler assembler(kLimit);
  FusionOutput decoded = Frame(FUNCTIONAL_SCENE);
  const auto before = decoded.SerializeAsString();
  std::string error;
  EXPECT_EQ(assembler.Push(first.parts[1], &decoded, &error),
            PodAssemblyResult::REJECTED);
  ASSERT_EQ(assembler.Push(first.parts[0], &decoded, &error),
            PodAssemblyResult::INCOMPLETE);
  EXPECT_EQ(assembler.Push(second.parts[1], &decoded, &error),
            PodAssemblyResult::REJECTED);
  EXPECT_EQ(decoded.SerializeAsString(), before);
  const auto expected = Frame();
  auto recovery = Pack(expected, 1, 3);
  ASSERT_EQ(assembler.Push(recovery.parts[0], &decoded, &error),
            PodAssemblyResult::COMPLETE)
      << error;
  EXPECT_EQ(decoded.SerializeAsString(), expected.SerializeAsString());
}

TEST(FusionPodTest, ReplacesIncompleteFrameWithoutEmittingItsPartialEvidence) {
  auto large = Pack(LargeFrame());
  auto small = Pack(Frame(FUNCTIONAL_SCENE), 1, 2);
  FusionPodAssembler assembler(kLimit);
  FusionOutput decoded;
  std::string error;
  ASSERT_EQ(assembler.Push(large.parts[0], &decoded, &error),
            PodAssemblyResult::INCOMPLETE);
  EXPECT_EQ(assembler.Push(small.parts[0], &decoded, &error),
            PodAssemblyResult::COMPLETE);
  EXPECT_EQ(decoded.SerializeAsString(),
            Frame(FUNCTIONAL_SCENE).SerializeAsString());
  auto restarted = Pack(Frame(), 2, 1);
  EXPECT_EQ(assembler.Push(restarted.parts[0], &decoded, &error),
            PodAssemblyResult::COMPLETE);
}

TEST(FusionPodTest, EnforcesByteBudgetAndCanonicalWritableBuffers) {
  const auto frame = Frame();
  auto packed = Pack(frame);
  FusionPodAssembler assembler(frame.ByteSizeLong() - 1);
  FusionOutput decoded = Frame(FUNCTIONAL_SCENE);
  const auto before = decoded.SerializeAsString();
  std::string error;
  EXPECT_EQ(assembler.Push(packed.parts[0], &decoded, &error),
            PodAssemblyResult::REJECTED);
  EXPECT_EQ(decoded.SerializeAsString(), before);
  const auto unchanged = *packed.storage[0];
  for (const auto size :
       {packed.storage[0]->size() - 1, packed.storage[0]->size() + 1}) {
    EXPECT_FALSE(EncodeFusionPod(frame, 1, 2,
                                 {{packed.storage[0]->data(), size}}, &error));
    EXPECT_EQ(*packed.storage[0], unchanged);
  }
  EXPECT_FALSE(EncodeFusionPod(frame, 0, 1, {}, &error));
}

class RejectingWriter final : public cyber::Writer<PodMessage> {
 public:
  RejectingWriter() : Writer(cyber::proto::RoleAttributes{}) {}
  bool HasReader() override { return has_reader; }
  bool Loan(std::size_t, LoanedMessage*) override {
    ++loan_calls;
    return false;
  }
  bool Publish(LoanedMessage&&) override {
    ++publish_calls;
    return false;
  }
  bool Write(const PodMessage&) override {
    ++copy_calls;
    return false;
  }
  bool Write(const std::shared_ptr<PodMessage>&) override {
    ++copy_calls;
    return false;
  }
  bool has_reader = false;
  int loan_calls = 0;
  int publish_calls = 0;
  int copy_calls = 0;
};

TEST(FusionPodTest, SkipsAbsentReadersAndNeverFallsBackAfterLoanRejection) {
  auto writer = std::make_shared<RejectingWriter>();
  FusionPodPublisher publisher(writer, kLimit);
  PodPublicationWork work;
  std::string error;
  EXPECT_EQ(publisher.Publish(Frame(), &error, &work),
            PodPublishResult::NO_READERS);
  EXPECT_EQ(writer->loan_calls, 0);
  writer->has_reader = true;
  EXPECT_EQ(publisher.Publish(Frame(), &error, &work),
            PodPublishResult::REJECTED);
  EXPECT_EQ(writer->loan_calls, 1);
  EXPECT_EQ(writer->publish_calls, 0);
  EXPECT_EQ(writer->copy_calls, 0);
  FusionPodPublisher limited(writer, 1);
  EXPECT_EQ(limited.Publish(Frame(), &error), PodPublishResult::REJECTED);
  EXPECT_EQ(writer->loan_calls, 1);
}

}  // namespace
}  // namespace lidar_fusion
}  // namespace apollo
