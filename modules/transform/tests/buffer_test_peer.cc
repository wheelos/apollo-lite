// Copyright 2026 WheelOS. All Rights Reserved.
#include "modules/transform/tests/buffer_test_peer.h"

#include <memory>
#include <string>

#include "modules/transform/internal/buffer_impl.h"

namespace apollo {
namespace transform {

void BufferTestPeer::ReceiveDynamic(
    Buffer& buffer, const std::shared_ptr<const TransformStampeds>& message) {
  buffer.impl_->Receive(message, false);
}

void BufferTestPeer::ReceiveStatic(
    Buffer& buffer, const std::shared_ptr<const TransformStampeds>& message) {
  buffer.impl_->Receive(message, true);
}

std::string BufferTestPeer::DynamicTopic(const Buffer& buffer) {
  const auto& reader = buffer.impl_->listener_.dynamic_reader_;
  return reader ? reader->GetChannelName() : "";
}

std::string BufferTestPeer::StaticTopic(const Buffer& buffer) {
  const auto& reader = buffer.impl_->listener_.static_reader_;
  return reader ? reader->GetChannelName() : "";
}

bool BufferTestPeer::HasNode(const Buffer& buffer) {
  return buffer.impl_->listener_.node_ != nullptr;
}

}  // namespace transform
}  // namespace apollo
