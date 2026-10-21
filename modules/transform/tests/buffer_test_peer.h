// Copyright 2026 WheelOS. All Rights Reserved.
#pragma once

#include <memory>
#include <string>

#include "modules/transform/buffer.h"

namespace apollo {
namespace transform {

// Test-only ingress uses exactly the same validation as transport callbacks.
class BufferTestPeer {
 public:
  static void ReceiveDynamic(
      Buffer& buffer,  // NOLINT
      const std::shared_ptr<const TransformStampeds>& message);
  static void ReceiveStatic(
      Buffer& buffer,  // NOLINT
      const std::shared_ptr<const TransformStampeds>& message);
  static std::string DynamicTopic(const Buffer& buffer);
  static std::string StaticTopic(const Buffer& buffer);
  static bool HasNode(const Buffer& buffer);
};

}  // namespace transform
}  // namespace apollo
