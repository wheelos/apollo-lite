// Copyright 2026 WheelOS. All Rights Reserved.
#pragma once

#include <functional>
#include <memory>

#include "wheelos_msgs/transform_msgs/transform.pb.h"

#include "cyber/node/node.h"

namespace apollo {
namespace transform {

class BufferTestPeer;

namespace internal {

class TransformListener {
 public:
  using Callback = std::function<void(
      const std::shared_ptr<const TransformStampeds>&, bool)>;

  ~TransformListener();
  int Init(const Callback& callback);

 private:
  friend class ::apollo::transform::BufferTestPeer;

  std::unique_ptr<cyber::Node> node_;
  std::shared_ptr<cyber::Reader<TransformStampeds>> dynamic_reader_;
  std::shared_ptr<cyber::Reader<TransformStampeds>> static_reader_;
};

}  // namespace internal
}  // namespace transform
}  // namespace apollo
