// Copyright 2026 WheelOS. All Rights Reserved.
#include "modules/transform/internal/transform_listener.h"

#include <memory>
#include <string>

#include "absl/strings/str_cat.h"

#include "cyber/cyber.h"
#include "modules/common/adapters/adapter_gflags.h"

namespace apollo {
namespace transform {
namespace internal {

TransformListener::~TransformListener() {
  dynamic_reader_.reset();
  static_reader_.reset();
  node_.reset();
}

int TransformListener::Init(const Callback& callback) {
  if (dynamic_reader_ && static_reader_) {
    return cyber::SUCC;
  }
  if (FLAGS_tf_topic.empty() || FLAGS_tf_static_topic.empty() ||
      FLAGS_tf_topic == FLAGS_tf_static_topic) {
    AERROR << "Dynamic and static TF require distinct non-empty topics.";
    return cyber::FAIL;
  }
  const std::string name =
      absl::StrCat("transform_listener_", cyber::Time::Now().ToNanosecond());
  node_ = cyber::CreateNode(name);
  if (!node_) {
    AERROR << "Failed to create transform listener node.";
    return cyber::FAIL;
  }
  cyber::proto::RoleAttributes dynamic;
  dynamic.set_channel_name(FLAGS_tf_topic);
  dynamic_reader_ = node_->CreateReader<TransformStampeds>(
      dynamic,
      [callback](const std::shared_ptr<const TransformStampeds>& message) {
        callback(message, false);
      });
  cyber::proto::RoleAttributes fixed;
  fixed.set_channel_name(FLAGS_tf_static_topic);
  fixed.mutable_qos_profile()->CopyFrom(
      cyber::transport::QosProfileConf::QOS_PROFILE_TF_STATIC);
  static_reader_ = node_->CreateReader<TransformStampeds>(
      fixed,
      [callback](const std::shared_ptr<const TransformStampeds>& message) {
        callback(message, true);
      });
  if (!dynamic_reader_ || !static_reader_) {
    AERROR << "Failed to create transform readers.";
    dynamic_reader_.reset();
    static_reader_.reset();
    node_.reset();
    return cyber::FAIL;
  }
  return cyber::SUCC;
}

}  // namespace internal
}  // namespace transform
}  // namespace apollo
