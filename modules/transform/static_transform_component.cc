/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/transform/static_transform_component.h"

#include <string>
#include <utility>
#include <vector>

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/message_util.h"
#include "modules/transform/static_transform_loader.h"

namespace apollo {
namespace transform {

bool StaticTransformComponent::Init() {
  if (!registry_.LoadFromEnvironment()) {
    AERROR << "Failed to load selected vehicle calibration profile.";
    return false;
  }
  cyber::proto::RoleAttributes attr;
  attr.set_channel_name(FLAGS_tf_static_topic);
  attr.mutable_qos_profile()->CopyFrom(
      cyber::transport::QosProfileConf::QOS_PROFILE_TF_STATIC);
  writer_ = node_->CreateWriter<TransformStampeds>(attr);
  if (!SendTransforms()) {
    AERROR << "Failed to send static transforms.";
    return false;
  }
  return true;
}

bool StaticTransformComponent::SendTransforms() {
  TransformStampeds transforms;
  if (!LoadStaticTransforms(registry_, &transforms)) {
    return false;
  }
  for (const auto& transform : transforms.transforms()) {
    AINFO << "Broadcast static transform, frame id ["
          << transform.header().frame_id() << "], child frame id ["
          << transform.child_frame_id() << "]";
  }
  std::vector<TransformStamped> transform_vector(
      transforms.transforms().begin(), transforms.transforms().end());
  SendTransform(transform_vector);
  return true;
}

void StaticTransformComponent::SendTransform(
    const std::vector<TransformStamped>& msgtf) {
  for (auto it_in = msgtf.begin(); it_in != msgtf.end(); ++it_in) {
    bool match_found = false;
    for (auto& it_msg : *transform_stampeds_.mutable_transforms()) {
      if (it_in->child_frame_id() == it_msg.child_frame_id()) {
        it_msg = *it_in;
        match_found = true;
        break;
      }
    }
    if (!match_found) {
      *transform_stampeds_.add_transforms() = *it_in;
    }
  }

  common::util::FillHeader(node_->Name(), &transform_stampeds_);
  writer_->Write(transform_stampeds_);
}

CYBER_REGISTER_COMPONENT(StaticTransformComponent)

}  // namespace transform
}  // namespace apollo
