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

#include "modules/transform/components/static_transform_component.h"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/message_util.h"
#include "modules/common/vehicle_calibration/registry.h"
#include "modules/transform/calibration/static_transform_loader.h"

namespace apollo {
namespace transform {

bool StaticTransformComponent::Init() {
  common::vehicle_calibration::Registry registry;
  TransformStampeds transforms;
  if (!registry.LoadFromEnvironment() ||
      !LoadStaticTransforms(registry, &transforms)) {
    AERROR << "Failed to load selected vehicle calibration.";
    return false;
  }
  cyber::proto::RoleAttributes attr;
  attr.set_channel_name(FLAGS_tf_static_topic);
  attr.mutable_qos_profile()->CopyFrom(
      cyber::transport::QosProfileConf::QOS_PROFILE_TF_STATIC);
  writer_ = node_->CreateWriter<TransformStampeds>(attr);
  if (writer_ == nullptr) {
    AERROR << "Failed to create static transform writer.";
    return false;
  }
  for (const auto& transform : transforms.transforms()) {
    AINFO << "Broadcast static transform, frame id ["
          << transform.header().frame_id() << "], child frame id ["
          << transform.child_frame_id() << "]";
  }
  common::util::FillHeader(node_->Name(), &transforms);
  if (!writer_->Write(transforms)) {
    AERROR << "Failed to publish the selected static transform tree.";
    return false;
  }
  return true;
}

CYBER_REGISTER_COMPONENT(StaticTransformComponent)

}  // namespace transform
}  // namespace apollo
