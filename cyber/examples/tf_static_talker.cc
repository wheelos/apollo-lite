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

#include <iostream>
#include <string>

#include "cyber/cyber.h"
#include "cyber/time/time.h"
#include "cyber/transport/qos/qos_profile_conf.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common_msgs/transform_msgs/transform.pb.h"

using apollo::cyber::Time;
using apollo::cyber::proto::RoleAttributes;
using apollo::cyber::transport::QosProfileConf;
using apollo::transform::TransformStamped;
using apollo::transform::TransformStampeds;

int main(int argc, char* argv[]) {
  apollo::cyber::Init(argv[0]);

  std::string channel = FLAGS_tf_static_topic;
  if (argc > 1) {
    channel = argv[1];
  }

  auto node = apollo::cyber::CreateNode("tf_static_talker");

  RoleAttributes attr;
  attr.set_channel_name(channel);
  attr.mutable_qos_profile()->CopyFrom(QosProfileConf::QOS_PROFILE_TF_STATIC);
  auto writer = node->CreateWriter<TransformStampeds>(attr);

  auto msg = std::make_shared<TransformStampeds>();
  const double now_sec = Time::Now().ToSecond();
  msg->mutable_header()->set_timestamp_sec(now_sec);
  msg->mutable_header()->set_module_name(node->Name());

  TransformStamped* transform = msg->add_transforms();
  transform->mutable_header()->set_timestamp_sec(now_sec);
  transform->mutable_header()->set_frame_id("world");
  transform->set_child_frame_id("demo_sensor");
  transform->mutable_transform()->mutable_translation()->set_x(1.0);
  transform->mutable_transform()->mutable_translation()->set_y(2.0);
  transform->mutable_transform()->mutable_translation()->set_z(3.0);
  transform->mutable_transform()->mutable_rotation()->set_qx(0.0);
  transform->mutable_transform()->mutable_rotation()->set_qy(0.0);
  transform->mutable_transform()->mutable_rotation()->set_qz(0.0);
  transform->mutable_transform()->mutable_rotation()->set_qw(1.0);

  writer->Write(msg);
  std::cout << "published one TF static snapshot on " << channel
            << ", waiting for late-join readers" << std::endl;

  apollo::cyber::WaitForShutdown();
  return 0;
}