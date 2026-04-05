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

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

#include "cyber/cyber.h"
#include "cyber/time/time.h"
#include "cyber/transport/qos/qos_profile_conf.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common_msgs/transform_msgs/transform.pb.h"

using apollo::cyber::Time;
using apollo::cyber::proto::RoleAttributes;
using apollo::cyber::transport::QosProfileConf;
using apollo::transform::TransformStampeds;

int main(int argc, char* argv[]) {
  apollo::cyber::Init(argv[0]);

  std::string channel = FLAGS_tf_static_topic;
  if (argc > 1) {
    channel = argv[1];
  }

  int wait_seconds = 5;
  if (argc > 2) {
    wait_seconds = std::max(1, std::atoi(argv[2]));
  }

  auto node = apollo::cyber::CreateNode(
      "tf_static_probe_" + std::to_string(Time::Now().ToNanosecond()));

  RoleAttributes attr;
  attr.set_channel_name(channel);
  attr.mutable_qos_profile()->CopyFrom(QosProfileConf::QOS_PROFILE_TF_STATIC);

  std::atomic<bool> received(false);
  std::atomic<int> transform_count(0);

  auto reader = node->CreateReader<TransformStampeds>(
      attr, [&](const std::shared_ptr<TransformStampeds>& msg) {
        transform_count.store(msg->transforms_size());
        received.store(true);
        std::cout << "received " << msg->transforms_size()
                  << " transform(s) from " << channel << std::endl;
        for (int index = 0; index < msg->transforms_size(); ++index) {
          const auto& transform = msg->transforms(index);
          std::cout << "  [" << index << "] " << transform.header().frame_id()
                    << " -> " << transform.child_frame_id() << std::endl;
        }
      });

  (void)reader;

  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::seconds(wait_seconds);
  while (apollo::cyber::OK() && !received.load() &&
         std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  if (!received.load()) {
    std::cerr << "timeout waiting for TF static snapshot on " << channel
              << " after " << wait_seconds << " second(s)" << std::endl;
    return 2;
  }

  std::cout << "probe finished with " << transform_count.load()
            << " transform(s)" << std::endl;
  return 0;
}