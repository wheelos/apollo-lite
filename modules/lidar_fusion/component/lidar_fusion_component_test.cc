// Copyright 2026 WheelOS. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "modules/lidar_fusion/component/lidar_fusion_component.h"

#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "google/protobuf/text_format.h"

#include "cyber/cyber.h"

extern char** environ;

namespace apollo {
namespace lidar_fusion {
namespace {

template <typename Predicate>
bool Wait(Predicate predicate) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return predicate();
}

constexpr char kPodBootstrap[] = "lidar_fusion_component_pod_bootstrap";

bool SavePodResult(const std::string& path, const std::string& bytes) {
  {
    std::ofstream file(path + ".tmp", std::ios::binary);
    file.write(bytes.data(), bytes.size());
    if (!file.good()) {
      return false;
    }
  }
  return std::rename((path + ".tmp").c_str(), path.c_str()) == 0;
}

int ReceivePodResults(const std::string& path, bool diagnostic) {
  FusionConfig config;
  if (!cyber::common::GetProtoFromFile(path, &config)) {
    return 1;
  }
  auto node =
      cyber::CreateNode("fusion_pod_receiver_" + std::to_string(getpid()));
  std::atomic<int> main_count{0}, diagnostic_count{0};
  std::atomic<bool> failed{false};
  using Pod = cyber::transport::PodMessage;
  const auto subscribe = [&](const std::string& channel,
                             const std::string& kind, std::atomic<int>* count) {
    cyber::ReaderConfig reader_config;
    std::string error;
    if (!ConfigureFusionPodReader(channel,
                                  config.publication().max_pod_frame_bytes(),
                                  &reader_config, &error)) {
      return std::shared_ptr<cyber::Reader<Pod>>{};
    }
    auto assembler = std::make_shared<FusionPodAssembler>(
        config.publication().max_pod_frame_bytes());
    return node->CreateReader<Pod>(
        reader_config,
        [&, assembler, kind, count](const std::shared_ptr<Pod>& message) {
          FusionOutput decoded;
          std::string error;
          if (!message->is_borrowed()) {
            failed.store(true);
            return;
          }
          const auto result = assembler->Push(message, &decoded, &error);
          if (result == PodAssemblyResult::REJECTED) {
            failed.store(true);
          } else if (result == PodAssemblyResult::COMPLETE) {
            if (decoded.header().frame_id() == kPodBootstrap) {
              if (!SavePodResult(path + ".pod-bootstrap-" + kind, "ok")) {
                failed.store(true);
              }
            } else {
              const int index = count->load();
              if (index >= 2 ||
                  !SavePodResult(path + ".pod-" + kind + std::to_string(index),
                                 decoded.SerializeAsString())) {
                failed.store(true);
              } else {
                count->store(index + 1);
              }
            }
          }
        });
  };
  auto main = subscribe(config.output_channel(), "main", &main_count);
  auto full = diagnostic ? subscribe(config.publication().diagnostic_channel(),
                                     "diag", &diagnostic_count)
                         : nullptr;
  if (main == nullptr || (diagnostic && full == nullptr) ||
      !SavePodResult(path + ".pod-ready", "reader-created")) {
    return 1;
  }
  const bool complete = Wait([&] {
    return failed.load() || (main_count.load() == 2 &&
                             (!diagnostic || diagnostic_count.load() == 2));
  });
  main->Shutdown();
  if (full != nullptr) {
    full->Shutdown();
  }
  return complete && !failed.load() ? 0 : 1;
}

class ComponentTest
    : public testing::TestWithParam<
          std::tuple<bool, OutputDetail, bool, PublicationTransport>> {
 protected:
  void TearDown() override {
    if (component != nullptr) {
      component->Shutdown();
    }
    if (reader != nullptr) {
      reader->Shutdown();
    }
    if (diagnostic_reader != nullptr) {
      diagnostic_reader->Shutdown();
    }
    if (pod_process > 0) {
      int status = 0;
      const bool exited = Wait([&] {
        return waitpid(pod_process, &status, WNOHANG) == pod_process;
      });
      if (!exited) {
        kill(pod_process, SIGTERM);
        waitpid(pod_process, &status, 0);
      }
      EXPECT_TRUE(exited);
      EXPECT_TRUE(WIFEXITED(status) && WEXITSTATUS(status) == 0);
    }
    if (!config_path.empty()) {
      for (const auto* suffix :
           {".pod-ready", ".pod-bootstrap-main", ".pod-bootstrap-diag",
            ".pod-main0", ".pod-main1", ".pod-diag0", ".pod-diag1"}) {
        const auto path = config_path + suffix;
        if (access(path.c_str(), F_OK) == 0) {
          EXPECT_EQ(std::remove(path.c_str()), 0);
        }
        if (access((path + ".tmp").c_str(), F_OK) == 0) {
          EXPECT_EQ(std::remove((path + ".tmp").c_str()), 0);
        }
      }
      EXPECT_EQ(std::remove(config_path.c_str()), 0);
    }
  }

  void CollectPodResults() {
    if (pod_process <= 0) {
      return;
    }
    const auto collect = [&](const std::string& kind,
                             std::vector<FusionOutput>* target) {
      while (target->size() < 2) {
        std::ifstream file(
            config_path + ".pod-" + kind + std::to_string(target->size()),
            std::ios::binary);
        if (!file.good()) {
          return;
        }
        FusionOutput frame;
        ASSERT_TRUE(frame.ParseFromIstream(&file));
        target->push_back(std::move(frame));
      }
    };
    collect("main", &results);
    collect("diag", &diagnostics);
  }

  std::shared_ptr<drivers::PointCloud> Cloud(int id) {
    auto cloud = std::make_shared<drivers::PointCloud>();
    cloud->set_frame_id("fixture_lidar");
    cloud->set_measurement_time(100 + id);
    cloud->mutable_header()->set_frame_id("fixture_lidar");
    cloud->mutable_header()->set_sequence_num(id);
    cloud->mutable_header()->set_lidar_timestamp((100ULL + id) * 1000000000ULL);
    auto* p = cloud->add_point();
    p->set_x(2);
    p->set_y(1);
    p->set_z(0.8);
    return cloud;
  }

  std::shared_ptr<LidarFusionComponent> component;
  std::shared_ptr<cyber::Reader<FusionOutput>> reader;
  std::shared_ptr<cyber::Reader<FusionOutput>> diagnostic_reader;
  std::string config_path;
  std::mutex mutex;
  std::vector<FusionOutput> results;
  std::vector<FusionOutput> diagnostics;
  std::vector<std::shared_ptr<FusionOutput>> retained_results;
  std::vector<std::shared_ptr<FusionOutput>> retained_diagnostics;
  pid_t pod_process = -1;
};

TEST_P(ComponentTest, PublishesAlignedAndTimedOutFramesOnActualCyberChannels) {
  const bool temporal_enabled = std::get<0>(GetParam());
  const auto detail = std::get<1>(GetParam());
  const bool subscribe_diagnostic = std::get<2>(GetParam());
  const auto transport = std::get<3>(GetParam());
  const char* configured_root = std::getenv("APOLLO_ROOT_DIR");
  const std::string root =
      configured_root != nullptr ? configured_root : "/apollo";
  FusionConfig config;
  ASSERT_TRUE(cyber::common::GetProtoFromFile(
      root + "/modules/lidar_fusion/conf/lidar_fusion.pb.txt", &config));
  SingleFrameFusion core;
  std::string error;
  EXPECT_FALSE(
      core.Init(config, &error));  // Deployment template is fail-closed.
  const std::string channel =
      "/test/lidar_fusion_" + std::to_string(getpid()) + "_" +
      std::to_string(temporal_enabled) + "_" + std::to_string(detail) + "_" +
      std::to_string(subscribe_diagnostic) + "_" + std::to_string(transport);
  config.set_pointcloud_channel(channel + "/cloud");
  config.set_detection_channel(channel + "/det");
  config.set_segmentation_channel(channel + "/seg");
  config.set_output_channel(channel + "/output");
  config.set_sensor_frame("fixture_lidar");
  config.mutable_sensor_to_base()->set_version("fixture-extrinsic");
  config.set_wait_ms(100);
  config.mutable_publication()->set_detail(detail);
  config.mutable_publication()->set_transport(transport);
  if (detail == FUNCTIONAL_SCENE) {
    config.mutable_publication()->set_diagnostic_channel(channel + "/full");
  }
  if (temporal_enabled) {
    auto* temporal = config.mutable_temporal();
    temporal->set_enabled(true);
    temporal->set_pose_channel(channel + "/pose");
    temporal->set_odom_frame("fixture_odom");
    temporal->set_max_dt(2);
    temporal->set_confirmation_hits(1);
    temporal->set_feedback(true);
    config.set_core_min_points(1);
  }
  ASSERT_TRUE(core.Init(config, &error)) << error;
  std::string text;
  ASSERT_TRUE(google::protobuf::TextFormat::PrintToString(config, &text));
  config_path = root + "/.lidar-fusion-component-XXXXXX";
  const int descriptor = mkstemp(config_path.data());
  ASSERT_GE(descriptor, 0);
  close(descriptor);
  {
    std::ofstream file(config_path);
    file << text;
    ASSERT_TRUE(file.good());
  }
  component = std::make_shared<LidarFusionComponent>();
  cyber::proto::TimerComponentConfig timer;
  timer.set_name("lidar_fusion_fixture_" + std::to_string(getpid()));
  timer.set_config_file_path(config_path);
  timer.set_interval(5);
  ASSERT_TRUE(component->Initialize(timer));
  auto node = cyber::CreateNode("lidar_fusion_io_" + std::to_string(getpid()));
  ASSERT_NE(node, nullptr);
  if (transport == PROTOBUF) {
    reader = node->CreateReader<FusionOutput>(
        config.output_channel(),
        [this](const std::shared_ptr<FusionOutput>& output) {
          std::lock_guard<std::mutex> lock(mutex);
          results.push_back(*output);
          retained_results.push_back(output);
        });
    ASSERT_NE(reader, nullptr);
  } else {
    std::string executable = "/proc/self/exe", mode = "--pod-receive";
    std::string subscribe = subscribe_diagnostic ? "1" : "0";
    char* args[] = {executable.data(), mode.data(), config_path.data(),
                    subscribe.data(), nullptr};
    ASSERT_EQ(posix_spawn(&pod_process, executable.c_str(), nullptr, nullptr,
                          args, environ),
              0);
    ASSERT_TRUE(Wait([&] {
      return access((config_path + ".pod-ready").c_str(), F_OK) == 0;
    }));
    const auto bootstrap = [&](const std::string& topic,
                               const std::string& kind) {
      auto writer = node->CreateWriter<cyber::transport::PodMessage>(topic);
      EXPECT_TRUE(Wait([&] { return writer->HasReader(); }));
      FusionPodPublisher publisher(writer,
                                   config.publication().max_pod_frame_bytes());
      FusionOutput marker;
      marker.mutable_header()->set_frame_id(kPodBootstrap);
      std::string error;
      bool rejected = false;
      const bool ready = Wait([&] {
        if (access((config_path + ".pod-bootstrap-" + kind).c_str(), F_OK) ==
            0) {
          return true;
        }
        if (publisher.Publish(marker, &error) == PodPublishResult::REJECTED) {
          rejected = true;
          return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        return false;
      });
      EXPECT_TRUE(ready && !rejected) << error;
    };
    bootstrap(config.output_channel(), "main");
    if (subscribe_diagnostic) {
      bootstrap(config.publication().diagnostic_channel(), "diag");
    }
  }
  if (subscribe_diagnostic && transport == PROTOBUF) {
    diagnostic_reader = node->CreateReader<FusionOutput>(
        config.publication().diagnostic_channel(),
        [this](const std::shared_ptr<FusionOutput>& output) {
          std::lock_guard<std::mutex> lock(mutex);
          diagnostics.push_back(*output);
          retained_diagnostics.push_back(output);
        });
    ASSERT_NE(diagnostic_reader, nullptr);
  }
  auto clouds =
      node->CreateWriter<drivers::PointCloud>(config.pointcloud_channel());
  auto detections =
      node->CreateWriter<DetectionFrame>(config.detection_channel());
  auto semantics =
      node->CreateWriter<SemanticFrame>(config.segmentation_channel());
  auto poses =
      temporal_enabled
          ? node->CreateWriter<LocalPose>(config.temporal().pose_channel())
          : nullptr;
  ASSERT_NE(clouds, nullptr);
  ASSERT_NE(detections, nullptr);
  ASSERT_NE(semantics, nullptr);
  ASSERT_TRUE(Wait([&] {
    return clouds->HasReader() && detections->HasReader() &&
           semantics->HasReader() &&
           (reader == nullptr || reader->HasWriter()) &&
           (!temporal_enabled || poses->HasReader()) &&
           (diagnostic_reader == nullptr || diagnostic_reader->HasWriter());
  }));
  auto pose = [&](double stamp) {
    if (poses != nullptr) {
      auto message = std::make_shared<LocalPose>();
      message->set_stamp(stamp);
      message->set_base_frame(config.base_frame());
      message->set_odom_frame(config.temporal().odom_frame());
      message->set_epoch(1);
      message->set_valid(true);
      message->mutable_translation()->set_x(0);
      EXPECT_TRUE(poses->Write(message));
      // Isolate pose delivery from the independent cloud/result callback order.
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
  };
  pose(101);
  const auto cloud = Cloud(1);
  auto detection = std::make_shared<DetectionFrame>();
  *detection->mutable_header() = cloud->header();
  detection->set_sensor_frame_id("fixture_lidar");
  detection->set_backend(config.detection_backend());
  detection->set_model_revision("synthetic-publication-fixture");
  auto* box = detection->add_detection();
  box->set_center_x(2);
  box->set_center_y(1);
  box->set_center_z(0.8);
  box->set_size_x(1);
  box->set_size_y(1);
  box->set_size_z(1);
  box->set_yaw(0);
  box->set_class_id(0);
  box->set_score(0.9);
  auto semantic = std::make_shared<SemanticFrame>();
  *semantic->mutable_header() = cloud->header();
  semantic->set_source_frame_id("fixture_lidar");
  semantic->set_source_topic(config.pointcloud_channel());
  semantic->set_point_count(1);
  auto* label = semantic->add_point_label();
  label->set_point_index(0);
  label->set_semantic_label(1);
  label->set_confidence(0.95);
  // Real callbacks see independently scheduled sources, including result-first.
  ASSERT_TRUE(semantics->Write(semantic));
  ASSERT_TRUE(clouds->Write(cloud));
  ASSERT_TRUE(detections->Write(detection));
  ASSERT_TRUE(Wait([&] {
    std::lock_guard<std::mutex> lock(mutex);
    CollectPodResults();
    return !results.empty() && (!subscribe_diagnostic || !diagnostics.empty());
  }));
  pose(102);
  ASSERT_TRUE(clouds->Write(Cloud(2)));
  ASSERT_TRUE(semantics->Write(semantic));  // Late result from the old sweep.
  ASSERT_TRUE(Wait([&] {
    std::lock_guard<std::mutex> lock(mutex);
    CollectPodResults();
    return results.size() >= 2 &&
           (!subscribe_diagnostic || diagnostics.size() >= 2);
  }));
  component->Shutdown();
  component.reset();
  if (reader != nullptr) {
    reader->Shutdown();
  }
  std::lock_guard<std::mutex> lock(mutex);
  ASSERT_EQ(results.size(), 2);
  if (transport == PROTOBUF) {
    ASSERT_EQ(retained_results.size(), results.size());
    ASSERT_EQ(retained_diagnostics.size(), diagnostics.size());
    for (std::size_t i = 0; i < results.size(); ++i) {
      EXPECT_EQ(retained_results[i]->SerializeAsString(),
                results[i].SerializeAsString());
    }
    for (std::size_t i = 0; i < diagnostics.size(); ++i) {
      EXPECT_EQ(retained_diagnostics[i]->SerializeAsString(),
                diagnostics[i].SerializeAsString());
    }
  }
  EXPECT_EQ(results[0].health().mode(), NORMAL);
  ASSERT_EQ(results[0].object_size(), 1);
  EXPECT_EQ(results[0].header().frame_id(), "base_link");
  if (detail == FUNCTIONAL_SCENE) {
    for (int i = 0; i < 2; ++i) {
      EXPECT_EQ(results[i].publication().detail(), FUNCTIONAL_SCENE);
      EXPECT_EQ(results[i].publication().source_point_count(), 1);
      EXPECT_EQ(results[i].point_evidence_size(), 0);
      EXPECT_EQ(results[i].point_decision_size(), 0);
    }
    if (subscribe_diagnostic) {
      ASSERT_EQ(diagnostics.size(), 2);
      for (int i = 0; i < 2; ++i) {
        EXPECT_FALSE(diagnostics[i].has_publication());
        auto expected = diagnostics[i];
        expected.clear_point_evidence();
        expected.clear_point_decision();
        auto scene = results[i];
        scene.clear_publication();
        EXPECT_EQ(scene.SerializeAsString(), expected.SerializeAsString());
      }
      EXPECT_EQ(diagnostics[0].point_evidence(0).raw_semantic_label(), 1);
      EXPECT_FALSE(diagnostics[1].point_evidence(0).has_raw_semantic_label());
    } else {
      EXPECT_TRUE(diagnostics.empty());
    }
  } else {
    EXPECT_FALSE(results[0].has_publication());
    EXPECT_EQ(results[0].point_evidence(0).raw_semantic_label(), 1);
    EXPECT_FALSE(results[1].point_evidence(0).has_raw_semantic_label());
  }
  EXPECT_EQ(results[1].health().mode(), GEO_ONLY);
  EXPECT_EQ(results[1].header().sequence_num(), 2);
  if (temporal_enabled) {
    EXPECT_TRUE(results[0].health().temporal_enabled());
    EXPECT_TRUE(results[1].health().temporal_enabled());
    ASSERT_EQ(results[1].object_size(), 1);
    EXPECT_EQ(results[1].object(0).source(), TRACK_PRIOR);
    EXPECT_EQ(results[1].object(0).track_id(), results[0].object(0).track_id());
    EXPECT_TRUE(results[1].object(0).track_confirmed());
  } else {
    EXPECT_EQ(results[1].unknown_size(), 1);
    EXPECT_FALSE(results[1].health().temporal_enabled());
  }
}

INSTANTIATE_TEST_SUITE_P(
    TemporalOptIn, ComponentTest,
    testing::Values(std::make_tuple(false, FULL_EVIDENCE, false, PROTOBUF),
                    std::make_tuple(true, FULL_EVIDENCE, false, PROTOBUF),
                    std::make_tuple(false, FUNCTIONAL_SCENE, true, PROTOBUF),
                    std::make_tuple(true, FUNCTIONAL_SCENE, true, PROTOBUF),
                    std::make_tuple(false, FUNCTIONAL_SCENE, false, PROTOBUF),
                    std::make_tuple(false, FULL_EVIDENCE, false, CORE_POD),
                    std::make_tuple(true, FULL_EVIDENCE, false, CORE_POD),
                    std::make_tuple(false, FUNCTIONAL_SCENE, true, CORE_POD),
                    std::make_tuple(true, FUNCTIONAL_SCENE, true, CORE_POD),
                    std::make_tuple(false, FUNCTIONAL_SCENE, false, CORE_POD)));

TEST(PodRouteTest, NoReaderIsSkippedAndSameProcessLoanIsNotCopied) {
  using Pod = cyber::transport::PodMessage;
  auto node = cyber::CreateNode("fusion_pod_intra_" + std::to_string(getpid()));
  const auto channel =
      "/test/lidar_fusion_pod_intra/" + std::to_string(getpid());
  auto writer = node->CreateWriter<Pod>(channel);
  ASSERT_NE(writer, nullptr);
  FusionPodPublisher publisher(writer, 1024);
  FusionOutput frame;
  frame.mutable_header()->set_frame_id("fixture");
  std::string error;
  EXPECT_EQ(publisher.Publish(frame, &error), PodPublishResult::NO_READERS);
  std::atomic<int> callbacks{0};
  auto reader = node->CreateReader<Pod>(
      channel, [&](const std::shared_ptr<Pod>&) { ++callbacks; });
  ASSERT_TRUE(Wait([&] { return writer->HasReader(); }));
  EXPECT_EQ(publisher.Publish(frame, &error), PodPublishResult::REJECTED);
  EXPECT_EQ(callbacks.load(), 0);
  reader->Shutdown();
  writer->Shutdown();
}

}  // namespace
}  // namespace lidar_fusion
}  // namespace apollo

int main(int argc, char** argv) {
  if (argc == 4 && std::string(argv[1]) == "--pod-receive") {
    apollo::cyber::Init(argv[0]);
    const int result = apollo::lidar_fusion::ReceivePodResults(
        argv[2], std::string(argv[3]) == "1");
    apollo::cyber::Clear();
    return result;
  }
  testing::InitGoogleTest(&argc, argv);
  apollo::cyber::Init(argv[0]);
  const int result = RUN_ALL_TESTS();
  apollo::cyber::Clear();
  return result;
}
