// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include <algorithm>
#include <condition_variable>
#include <future>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <thread>
#include <vector>

#include "google/protobuf/text_format.h"
#include "google/protobuf/util/json_util.h"
#include "native_lio_config.h"
#include "native_recorded_cloud.h"
#include "native_recorded_input.h"

#include "native_global_diagnostic.pb.h"
#include "native_odometry_diagnostic.pb.h"
#include "wheelos_msgs/sensor_msgs/imu.pb.h"
#include "wheelos_msgs/sensor_msgs/pointcloud.pb.h"
#include "wheelos_msgs/sensor_msgs/gnss.pb.h"

#include "cyber/cyber.h"

namespace host = apollo::localization::openloc_diagnostic;
using host::recorded::Number;
using host::recorded::Read;
using host::recorded::Time;

namespace {
struct Output {
  std::mutex mutex;
  std::condition_variable condition;
  std::ofstream stream;
  uint64_t messages = 0;
  uint64_t usable = 0;
  uint64_t last_sequence = 0;
  uint64_t transforms = 0;
  uint64_t imu_ack = 0;
  uint64_t lidar_ack = 0;
  uint64_t map_ack = 0;
  std::ofstream global_stream;
  uint64_t global_messages = 0;
  uint64_t global_full_pose = 0;
  uint64_t global_usable = 0;
  uint64_t target_graph_available = 0;
  uint64_t global_odometry_sequence = 0;
  bool global_ready = false;
  std::exception_ptr error;
};

int64_t GgaReceive(const apollo::drivers::gnss::RawData& packet) {
  const long double ns = static_cast<long double>(packet.header().timestamp_sec()) * 1e9L;
  if (!packet.has_header() || !packet.header().has_timestamp_sec() ||
      !packet.header().has_sequence_num() || packet.header().sequence_num() == 0 ||
      !packet.has_data() || !std::isfinite(ns) || ns <= 0 ||
      ns > std::numeric_limits<int64_t>::max() - 1.0L)
    throw std::invalid_argument("recorded raw GGA needs original header identity/time");
  return static_cast<int64_t>(std::round(ns));
}

void Replay(const std::filesystem::path& input,
            const host::NativeLioHostConfig& profile,
            const std::filesystem::path& output, bool prefetch_cloud,
            const std::string& global_topic, const std::string& gga_input,
            const std::string& gga_topic, const std::string& map_topic,
            bool map_admission_ack, bool target_graph) {
  host::LocalConfiguration(profile);
  if (target_graph &&
      (global_topic.empty() || !profile.has_graph() || !map_topic.empty()))
    throw std::invalid_argument(
        "target graph replay requires local graph export, global observation "
        "and no separate raw map topic");
  if (!profile.recorded_receive_time_from_header() ||
      profile.imu_measurement_to_unix_offset() != 0 ||
      profile.lidar_measurement_to_unix_offset() != 0)
    throw std::runtime_error(
        "Cyber replay requires original Unix/header receive mode");
  if (!map_topic.empty() &&
      (map_topic == profile.lidar_topic() || map_topic == profile.imu_topic() ||
       map_topic == global_topic || map_topic == gga_topic))
    throw std::invalid_argument("map replay topic must be separate from recorded retry inputs");
  if (!std::filesystem::create_directory(output))
    throw std::runtime_error("Cyber replay output must be a new directory");
  const auto imu =
      Read(input / "imu.csv", "timestamp_ns,receive_ns,ax,ay,az,gx,gy,gz", 8);
  const auto scans = Read(input / "scans.csv", "end_ns,receive_ns,points", 3);
  if (imu.empty() || scans.empty())
    throw std::runtime_error("Cyber replay requires raw IMU and scans");
  std::vector<apollo::drivers::gnss::RawData> gga;
  if (!gga_input.empty()) {
    if (std::filesystem::file_size(gga_input) > 1024 * 1024)
      throw std::runtime_error("oversized raw GGA replay bundle");
    std::ifstream file(gga_input);
    std::string line;
    int64_t previous = 0;
    while (std::getline(file, line)) {
      apollo::drivers::gnss::RawData packet;
      const auto parsed = google::protobuf::util::JsonStringToMessage(line, &packet);
      if (!parsed.ok() || !packet.IsInitialized())
        throw std::runtime_error("invalid recorded raw GGA JSON: " + parsed.ToString());
      const auto receive = GgaReceive(packet);
      if (receive <= previous || receive < Time(imu.front()[1]) ||
          receive > Time(imu.back()[1]))
        throw std::runtime_error("raw GGA receive clock outside ordered IMU window");
      previous = receive;
      gga.push_back(std::move(packet));
    }
    if (!file.eof() || gga.empty())
      throw std::runtime_error("empty or incomplete raw GGA replay input");
  }
  auto state = std::make_shared<Output>();
  state->stream.open(output / "publications.csv");
  if (!state->stream)
    throw std::runtime_error("cannot create subscriber journal");
  state->stream
      << std::setprecision(17)
      << "sequence,measurement_s,receive_s,age_s,usable,new_measurement,"
         "reason,publication_interval_ns,px,py,pz,qx,qy,qz,qw,vx,vy,vz\n";
  auto node = apollo::cyber::CreateNode("openloc_recorded_sensor_replay");
  if (!node) throw std::runtime_error("cannot create Cyber replay node");
  auto reader = node->CreateReader<host::NativeOdometryDiagnostic>(
      "/openloc/diagnostic/odometry",
      [state, session = profile.session()](
          const std::shared_ptr<host::NativeOdometryDiagnostic>& message) {
        if (message->session() != session) return;
        std::lock_guard<std::mutex> lock(state->mutex);
        try {
          if (message->trusted() || !message->diagnostic_only())
            throw std::runtime_error(
                "recorded output incorrectly promoted trust");
          if (!message->recorded_input())
            throw std::runtime_error("recorded output lacks replay label");
          state->last_sequence =
              std::max(state->last_sequence, message->source_sequence());
          ++state->messages;
          state->usable += message->usable();
          state->stream << message->source_sequence() << ','
                        << message->measurement_time() << ','
                        << message->receive_time() << ','
                        << message->measurement_age() << ','
                        << message->usable() << ','
                        << message->new_measurement() << ','
                        << message->reason() << ','
                        << message->publication_interval_ns();
          const auto& transform = message->odom_from_imu().transform();
          const auto& p = transform.translation();
          const auto& q = transform.rotation();
          const auto& v = message->velocity_in_odom();
          for (double value : {p.x(), p.y(), p.z(), q.qx(), q.qy(), q.qz(),
                               q.qw(), v.x(), v.y(), v.z()}) {
            state->stream << ',';
            if (message->usable()) state->stream << value;
          }
          state->stream << '\n';
          if (!state->stream)
            throw std::runtime_error("incomplete subscriber journal");
        } catch (...) {
          if (!state->error) state->error = std::current_exception();
        }
        state->condition.notify_all();
      });
  auto tf = node->CreateReader<apollo::transform::TransformStampeds>(
      "/openloc/diagnostic/tf",
      [state, session = profile.session()](
          const std::shared_ptr<apollo::transform::TransformStampeds>&
              message) {
        std::lock_guard<std::mutex> lock(state->mutex);
        for (const auto& item : message->transforms())
          if (item.child_frame_id().find("/" + session + "/") !=
              std::string::npos)
            ++state->transforms;
      });
  auto imu_writer =
      node->CreateWriter<apollo::drivers::gnss::Imu>(profile.imu_topic());
  auto lidar_writer =
      node->CreateWriter<apollo::drivers::PointCloud>(profile.lidar_topic());
  std::shared_ptr<apollo::cyber::Writer<apollo::drivers::gnss::RawData>> gga_writer;
  if (!gga_topic.empty()) {
    gga_writer = node->CreateWriter<apollo::drivers::gnss::RawData>(gga_topic);
    if (!gga_writer) throw std::runtime_error("cannot create raw GGA replay writer");
  }
  std::shared_ptr<apollo::cyber::Writer<apollo::drivers::PointCloud>> map_writer;
  if (!map_topic.empty()) {
    map_writer = node->CreateWriter<apollo::drivers::PointCloud>(map_topic);
    if (!map_writer) throw std::runtime_error("cannot create raw map replay writer");
  }
  const auto ack_callback = [state, session = profile.session()](bool lidar, bool map = false) {
    return [state, session, map,
            lidar](const std::shared_ptr<host::NativeRecordedIngressAck>& ack) {
      if (ack->session() != session) return;
      std::lock_guard<std::mutex> lock(state->mutex);
      auto& frontier = map ? state->map_ack : lidar ? state->lidar_ack : state->imu_ack;
      frontier = std::max(frontier, ack->source_sequence());
      state->condition.notify_all();
    };
  };
  auto imu_ack = node->CreateReader<host::NativeRecordedIngressAck>(
      profile.imu_topic() + "/recorded_ack", ack_callback(false));
  auto lidar_ack = node->CreateReader<host::NativeRecordedIngressAck>(
      profile.lidar_topic() + "/recorded_ack", ack_callback(true));
  std::shared_ptr<apollo::cyber::Reader<host::NativeRecordedIngressAck>> map_ack;
  if (map_admission_ack) {
    map_ack = node->CreateReader<host::NativeRecordedIngressAck>(
        map_topic + "/recorded_ack", ack_callback(false, true));
    if (!map_ack) throw std::runtime_error("cannot create recorded map acknowledgement reader");
  }
  if (!reader || !tf || !imu_writer || !lidar_writer || !imu_ack || !lidar_ack)
    throw std::runtime_error("cannot create recorded Cyber endpoints");
  std::shared_ptr<apollo::cyber::Reader<host::NativeGlobalDiagnostic>> global;
  if (!global_topic.empty()) {
    state->global_stream.open(output / "global_publications.csv");
    state->global_stream
        << "publication_sequence,measurement_ns,odometry_sequence,correction_id,"
           "usable,full_pose,reason";
    if (target_graph)
      state->global_stream
          << ",selected_owner,motion_error_model_id,target_graph_available,"
             "graph_assessment_reason,recorded_source_clock,local_processing_delay_s";
    state->global_stream << '\n';
    if (!state->global_stream)
      throw std::runtime_error("cannot create global subscriber journal");
    global = node->CreateReader<host::NativeGlobalDiagnostic>(
        global_topic,
        [state, session = profile.session(), target_graph,
         source_clock = target_graph && profile.graph().recorded_source_clock(),
         model = profile.graph().motion_error_model_id()](
            const std::shared_ptr<host::NativeGlobalDiagnostic>& message) {
          std::lock_guard<std::mutex> lock(state->mutex);
          try {
            if (!message->recorded_input() || message->trusted() ||
                !message->diagnostic_only())
              throw std::runtime_error("composed global output lost diagnostic bounds");
            if (message->selected_owner() != (target_graph ? "fixed_lag" : "diagnostic") ||
                message->recorded_source_clock() != source_clock ||
                (target_graph && message->motion_error_model_id() != model) ||
                (message->target_graph_available() &&
                 (!target_graph || !message->usable() ||
                  !message->full_pose_available())))
              throw std::runtime_error("composed global output changed selected owner/model");
            state->global_ready = true;
            if (message->session() == session) {
              ++state->global_messages;
              state->global_usable += message->usable();
              state->global_full_pose +=
                  message->usable() && message->full_pose_available();
              state->target_graph_available += message->target_graph_available();
              state->global_odometry_sequence = std::max(
                  state->global_odometry_sequence, message->odometry_sequence());
              state->global_stream << message->publication_sequence() << ','
                  << message->measurement_ns() << ',' << message->odometry_sequence()
                  << ',' << message->correction_id() << ',' << message->usable()
                  << ',' << message->full_pose_available() << ','
                  << std::quoted(message->reason());
              if (target_graph)
                state->global_stream << ',' << std::quoted(message->selected_owner())
                    << ',' << std::quoted(message->motion_error_model_id())
                    << ',' << message->target_graph_available()
                    << ',' << std::quoted(message->graph_assessment_reason())
                    << ',' << message->recorded_source_clock()
                    << ',' << message->local_processing_delay_s();
              state->global_stream << '\n';
              if (!state->global_stream)
                throw std::runtime_error("incomplete global subscriber journal");
            }
          } catch (...) {
            if (!state->error) state->error = std::current_exception();
          }
          state->condition.notify_all();
        });
    if (!global)
      throw std::runtime_error("cannot subscribe composed global output");
  }
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!imu_writer->HasReader() || !lidar_writer->HasReader() ||
         (gga_writer && !gga_writer->HasReader()) ||
         (map_writer && !map_writer->HasReader())) {
    if (std::chrono::steady_clock::now() >= deadline)
      throw std::runtime_error("recorded host readers not discovered");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (global) {
    std::unique_lock<std::mutex> lock(state->mutex);
    if (!state->condition.wait_for(lock, std::chrono::seconds(5),
            [&] { return state->error || state->global_ready; }))
      throw std::runtime_error("composed global publisher did not respond");
    if (state->error) std::rethrow_exception(state->error);
  }
  const int64_t first = Time(imu.front()[1]);
  const auto origin = std::chrono::steady_clock::now();
  const auto decode = [&](std::size_t index) {
    return host::recorded::ReadCloud(input, scans.at(index),
                                     profile.lidar_frame(), index + 1);
  };
  std::future<apollo::drivers::PointCloud> next_cloud;
  if (prefetch_cloud) next_cloud = std::async(std::launch::async, decode, 0);
  std::size_t next_scan = 0;
  std::size_t next_gga = 0;
  uint64_t imu_sequence = 0;
  uint64_t submissions = 0;
  uint64_t map_submissions = 0;
  int64_t maximum_send_lateness_ns = 0;
  std::ofstream send_timing(output / "send_timing.csv");
  send_timing << "sequence,receive_ns,send_lateness_ns\n";
  if (!send_timing)
    throw std::runtime_error("cannot create send timing journal");
  const auto submit = [&](const auto& writer, const auto& message,
                          uint64_t sequence, bool lidar, bool map = false) {
    const auto due = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    for (;;) {
      if (!writer->Write(message))
        throw std::runtime_error("recorded raw submission rejected");
      ++submissions;
      std::unique_lock<std::mutex> lock(state->mutex);
      const auto accepted = [&] {
        return state->error ||
               (map ? state->map_ack : lidar ? state->lidar_ack : state->imu_ack) >= sequence;
      };
      if (state->condition.wait_for(lock, std::chrono::milliseconds(10),
                                    accepted)) {
        if (state->error) std::rethrow_exception(state->error);
        return;
      }
      if (std::chrono::steady_clock::now() >= due)
        throw std::runtime_error("recorded input admission not acknowledged");
    }
  };
  for (const auto& row : imu) {
    const int64_t receive = Time(row[1]);
    while ((next_scan < scans.size() && Time(scans[next_scan][1]) <= receive) ||
           (next_gga < gga.size() && GgaReceive(gga[next_gga]) <= receive)) {
      const bool cloud_due = next_scan < scans.size() &&
          (next_gga == gga.size() ||
           Time(scans[next_scan][1]) <= GgaReceive(gga[next_gga]));
      if (cloud_due) {
        const auto& source = scans[next_scan];
        auto cloud = prefetch_cloud ? next_cloud.get() : decode(next_scan);
        if (prefetch_cloud && next_scan + 1 < scans.size())
          next_cloud = std::async(std::launch::async, decode, next_scan + 1);
        std::this_thread::sleep_until(
            origin + std::chrono::nanoseconds(Time(source[1]) - first));
        submit(lidar_writer, cloud, next_scan + 1, true);
        if (map_writer) {
          // Local admission retries must not duplicate map-owner observations.
          if (map_admission_ack) {
            submit(map_writer, cloud, next_scan + 1, false, true);
          } else if (!map_writer->Write(cloud)) {
            throw std::runtime_error("raw map replay transport rejected submission");
          }
          ++map_submissions;
        }
        ++next_scan;
      } else {
        const auto& packet = gga[next_gga];
        std::this_thread::sleep_until(
            origin + std::chrono::nanoseconds(GgaReceive(packet) - first));
        if (!gga_writer->Write(packet))
          throw std::runtime_error("raw GGA replay transport rejected submission");
        ++next_gga;
      }
    }
    std::this_thread::sleep_until(origin +
                                  std::chrono::nanoseconds(receive - first));
    apollo::drivers::gnss::Imu sample;
    sample.mutable_header()->set_timestamp_sec(receive / 1e9);
    sample.mutable_header()->set_sequence_num(++imu_sequence);
    sample.set_measurement_time(Time(row[0]) / 1e9);
    auto* force = sample.mutable_linear_acceleration();
    force->set_x(Number(row[2]));
    force->set_y(Number(row[3]));
    force->set_z(Number(row[4]));
    auto* rate = sample.mutable_angular_velocity();
    rate->set_x(Number(row[5]));
    rate->set_y(Number(row[6]));
    rate->set_z(Number(row[7]));
    const auto lateness_ns = std::max<int64_t>(
        0, std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now() -
               (origin + std::chrono::nanoseconds(receive - first)))
               .count());
    maximum_send_lateness_ns = std::max(maximum_send_lateness_ns, lateness_ns);
    send_timing << imu_sequence << ',' << receive << ',' << lateness_ns << '\n';
    submit(imu_writer, sample, imu_sequence, false);
    if (target_graph) {
      // Recorded processing backpressure, not a live throughput/deadline claim.
      std::unique_lock<std::mutex> lock(state->mutex);
      const auto processed = [&] {
        const bool graph_due = state->usable != 0 &&
            (state->global_odometry_sequence == 0 ||
             imu_sequence % profile.graph().odometry_stride() == 0);
        return state->error ||
            (state->last_sequence >= imu_sequence &&
             (!graph_due || state->global_odometry_sequence >= imu_sequence));
      };
      const auto budget = std::chrono::seconds(
          profile.graph().recorded_source_clock() ? 30 : 2);
      if (!state->condition.wait_for(lock, budget, processed))
        throw std::runtime_error("recorded target graph processing frontier not acknowledged");
      if (state->error) std::rethrow_exception(state->error);
    }
  }
  std::unique_lock<std::mutex> lock(state->mutex);
  const bool completed = state->condition.wait_for(
      lock, std::chrono::seconds(3),
      [&] {
        return state->error ||
            (state->last_sequence == imu.size() &&
             (global_topic.empty() || state->global_odometry_sequence == imu.size()));
      });
  state->stream.flush();
  if (global) {
    state->global_stream.flush();
    if (!state->global_stream)
      throw std::runtime_error("incomplete global subscriber journal");
    if (state->global_usable == 0)
      throw std::runtime_error("composed raw-native path never published usable alignment");
    if (target_graph && state->target_graph_available == 0)
      throw std::runtime_error("composed raw-native path never published available target graph");
  }
  send_timing.close();
  std::ofstream summary(output / "summary.csv");
  summary
      << "imu_submitted,scans_submitted,messages_received,usable_received,"
         "last_source_sequence,tf_received,tail_received,transport_submissions,"
         "maximum_send_lateness_ns,global_messages_received,global_full_pose_received,"
         "global_last_odometry_sequence,global_usable_received,raw_gga_submitted,"
         "raw_map_submitted";
  if (target_graph) summary << ",target_graph_available_received";
  summary << '\n' << imu.size() << ',' << next_scan << ',' << state->messages << ','
      << state->usable << ',' << state->last_sequence << ','
      << state->transforms << ',' << completed << ',' << submissions << ','
      << maximum_send_lateness_ns << ',' << state->global_messages << ','
      << state->global_full_pose << ',' << state->global_odometry_sequence << ','
      << state->global_usable << ',' << next_gga << ',' << map_submissions;
  if (target_graph) summary << ',' << state->target_graph_available;
  summary << '\n';
  summary.close();
  if (!summary || !state->stream || !send_timing)
    throw std::runtime_error("incomplete Cyber replay summary");
  if (state->error) std::rethrow_exception(state->error);
  if (!completed || state->usable == 0 || state->transforms == 0 ||
      next_scan != scans.size() || next_gga != gga.size())
    throw std::runtime_error("recorded Cyber consumption incomplete");
}
}  // namespace

int main(int argc, char** argv) {
  if (argc < 4) {
    std::cerr
        << "Usage: native_lio_cyber_replay INPUT PROFILE.pb.txt NEW_OUTPUT "
           "[--prefetch-cloud] [--global-topic TOPIC [--target-graph]] "
           "[--raw-gga INPUT.jsonl --gga-topic TOPIC] "
           "[--map-cloud-topic TOPIC [--map-admission-ack]]\n";
    return 1;
  }
  bool initialized = false;
  try {
    bool prefetch_cloud = false;
    std::string global_topic;
    std::string gga_input;
    std::string gga_topic;
    std::string map_topic;
    bool map_admission_ack = false;
    bool target_graph = false;
    for (int i = 4; i < argc; ++i) {
      const std::string option(argv[i]);
      if (option == "--prefetch-cloud" && !prefetch_cloud) {
        prefetch_cloud = true;
      } else if (option == "--map-admission-ack" && !map_admission_ack) {
        map_admission_ack = true;
      } else if (option == "--target-graph" && !target_graph) {
        target_graph = true;
      } else if (option == "--global-topic" && global_topic.empty() && i + 1 < argc) {
        global_topic = argv[++i];
        if (global_topic.empty())
          throw std::invalid_argument("global topic must not be empty");
      } else if (option == "--raw-gga" && gga_input.empty() && i + 1 < argc) {
        gga_input = argv[++i];
        if (gga_input.empty())
          throw std::invalid_argument("raw GGA input must not be empty");
      } else if (option == "--gga-topic" && gga_topic.empty() && i + 1 < argc) {
        gga_topic = argv[++i];
        if (gga_topic.empty())
          throw std::invalid_argument("raw GGA topic must not be empty");
      } else if (option == "--map-cloud-topic" && map_topic.empty() && i + 1 < argc) {
        map_topic = argv[++i];
        if (map_topic.empty())
          throw std::invalid_argument("raw map topic must not be empty");
      } else {
        throw std::invalid_argument("unknown or repeated Cyber replay option: " + option);
      }
    }
    if (gga_input.empty() != gga_topic.empty() ||
        (!gga_input.empty() && global_topic.empty()))
      throw std::invalid_argument("raw GGA replay needs input, topic and global observation");
    if (!map_topic.empty() && global_topic.empty())
      throw std::invalid_argument("raw map replay needs global observation");
    if (map_admission_ack && map_topic.empty())
      throw std::invalid_argument("map admission acknowledgement requires a separate map topic");
    std::ifstream file(argv[2]);
    const std::string text((std::istreambuf_iterator<char>(file)), {});
    host::NativeLioHostConfig profile;
    if (!file || !google::protobuf::TextFormat::ParseFromString(text, &profile))
      throw std::runtime_error("unreadable recorded Cyber profile");
    if (!apollo::cyber::Init(argv[0]))
      throw std::runtime_error("Cyber initialization failed");
    initialized = true;
    Replay(argv[1], profile, argv[3], prefetch_cloud, global_topic,
           gga_input, gga_topic, map_topic, map_admission_ack, target_graph);
    apollo::cyber::Clear();
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "Recorded Cyber replay failed: " << error.what() << '\n';
    if (initialized) apollo::cyber::Clear();
    return 1;
  }
}
