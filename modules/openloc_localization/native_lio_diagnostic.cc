// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include <cmath>
#include <fstream>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <thread>

#include "google/protobuf/text_format.h"
#include "native_cyber_publisher.h"
#include "native_lio_config.h"
#include "native_lio_worker.h"
#include "native_source_ingress.h"

#include "wheelos_msgs/sensor_msgs/imu.pb.h"
#include "wheelos_msgs/sensor_msgs/pointcloud.pb.h"

namespace host = apollo::localization::openloc_diagnostic;
namespace u = ::openloc::localization::unified;

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: native_lio_diagnostic CONFIG.pb.txt\n";
    return 1;
  }
  bool initialized = false;
  try {
    std::ifstream file(argv[1]);
    const std::string text((std::istreambuf_iterator<char>(file)),
                           std::istreambuf_iterator<char>());
    host::NativeLioHostConfig profile;
    if (!file || !google::protobuf::TextFormat::ParseFromString(text, &profile))
      throw std::runtime_error("cannot read native LIO host profile");
    const auto config = host::LocalConfiguration(profile);
    const auto extrinsic = host::LidarExtrinsic(profile);
    if (profile.recorded_receive_time_from_header())
      std::cerr << "OpenLoc RECORDED DIAGNOSTIC MODE: original header receive "
                   "time, not live wall-clock freshness\n";
    if (!apollo::cyber::Init(argv[0]))
      throw std::runtime_error("Cyber initialization failed");
    initialized = true;
    {
      std::shared_ptr<apollo::cyber::Node> node =
          apollo::cyber::CreateNode("openloc_native_lio_diagnostic");
      if (!node) throw std::runtime_error("cannot create diagnostic node");
      host::NativeCyberPublisher publisher(
          node, config.max_sample_age,
          profile.recorded_receive_time_from_header());
      std::shared_ptr<apollo::cyber::Writer<host::NativeJointOdometryInput>> graph_output;
      host::NativePublication graph_publication(
          config.max_sample_age, profile.graph().recorded_source_clock());
      std::mutex graph_publish_mutex;
      host::NativeLioWorker::GraphCallback graph_callback;
      if (profile.has_graph()) {
        graph_output = node->CreateWriter<host::NativeJointOdometryInput>(
            profile.graph().output_topic());
        if (!graph_output) throw std::runtime_error("cannot create native graph writer");
        graph_callback = [&](const host::NativeGraphFrame& frame, auto arrival) {
          std::lock_guard<std::mutex> serial(graph_publish_mutex);
          graph_publication.Update(frame.nominal, frame.integrity, arrival);
          auto odometry = graph_publication.Sample(apollo::cyber::Time::Now().ToSecond());
          odometry.set_recorded_input(profile.recorded_receive_time_from_header());
          // Source-clock replay retains actual processing delay separately.
          // Live mode never renews source time to manufacture freshness.
          const std::string unavailable_detail = odometry.detail();
          const auto packet = odometry.usable()
              ? host::EncodeNativeGraph(frame, std::move(odometry),
                    profile.graph().motion_error_model_id())
              : host::UnavailableNativeGraph(std::move(odometry),
                    profile.graph().motion_error_model_id(), unavailable_detail);
          if (!graph_output->Write(packet))
            throw std::runtime_error("native graph transport rejected write");
          return packet.model_available() && packet.odometry().usable();
        };
      }
      std::shared_ptr<apollo::cyber::Writer<host::NativeRecordedIngressAck>>
          imu_ack;
      std::shared_ptr<apollo::cyber::Writer<host::NativeRecordedIngressAck>>
          lidar_ack;
      if (profile.recorded_receive_time_from_header()) {
        imu_ack = node->CreateWriter<host::NativeRecordedIngressAck>(
            profile.imu_topic() + "/recorded_ack");
        lidar_ack = node->CreateWriter<host::NativeRecordedIngressAck>(
            profile.lidar_topic() + "/recorded_ack");
        if (!imu_ack || !lidar_ack)
          throw std::runtime_error(
              "cannot create recorded ingress acknowledgements");
      }
      const auto acknowledge = [&](const auto& writer, uint64_t sequence) {
        if (!writer) return;
        host::NativeRecordedIngressAck ack;
        ack.set_session(profile.session());
        ack.set_source_sequence(sequence);
        if (!writer->Write(ack))
          throw std::runtime_error("recorded ingress acknowledgement failed");
      };
      host::NativeLioWorker worker(
          config, profile.bootstrap_voxel_m(), profile.journal(),
          [&](const auto& state, const auto& integrity, auto arrival) {
            publisher.Update(state, integrity, arrival);
          },
          std::chrono::milliseconds(profile.lidar_commit_delay_ms()),
          std::chrono::milliseconds(profile.startup_task_timeout_ms()), {},
          std::move(graph_callback),
          profile.has_graph() ? profile.graph().odometry_stride() : 1);
      host::NativeSourceIngress ingress(
          profile.journal() + ".source.csv", profile.require_live_source_sequences());
      std::mutex input_mutex;
      std::exception_ptr error;
      uint64_t imu_sequence = 0;
      uint64_t scan_sequence = 0;
      std::string last_recorded_imu;
      std::string last_recorded_scan;
      const auto capture_failure = [&](host::NativeSourceIngress::Stream stream,
                                       double receive) {
        const auto failure = std::current_exception();
        std::lock_guard<std::mutex> lock(input_mutex);
        if (!error) {
          error = failure;
          try {
            ingress.Failure(stream, receive, failure);
          } catch (...) {
            AERROR << "OpenLoc source failure journal could not be written";
            error = std::current_exception();
          }
          AERROR << "OpenLoc raw source admission failed";
          if (stream != host::NativeSourceIngress::Stream::kPublisher) {
            try {
              std::string detail;
              try {
                std::rethrow_exception(failure);
              } catch (const std::exception& cause) {
                detail = cause.what();
              } catch (...) {
                detail = "non-standard native host exception";
              }
              publisher.Invalidate({u::Reason::INVALID_INPUT,
                                    "Native host stopped: " + detail});
              if (graph_output) {
                std::lock_guard<std::mutex> serial(graph_publish_mutex);
                const std::string reason = "Native host stopped: " + detail;
                graph_publication.Invalidate({u::Reason::INVALID_INPUT, reason});
                auto source = graph_publication.Sample(
                    apollo::cyber::Time::Now().ToSecond());
                source.set_recorded_input(profile.recorded_receive_time_from_header());
                const auto packet = host::UnavailableNativeGraph(
                    std::move(source), profile.graph().motion_error_model_id(), reason);
                if (!graph_output->Write(packet))
                  throw std::runtime_error("terminal native graph publication failed");
              }
            } catch (...) {
              AERROR << "OpenLoc terminal unavailable publication failed";
              try {
                ingress.Failure(host::NativeSourceIngress::Stream::kPublisher,
                                receive, std::current_exception());
              } catch (...) {
                AERROR << "OpenLoc terminal publisher failure journal could not be written";
                error = std::current_exception();
              }
            }
          }
        }
      };
      struct Inputs {
        std::shared_ptr<apollo::cyber::Reader<apollo::drivers::gnss::Imu>> imu;
        std::shared_ptr<apollo::cyber::Reader<apollo::drivers::PointCloud>> lidar;
        void Shutdown() {
          if (imu) imu->Shutdown();
          if (lidar) lidar->Shutdown();
        }
        ~Inputs() { Shutdown(); }
      } inputs;
      apollo::cyber::ReaderConfig reader_config;
      reader_config.pending_queue_size = 256;
      reader_config.channel_name = profile.imu_topic();
      inputs.imu = node->CreateReader<apollo::drivers::gnss::Imu>(
          reader_config,
          [&](const std::shared_ptr<apollo::drivers::gnss::Imu>& message) {
            const auto arrival = host::NativePublication::Clock::now();
            const double receive = profile.recorded_receive_time_from_header()
                                       ? message->header().timestamp_sec()
                                       : apollo::cyber::Time::Now().ToSecond();
            try {
              std::unique_lock<std::mutex> lock(input_mutex);
              if (error) return;
              if (!message->has_measurement_time() ||
                  (profile.recorded_receive_time_from_header() &&
                   (!message->has_header() ||
                    !message->header().has_timestamp_sec() ||
                    !message->header().has_sequence_num() ||
                    message->header().sequence_num() == 0)) ||
                  !message->has_linear_acceleration() ||
                  !message->has_angular_velocity())
                throw std::runtime_error("incomplete raw IMU");
              if (imu_ack) {
                const uint64_t sequence = message->header().sequence_num();
                if (sequence == imu_sequence) {
                  if (message->SerializeAsString() != last_recorded_imu)
                    throw std::runtime_error("conflicting recorded IMU retry");
                  lock.unlock();
                  acknowledge(imu_ack, sequence);
                  return;
                }
                if (sequence != imu_sequence + 1)
                  throw std::runtime_error(
                      "recorded IMU source sequence skipped");
              }
              const auto& force = message->linear_acceleration();
              const auto& rate = message->angular_velocity();
              if (!force.has_x() || !force.has_y() || !force.has_z() ||
                  !rate.has_x() || !rate.has_y() || !rate.has_z())
                throw std::runtime_error("missing calibrated FLU IMU axes");
              u::ImuSample sample;
              sample.stamp = {message->measurement_time() +
                                  profile.imu_measurement_to_unix_offset(),
                              receive, imu_sequence + 1, "unix"};
              sample.acceleration = {force.x(), force.y(), force.z()};
              sample.angular_velocity = {rate.x(), rate.y(), rate.z()};
              const auto sequence_header = message->has_header() &&
                      message->header().has_sequence_num()
                  ? std::optional<uint32_t>(message->header().sequence_num())
                  : std::nullopt;
              ingress.Admit(host::NativeSourceIngress::Stream::kImu,
                  sequence_header, receive,
                  [&] { worker.AddImu(std::move(sample), arrival); });
              const auto sequence = ++imu_sequence;
              if (imu_ack) last_recorded_imu = message->SerializeAsString();
              lock.unlock();
              acknowledge(imu_ack, sequence);
            } catch (...) {
              capture_failure(host::NativeSourceIngress::Stream::kImu, receive);
            }
          });
      reader_config.pending_queue_size = 2;
      reader_config.channel_name = profile.lidar_topic();
      inputs.lidar = node->CreateReader<apollo::drivers::PointCloud>(
          reader_config,
          [&](const std::shared_ptr<apollo::drivers::PointCloud>& message) {
            const auto arrival = host::NativePublication::Clock::now();
            const double receive = profile.recorded_receive_time_from_header()
                                       ? message->header().timestamp_sec()
                                       : apollo::cyber::Time::Now().ToSecond();
            // Decode separately from the IMU enqueue mutex.
            try {
              if (!message->has_measurement_time() ||
                  (profile.recorded_receive_time_from_header() &&
                   (!message->has_header() ||
                    !message->header().has_timestamp_sec() ||
                    !message->header().has_sequence_num() ||
                    message->header().sequence_num() == 0)) ||
                  message->frame_id() != config.lidar_frame ||
                  message->point_size() == 0 || message->point_size() > 1000000 ||
                  message->ByteSizeLong() > 32 * 1024 * 1024)
                throw std::runtime_error("invalid bounded raw point cloud");
              u::LidarScan scan;
              scan.stamp = {message->measurement_time() +
                                profile.lidar_measurement_to_unix_offset(),
                            receive, 0, "unix"};
              scan.frame_id = config.lidar_frame;
              scan.calibration_id = config.lidar_calibration_id;
              scan.base_from_lidar = extrinsic;
              std::size_t valid_points = 0;
              std::size_t no_returns = 0;
              double previous_time = 0;
              for (const auto& point : message->point()) {
                if (!point.has_timestamp() || point.timestamp() == 0 ||
                    !point.has_x() || !point.has_y() || !point.has_z() ||
                    !std::isfinite(point.x()) || !std::isfinite(point.y()) ||
                    !std::isfinite(point.z()))
                  throw std::runtime_error("raw point lacks finite XYZ/time");
                const double time =
                    static_cast<double>(point.timestamp()) / 1e9 +
                    profile.lidar_measurement_to_unix_offset();
                if (time < previous_time || time > scan.stamp.time)
                  throw std::runtime_error(
                      "point times not ordered within scan");
                previous_time = time;
                if (point.x() == 0 && point.y() == 0 && point.z() == 0)
                  ++no_returns;
                else
                  ++valid_points;
              }
              if (valid_points == 0)
                throw std::runtime_error("raw scan has no valid returns");
              const std::size_t stride =
                  (valid_points + config.lidar_max_points - 1) /
                  config.lidar_max_points;
              std::size_t valid_index = 0;
              for (const auto& point : message->point()) {
                if (point.x() == 0 && point.y() == 0 && point.z() == 0)
                  continue;
                if (valid_index++ % stride != 0) continue;
                scan.points.push_back(
                    {{point.x(), point.y(), point.z()},
                     static_cast<double>(point.timestamp()) / 1e9 +
                         profile.lidar_measurement_to_unix_offset()});
              }
              AINFO << "OpenLoc raw scan points=" << message->point_size()
                    << " no_returns=" << no_returns << " stride=" << stride
                    << " retained=" << scan.points.size();
              std::unique_lock<std::mutex> lock(input_mutex);
              if (error) return;
              if (lidar_ack) {
                const uint64_t sequence = message->header().sequence_num();
                if (sequence == scan_sequence) {
                  if (message->SerializeAsString() != last_recorded_scan)
                    throw std::runtime_error(
                        "conflicting recorded cloud retry");
                  lock.unlock();
                  acknowledge(lidar_ack, sequence);
                  return;
                }
                if (sequence != scan_sequence + 1)
                  throw std::runtime_error(
                      "recorded scan source sequence skipped");
              }
              scan.stamp.sequence = scan_sequence + 1;
              const auto sequence_header = message->has_header() &&
                      message->header().has_sequence_num()
                  ? std::optional<uint32_t>(message->header().sequence_num())
                  : std::nullopt;
              ingress.Admit(host::NativeSourceIngress::Stream::kLidar,
                  sequence_header, receive,
                  [&] { worker.AddLidar(std::move(scan), arrival); });
              const auto sequence = ++scan_sequence;
              if (lidar_ack) last_recorded_scan = message->SerializeAsString();
              lock.unlock();
              acknowledge(lidar_ack, sequence);
            } catch (...) {
              capture_failure(host::NativeSourceIngress::Stream::kLidar, receive);
            }
          });
      if (!inputs.imu || !inputs.lidar)
        throw std::runtime_error("cannot create raw input readers");
      while (apollo::cyber::OK()) {
        {
          std::lock_guard<std::mutex> lock(input_mutex);
          if (error) std::rethrow_exception(error);
        }
        try {
          worker.Check();
        } catch (...) {
          capture_failure(host::NativeSourceIngress::Stream::kWorker,
                          apollo::cyber::Time::Now().ToSecond());
          throw;
        }
        try {
          publisher.Check();
        } catch (...) {
          capture_failure(host::NativeSourceIngress::Stream::kPublisher,
                          apollo::cyber::Time::Now().ToSecond());
          throw;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      AINFO << "OpenLoc native host stopping input readers";
      inputs.Shutdown();
      {
        std::lock_guard<std::mutex> lock(input_mutex);
        if (error) std::rethrow_exception(error);
      }
      try {
        AINFO << "OpenLoc native host draining local worker";
        worker.Finish();
      } catch (...) {
        capture_failure(host::NativeSourceIngress::Stream::kWorker,
                        apollo::cyber::Time::Now().ToSecond());
        throw;
      }
      try {
        AINFO << "OpenLoc native host stopping diagnostic publisher";
        publisher.Finish();
      } catch (...) {
        capture_failure(host::NativeSourceIngress::Stream::kPublisher,
                        apollo::cyber::Time::Now().ToSecond());
        throw;
      }
      std::ofstream summary(profile.journal() + ".ingress.csv");
      summary << "imu_enqueued,scans_enqueued,recorded_receive_time\n"
              << imu_sequence << ',' << scan_sequence << ','
              << profile.recorded_receive_time_from_header() << '\n';
      summary.close();
      if (!summary) throw std::runtime_error("incomplete ingress journal");
    }
    AINFO << "OpenLoc native host clearing Cyber runtime";
    apollo::cyber::Clear();
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "OpenLoc native LIO host failed: " << error.what() << '\n';
    if (initialized) apollo::cyber::Clear();
    return 1;
  }
}
