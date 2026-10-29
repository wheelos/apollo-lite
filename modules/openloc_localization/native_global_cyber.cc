// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "native_global_cyber.h"

#include "native_odometry_diagnostic.pb.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "cyber/time/clock.h"

namespace apollo::localization::openloc_diagnostic {
namespace {
template <typename Message>
int64_t ReceiverTime(const Message& input, bool recorded) {
  if (!recorded) return cyber::Time::Now().ToNanosecond();
  if (!input.has_header() || !input.header().has_timestamp_sec())
    throw std::invalid_argument("recorded GNSS needs original header receive time");
  return NativeMapMatcher::Nanoseconds(input.header().timestamp_sec());
}
}  // namespace

struct NativeGlobalCyber::Shared {
  void CaptureError() {
    std::lock_guard<std::mutex> lock(mutex);
    if (!error) {
      error = std::current_exception();
      AERROR << "OpenLoc global host stopped by an explicit input/publication error";
    }
  }
  void Check() const {
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (error) std::rethrow_exception(error);
    }
    worker->Check();
  }
  mutable std::mutex mutex;
  std::mutex raw_ingress;
  std::mutex map_ingress;
  uint64_t map_sequence = 0;
  std::string map_payload;
  std::shared_ptr<cyber::Writer<NativeRecordedIngressAck>> map_ack;
  uint64_t raw_callback_sequence = 0;
  std::exception_ptr error;
  std::unique_ptr<NativeGlobalWorker> worker;
};

NativeGlobalCyber::NativeGlobalCyber(const std::shared_ptr<cyber::Node>& node,
                                     const NativeGlobalHostConfig& config)
    : timer_(10, [this] { Publish(); }, false) {
  if (!node) throw std::invalid_argument("global host needs a Cyber node");
  // Validate the profile before creating any reader or publisher. Callbacks
  // retain shared state, so reader teardown cannot leave a dangling host.
  auto completion =
      std::make_shared<std::shared_ptr<cyber::Writer<NativeGlobalProcessed>>>();
  shared_ = std::make_shared<Shared>();
  shared_->worker = std::make_unique<NativeGlobalWorker>(
      config, [completion](const NativeGlobalProcessed& processed) {
        if (!(*completion)->Write(processed))
          throw std::runtime_error("global completion transport rejected write");
      });
  output_ = node->CreateWriter<NativeGlobalDiagnostic>(config.output_topic());
  processed_ = node->CreateWriter<NativeGlobalProcessed>(
      config.output_topic() + "/processed");
  if (!output_ || !processed_)
    throw std::runtime_error("cannot create global diagnostic writers");
  *completion = processed_;
  cyber::ReaderConfig reader;
  reader.channel_name = config.odometry_topic();
  reader.pending_queue_size = 256;
  if (config.has_target_graph()) {
    reader.pending_queue_size = 4;
    graph_odometry_ = node->CreateReader<NativeJointOdometryInput>(
        reader, [state = shared_](const std::shared_ptr<NativeJointOdometryInput>& input) {
          try {
            state->Check();
            if (input->ByteSizeLong() > kMaximumNativeGraphBytes)
              throw std::invalid_argument("oversized native graph transport packet");
            auto copy = *input;
            auto* source = copy.mutable_odometry();
            const double now = cyber::Clock::NowInSeconds();
            if (!source->has_publication_time() ||
                !std::isfinite(source->publication_time()) ||
                source->publication_time() <= 0 || source->publication_time() > now + 0.02)
              throw std::invalid_argument("invalid native graph publication clock");
            if (source->has_measurement_age())
              source->set_measurement_age(source->measurement_age() +
                  std::max(0.0, now - source->publication_time()));
            state->worker->AddGraphOdometry(std::move(copy),
                                            NativeGlobalWorker::Clock::now());
          } catch (...) {
            state->CaptureError();
          }
        });
  } else {
    odometry_ = node->CreateReader<NativeOdometryDiagnostic>(
      reader, [state = shared_](
                  const std::shared_ptr<NativeOdometryDiagnostic>& input) {
        try {
          state->Check();
          auto copy = *input;
          const double now = cyber::Clock::NowInSeconds();
          if (!copy.has_publication_time() ||
              !std::isfinite(copy.publication_time()) ||
              copy.publication_time() <= 0 ||
              copy.publication_time() > now + 0.02)
            throw std::invalid_argument("invalid native publication clock");
          if (copy.has_measurement_age())
            copy.set_measurement_age(copy.measurement_age() +
                std::max(0.0, now - copy.publication_time()));
          state->worker->AddOdometry(std::move(copy),
                                    NativeGlobalWorker::Clock::now());
        } catch (...) {
          state->CaptureError();
        }
      });
  }
  reader.channel_name = config.gnss_topic();
  reader.pending_queue_size = 128;
  if (config.has_structured_gnss()) {
    best_pose_ = node->CreateReader<drivers::gnss::GnssBestPose>(
        reader, [state = shared_, recorded = config.recorded_input()](const auto& input) {
          try {
            state->Check();
            state->worker->AddBestPose(
                {*input, ReceiverTime(*input, recorded)}, NativeGlobalWorker::Clock::now());
          } catch (...) {
            state->CaptureError();
          }
        });
    if (config.structured_gnss().has_heading()) {
      reader.channel_name = config.structured_gnss().heading().topic();
      heading_ = node->CreateReader<drivers::gnss::Heading>(
          reader, [state = shared_, recorded = config.recorded_input()](const auto& input) {
            try {
              state->Check();
              state->worker->AddHeading(
                  {*input, ReceiverTime(*input, recorded)}, NativeGlobalWorker::Clock::now());
            } catch (...) {
              state->CaptureError();
            }
          });
    }
  } else if (config.has_raw_gga()) {
    raw_gga_ = node->CreateReader<drivers::gnss::RawData>(
        reader, [state = shared_, recorded = config.recorded_input()](
                    const std::shared_ptr<drivers::gnss::RawData>& input) {
          try {
            std::lock_guard<std::mutex> serial(state->raw_ingress);
            state->Check();
            if (!input->has_data())
              throw std::invalid_argument("raw GGA packet needs data");
            uint64_t sequence = ++state->raw_callback_sequence;
            int64_t receive = cyber::Time::Now().ToNanosecond();
            if (recorded) {
              if (!input->has_header() || !input->header().has_sequence_num() ||
                  input->header().sequence_num() == 0)
                throw std::invalid_argument("recorded GGA needs original source sequence");
              sequence = input->header().sequence_num();
              const long double ns =
                  static_cast<long double>(input->header().timestamp_sec()) * 1e9L;
              if (!input->header().has_timestamp_sec() || !std::isfinite(ns) ||
                  ns <= 0 || ns > std::numeric_limits<int64_t>::max() - 1.0L)
                throw std::invalid_argument("recorded GGA needs original Unix receive time");
              receive = static_cast<int64_t>(std::round(ns));
            }
            state->worker->AddRawGga(
                {input->data(), receive, sequence},
                NativeGlobalWorker::Clock::now());
          } catch (...) {
            state->CaptureError();
          }
        });
  } else {
    gnss_ = node->CreateReader<NativeGlobalGnssInput>(
      reader, [state = shared_](const std::shared_ptr<NativeGlobalGnssInput>& input) {
        try {
          state->Check();
          state->worker->AddGnss(*input, NativeGlobalWorker::Clock::now());
        } catch (...) {
          state->CaptureError();
        }
      });
  }
  if (config.has_map_topic()) {
    reader.channel_name = config.map_topic();
    reader.pending_queue_size = 128;
    map_ = node->CreateReader<NativeGlobalMapInput>(
        reader, [state = shared_](const std::shared_ptr<NativeGlobalMapInput>& input) {
          try {
            state->Check();
            state->worker->AddMap(*input, NativeGlobalWorker::Clock::now());
          } catch (...) {
            state->CaptureError();
          }
        });
  }
  if (config.has_reset_topic()) {
    reader.channel_name = config.reset_topic();
    reader.pending_queue_size = 32;
    reset_ = node->CreateReader<NativeGlobalResetRequest>(
        reader, [state = shared_](const auto& input) {
          try {
            state->Check();
            state->worker->AddReset(*input, NativeGlobalWorker::Clock::now());
          } catch (...) {
            state->CaptureError();
          }
        });
    reader.channel_name = config.reset_topic() + "/ack";
    reset_ack_ = node->CreateReader<NativeGlobalResetAck>(
        reader, [state = shared_](const auto& input) {
          try {
            state->Check();
            state->worker->AddResetAck(*input, NativeGlobalWorker::Clock::now());
          } catch (...) {
            state->CaptureError();
          }
        });
  }
  if (config.has_online_map() && !config.has_target_graph()) {
    const auto ack_session = config.online_map().recorded_ack_session();
    if (config.online_map().has_recorded_ack_session()) {
      shared_->map_ack = node->CreateWriter<NativeRecordedIngressAck>(
          config.online_map().cloud_topic() + "/recorded_ack");
      if (!shared_->map_ack) {
        StopInputs();
        throw std::runtime_error("cannot create recorded raw map acknowledgements");
      }
    }
    reader.channel_name = config.online_map().cloud_topic();
    reader.pending_queue_size = 2;
    raw_map_ = node->CreateReader<drivers::PointCloud>(
        reader, [state = shared_, recorded = config.recorded_input(), ack_session](
                    const std::shared_ptr<drivers::PointCloud>& input) {
          try {
            state->Check();
            if (input->point_size() > 200000 ||
                input->ByteSizeLong() > 8 * 1024 * 1024)
              throw std::invalid_argument("oversized raw map transport message");
            const int64_t receive = recorded
                ? NativeMapMatcher::Nanoseconds(input->header().timestamp_sec())
                : cyber::Time::Now().ToNanosecond();
            if (state->map_ack) {
              const auto sequence = input->header().sequence_num();
              if (!input->has_header() || !input->header().has_sequence_num() ||
                  sequence == 0)
                throw std::invalid_argument("recorded raw map requires original source identity");
              const auto payload = input->SerializeAsString();
              {
                std::lock_guard<std::mutex> lock(state->map_ingress);
                if (sequence == state->map_sequence) {
                  if (payload != state->map_payload)
                    throw std::invalid_argument("conflicting recorded raw map retry");
                } else {
                  if (sequence != state->map_sequence + 1)
                    throw std::invalid_argument("recorded raw map source sequence skipped");
                  state->worker->AddRawMap({*input, receive}, NativeGlobalWorker::Clock::now());
                  state->map_sequence = sequence;
                  state->map_payload = payload;
                }
              }
              NativeRecordedIngressAck accepted;
              accepted.set_session(ack_session);
              accepted.set_source_sequence(sequence);
              if (!state->map_ack->Write(accepted))
                throw std::runtime_error("recorded raw map acknowledgement failed");
            } else {
              state->worker->AddRawMap({*input, receive}, NativeGlobalWorker::Clock::now());
            }
          } catch (...) {
            state->CaptureError();
          }
        });
  }
  if ((!odometry_ && !graph_odometry_) || (!gnss_ && !raw_gga_ && !best_pose_) ||
      (config.structured_gnss().has_heading() && !heading_) ||
      (config.has_map_topic() && !map_) ||
      (config.has_online_map() && !config.has_target_graph() && !raw_map_) ||
      (config.has_reset_topic() && (!reset_ || !reset_ack_))) {
    StopInputs();
    throw std::runtime_error("cannot create global diagnostic readers");
  }
  timer_.Start();
}

NativeGlobalCyber::~NativeGlobalCyber() {
  timer_.Stop();
  StopInputs();
  shared_.reset();
}

void NativeGlobalCyber::Check() const {
  shared_->Check();
}

void NativeGlobalCyber::StopInputs() {
  // Node retains shared readers even after the host releases its handles.
  if (odometry_) odometry_->Shutdown();
  if (graph_odometry_) graph_odometry_->Shutdown();
  if (gnss_) gnss_->Shutdown();
  if (raw_gga_) raw_gga_->Shutdown();
  if (best_pose_) best_pose_->Shutdown();
  if (heading_) heading_->Shutdown();
  if (map_) map_->Shutdown();
  if (raw_map_) raw_map_->Shutdown();
  if (reset_) reset_->Shutdown();
  if (reset_ack_) reset_ack_->Shutdown();
  shared_->map_ack.reset();
  odometry_.reset();
  graph_odometry_.reset();
  gnss_.reset();
  raw_gga_.reset();
  best_pose_.reset();
  heading_.reset();
  map_.reset();
  raw_map_.reset();
  reset_.reset();
  reset_ack_.reset();
}

void NativeGlobalCyber::Publish() {
  try {
    Check();
    const auto message = shared_->worker->Sample(cyber::Clock::NowInSeconds());
    if (!output_->Write(message))
      throw std::runtime_error("global diagnostic transport rejected write");
  } catch (...) {
    shared_->CaptureError();
  }
}

void NativeGlobalCyber::Finish() {
  timer_.Stop();
  StopInputs();
  shared_->worker->Finish();
  Check();
}

}  // namespace apollo::localization::openloc_diagnostic
