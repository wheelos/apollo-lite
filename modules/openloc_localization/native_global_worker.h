// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#pragma once

#include <condition_variable>
#include <deque>
#include <exception>
#include <fstream>
#include <functional>
#include <future>
#include <mutex>
#include <optional>
#include <set>
#include <thread>
#include <variant>

#include "native_global_diagnostic.pb.h"
#include "native_gga_adapter.h"
#include "native_structured_gnss_adapter.h"
#include "native_map_matcher.h"
#include "native_publication.h"
#include "native_graph_transport.h"
#include "openloc/localization/global/diagnostic_alignment.h"

namespace apollo::localization::openloc_diagnostic {

// One diagnostic global owner. It consumes copied messages, never local-owner
// pointers, and cannot feed corrections back to native ODOM.
class NativeGlobalWorker {
 public:
  using Clock = NativePublication::Clock;
  using Callback = std::function<void(const NativeGlobalProcessed&)>;
  using MapOperation = std::function<NativeMapMatch(const NativeMapJob&)>;
  explicit NativeGlobalWorker(const NativeGlobalHostConfig& config,
                              Callback processed = {},
                              MapOperation map_operation = {});
  ~NativeGlobalWorker();
  void AddOdometry(NativeOdometryDiagnostic message, Clock::time_point arrival);
  void AddGraphOdometry(NativeJointOdometryInput message, Clock::time_point arrival);
  void AddGnss(NativeGlobalGnssInput message, Clock::time_point arrival);
  void AddMap(NativeGlobalMapInput message, Clock::time_point arrival);
  struct RawMapPacket {
    drivers::PointCloud cloud;
    int64_t receive_ns = 0;
    int64_t reference_ns = 0;
    uint32_t source_points = 0;
    uint32_t no_returns = 0;
    uint32_t sample_stride = 1;
  };
  void AddRawMap(RawMapPacket packet, Clock::time_point arrival);
  void AddReset(NativeGlobalResetRequest message, Clock::time_point arrival);
  void AddResetAck(NativeGlobalResetAck message, Clock::time_point arrival);
  struct RawGgaPacket {
    std::string bytes;
    int64_t receive_ns = 0;
    uint64_t sequence = 0;
  };
  void AddRawGga(RawGgaPacket packet, Clock::time_point arrival);
  struct BestPosePacket {
    drivers::gnss::GnssBestPose message;
    int64_t receive_ns = 0;
  };
  struct HeadingPacket {
    drivers::gnss::Heading message;
    int64_t receive_ns = 0;
  };
  void AddBestPose(BestPosePacket packet, Clock::time_point arrival);
  void AddHeading(HeadingPacket packet, Clock::time_point arrival);
  NativeGlobalDiagnostic Sample(double publication_time,
                                Clock::time_point now = Clock::now());
  void Check() const;
  void Finish();

 private:
  struct Event {
    std::variant<NativeOdometryDiagnostic, NativeGlobalGnssInput,
                 NativeGlobalMapInput, RawGgaPacket,
                 NativeGlobalResetRequest, NativeGlobalResetAck,
                 RawMapPacket, NativeJointOdometryInput,
                 BestPosePacket, HeadingPacket> message;
    Clock::time_point arrival;
    std::size_t bytes;
  };
  struct Snapshot {
    NativeOdometryDiagnostic odometry;
    Clock::time_point arrival;
    std::optional<::openloc::localization::DiagnosticAlignmentResult> result;
    std::string graph_assessment_reason;
  };
  void Enqueue(Event event);
  void Run();
  ::openloc::localization::Status Process(const NativeOdometryDiagnostic& input,
                                         Clock::time_point arrival,
                                         const NativeDecodedGraph* graph = nullptr);
  ::openloc::localization::Status Process(const NativeJointOdometryInput& input,
                                         Clock::time_point arrival);
  ::openloc::localization::Status Process(const NativeGlobalGnssInput& input);
  ::openloc::localization::Status Process(const NativeGlobalMapInput& input);
  ::openloc::localization::Status Process(const RawGgaPacket& input);
  ::openloc::localization::Status Process(const BestPosePacket& input);
  ::openloc::localization::Status Process(const HeadingPacket& input);
  ::openloc::localization::Status Process(RawMapPacket input, Clock::time_point arrival);
  void CompleteMap();
  void StartPendingMap();
  void FinalizeMap(NativeMapMatch match);
  ::openloc::localization::Status Process(const NativeGlobalResetRequest& input);
  ::openloc::localization::Status Process(const NativeGlobalResetAck& input);
  ::openloc::localization::Status CheckAlignment(
      const std::string& session, uint64_t generation, int64_t measurement) const;
  void JournalControl();
  void Refresh();
  void ConfigureOwner();
  ::openloc::localization::Status OwnerResult(
      ::openloc::localization::DiagnosticAlignmentResult* result,
      std::string* graph_assessment_reason = nullptr) const;
  void Stop();
  NativeGlobalHostConfig config_;
  ::openloc::localization::DiagnosticAlignmentConfig policy_;
  std::unique_ptr<::openloc::localization::DiagnosticGlobalAlignment> owner_;
  std::unique_ptr<::openloc::localization::FixedLagGlobalAlignment> graph_owner_;
  struct ResetState {
    uint64_t generation = 1;
    std::optional<NativeGlobalResetRequest> request;
    std::set<std::string> pending;
  };
  ResetState reset_;
  ResetState published_reset_;
  int64_t reset_frontier_ns_ = 0;
  std::unique_ptr<NativeGgaAdapter> gga_;
  std::unique_ptr<NativeStructuredGnssAdapter> structured_gnss_;
  uint64_t raw_sequence_ = 0;
  std::unique_ptr<NativeMapMatcher> map_matcher_;
  MapOperation map_operation_;
  std::deque<NativeMapPose> map_history_;
  std::future<NativeMapMatch> map_future_;
  std::optional<NativeMapJob> map_job_;
  Clock::time_point map_deadline_;
  uint64_t raw_map_sequence_ = 0;
  uint64_t map_tasks_completed_ = 0;
  uint64_t local_revision_ = 0;
  uint64_t map_local_revision_ = 0;
  uint64_t map_alignment_generation_ = 0;
  Callback processed_;
  std::ofstream journal_;
  std::ofstream innovation_journal_;
  std::optional<NativeOdometryDiagnostic> latest_;
  Clock::time_point latest_arrival_;
  uint64_t last_publication_sequence_ = 0;
  std::string publication_session_;
  bool local_available_ = false;
  bool owner_has_odometry_ = false;
  uint64_t last_decision_ = 0;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<Event> queue_;
  std::size_t outstanding_ = 0;
  std::size_t bytes_ = 0;
  uint64_t processed_events_ = 0;
  bool stopping_ = false;
  std::exception_ptr error_;
  std::optional<Snapshot> snapshot_;
  std::string unavailable_model_detail_;
  std::mutex sample_mutex_;
  uint64_t publications_ = 0;
  std::optional<NativeGlobalDiagnostic> last_emitted_;
  std::thread thread_;
};

}  // namespace apollo::localization::openloc_diagnostic
