// Copyright 2026 WheelOS All Rights Reserved.
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


#include "modules/drivers/lidar/processor/lidar_unified_component.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <iomanip>
#include <limits>
#include <sstream>

#include "modules/drivers/lidar/processor/policy/lidar_policy_common.h"
#include "modules/transform/transform_query.h"

namespace apollo {
namespace drivers {
namespace lidar {

namespace {

constexpr int kPosePrefetchLogFrequency = 10;

double ElapsedMilliseconds(uint64_t start_ns) {
  return static_cast<double>(cyber::Time::Now().ToNanosecond() - start_ns) /
         1e6;
}

std::string FormatTimestampForLog(double timestamp_sec) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(9) << timestamp_sec << "s";
  if (timestamp_sec > 0.0) {
    stream << " [" << cyber::Time(timestamp_sec).ToString() << "]";
  }
  return stream.str();
}

std::string FormatDeltaForLog(double delta_sec) {
  const double abs_delta_sec = std::fabs(delta_sec);
  std::ostringstream stream;
  stream << std::showpos << std::fixed
         << std::setprecision(abs_delta_sec >= 1.0 ? 3 : 6) << delta_sec
         << "s" << std::noshowpos;
  if (abs_delta_sec >= 86400.0) {
    stream << " (~" << std::fixed << std::setprecision(3)
           << abs_delta_sec / 86400.0 << " days)";
  } else if (abs_delta_sec >= 3600.0) {
    stream << " (~" << std::fixed << std::setprecision(3)
           << abs_delta_sec / 3600.0 << " h)";
  } else if (abs_delta_sec >= 60.0) {
    stream << " (~" << std::fixed << std::setprecision(3)
           << abs_delta_sec / 60.0 << " min)";
  } else if (abs_delta_sec > 0.0 && abs_delta_sec < 1.0) {
    stream << " (~" << std::fixed << std::setprecision(3)
           << abs_delta_sec * 1000.0 << " ms)";
  }
  return stream.str();
}

const char* DescribeTimeRelation(double delta_sec) {
  return std::fabs(delta_sec) <= 1e-6 ? "aligned"
                                      : (delta_sec > 0.0 ? "ahead"
                                                         : "behind");
}

std::string BuildPointCloudTimeSummary(const PointCloud& point_cloud) {
  double min_point_time_sec = point_cloud.measurement_time();
  double max_point_time_sec = point_cloud.measurement_time();
  ResolvePointTimestampBounds(point_cloud, &min_point_time_sec,
                              &max_point_time_sec);

  const double measurement_time_sec = point_cloud.measurement_time();
  const double now_sec = cyber::Time::Now().ToSecond();
  const double measurement_vs_now_sec = measurement_time_sec - now_sec;

  std::ostringstream message;
  message << "measurement=" << FormatTimestampForLog(measurement_time_sec)
          << ", point_range=[" << FormatTimestampForLog(min_point_time_sec)
          << ", " << FormatTimestampForLog(max_point_time_sec) << "]"
          << ", measurement_vs_now="
          << FormatDeltaForLog(measurement_vs_now_sec) << " ("
          << DescribeTimeRelation(measurement_vs_now_sec) << ")";
  if (std::fabs(max_point_time_sec - min_point_time_sec) > 1e-6) {
    message << ", scan_span="
            << FormatDeltaForLog(max_point_time_sec - min_point_time_sec);
  }
  return message.str();
}

std::string ResolvePolicyMode(const LidarUnifiedComponentConfig& config) {
#ifdef APOLLO_LIDAR_POLICY_FORCE_CPU
  (void)config;
  return "cpu";
#endif
#ifdef APOLLO_LIDAR_POLICY_FORCE_GPU
  (void)config;
  return "gpu";
#endif
  switch (config.compute_mode()) {
    case LidarUnifiedComponentConfig::COMPUTE_MODE_GPU:
      return "gpu";
    case LidarUnifiedComponentConfig::COMPUTE_MODE_CPU:
    default:
      return "cpu";
  }
}

apollo::transform::TimedTransformResolverOptions BuildTransformResolverOptions(
    const LidarUnifiedComponentConfig& config) {
  apollo::transform::TimedTransformResolverOptions options;
  options.query_timeout_sec =
      static_cast<float>(config.sensor_pose_query_timeout_sec());
  options.cache_duration_sec = config.sensor_pose_cache_duration_sec();
  options.max_extrapolation_sec =
      config.sensor_pose_cache_max_extrapolation_sec();
  return options;
}

}  // namespace

bool LidarUnifiedComponent::Init() {
  static_extrinsics_.clear();
  if (!GetProtoConfig(&config_)) {
    AERROR << "Load config failed, config file: " << ConfigFilePath();
    return false;
  }

  if (!ValidateConfig()) {
    return false;
  }

  writer_ = node_->CreateWriter<::apollo::drivers::PointCloud>(
      config_.output_channel());
  tf_buffer_ = apollo::transform::Buffer::Instance();

  const std::string policy_mode = ResolvePolicyMode(config_);
  deskew_policy_ = LidarPolicyFactory::CreateDeskewPolicy(policy_mode);
  fusion_policy_ = LidarPolicyFactory::CreateFusionPolicy(policy_mode);
  filter_policy_ = LidarPolicyFactory::CreateFilterPolicy(policy_mode);
  if (deskew_policy_ == nullptr || fusion_policy_ == nullptr ||
      filter_policy_ == nullptr) {
    AERROR << "Failed to create lidar policies for mode=" << policy_mode;
    return false;
  }
  if (!deskew_policy_->Init(config_, tf_buffer_) ||
      !fusion_policy_->Init(config_, tf_buffer_) ||
      !filter_policy_->Init(config_)) {
    AERROR << "Failed to initialize lidar policies for mode=" << policy_mode;
    return false;
  }

  ts_sanity_.SetConfig(config_.ts_sanity_min_interval_ms(),
                       config_.ts_sanity_max_interval_ms(),
                       config_.ts_sanity_max_jump_ms());
  degrade_policy_.SetConfig(config_.degrade_on_ts_anomaly(),
                            config_.ts_sanity_max_consecutive_errors());

  sensor_buffer_capacity_ =
      std::max<size_t>(2, static_cast<size_t>(config_.sensor_buffer_size()));

  auxiliary_inputs_.clear();
  auxiliary_readers_.clear();
  auxiliary_sensor_ids_by_topic_.clear();
  sensor_states_.clear();
  primary_sensor_id_.clear();
  if (config_.compensation_mode() != LidarUnifiedComponentConfig::OFF) {
    base_link_pose_resolver_ =
        std::make_unique<apollo::transform::TimedTransformResolver>(
            tf_buffer_, config_.map_frame_id(), config_.base_link_frame_id(),
            BuildTransformResolverOptions(config_));
  }

  for (const auto& input_cfg : config_.auxiliary_lidar_inputs()) {
    auxiliary_inputs_.push_back(
        SensorInput{input_cfg.topic_name(), input_cfg.time_settings()});

    auto reader = node_->CreateReader<::apollo::drivers::PointCloud>(
        input_cfg.topic_name(),
        [this, topic_name = input_cfg.topic_name()](
            const std::shared_ptr<::apollo::drivers::PointCloud>& msg) {
          OnAuxiliaryLidarMessage(topic_name, msg);
        });
    if (reader == nullptr) {
      AERROR << "Failed to create auxiliary lidar reader. topic="
             << input_cfg.topic_name();
      return false;
    }
    auxiliary_readers_.emplace(input_cfg.topic_name(), reader);
  }

  const size_t max_points = std::max<size_t>(
      1, static_cast<size_t>(config_.max_full_pointcloud_points()));
  full_pointcloud_buffer_.resize(max_points);
  fusion_flush_timer_.reset(new apollo::cyber::Timer(
      config_.fusion_flush_interval_ms(),
      [this]() { this->OnFusionFlushTimer(); }, false));
  fusion_flush_timer_->Start();

  AINFO << "LidarUnifiedComponent initialized. compute_mode=" << policy_mode
        << ", main_sensor=<auto>"
        << ", aux_count=" << auxiliary_inputs_.size()
        << ", max_points=" << max_points
        << ", fusion_wait_timeout_ms=" << config_.fusion_wait_timeout_ms()
        << ", pending_fusion_queue_size="
        << config_.pending_fusion_queue_size();
  return true;
}

bool LidarUnifiedComponent::Proc(
    const std::shared_ptr<::apollo::drivers::PointCloud>& point_cloud) {
  return OnReceiveMainLidar(point_cloud);
}

bool LidarUnifiedComponent::OnReceiveMainLidar(
    const PointCloudConstPtr& point_cloud) {
  if (point_cloud == nullptr) {
    AERROR << "Input point cloud is null";
    return false;
  }
  AINFO_EVERY(20) << "OnReceiveMainLidar measurement_time="
                  << point_cloud->measurement_time()
                  << ", points=" << point_cloud->point_size();

  const std::string sensor_id = ResolveSensorId(point_cloud, std::string());
  if (sensor_id.empty()) {
    AERROR << "Failed to resolve primary lidar sensor id from incoming frame";
    return false;
  }

  if (primary_sensor_id_.empty()) {
    primary_sensor_id_ = sensor_id;
    AINFO << "Resolved primary lidar sensor id from input stream: "
          << primary_sensor_id_;
  } else if (primary_sensor_id_ != sensor_id) {
    AERROR << "Primary lidar sensor id changed from " << primary_sensor_id_
           << " to " << sensor_id << ", drop frame to avoid TF mismatch";
    return false;
  }

  std::shared_ptr<BufferedFrame> buffered_frame;
  if (!PrepareBufferedFrame(primary_sensor_id_, point_cloud,
                            config_.primary_time_settings(),
                            &buffered_frame)) {
    AINFO_EVERY(kPosePrefetchLogFrequency)
        << "Drop primary lidar frame due to pose prefetch failure. sensor="
        << primary_sensor_id_ << ", "
        << BuildPointCloudTimeSummary(*point_cloud);
    return false;
  }
  PushToBuffer(primary_sensor_id_, buffered_frame);

  if (config_.ts_sanity_enabled()) {
    const TsSanityResult ts_result =
        ts_sanity_.Check(buffered_frame->time_contract.CanonicalAnchorSec());
    if (ts_result.status != TsSanityStatus::kOk &&
        ts_result.status != TsSanityStatus::kFirstFrame) {
      dtc_reporter_.ReportTsAnomaly(ts_result.status, ts_result.interval_ms,
                                    ts_result.consecutive_errors,
                                    primary_sensor_id_);
      const DegradeEvent event =
          degrade_policy_.OnTsAnomaly(ts_result.consecutive_errors);
      dtc_reporter_.ReportDegradeTransition(event);
    }
  }

  EnqueuePendingFusionFrame(point_cloud, primary_sensor_id_, buffered_frame);
  TryFlushPendingFusionFrames(false);
  return true;
}

bool LidarUnifiedComponent::ProcessFusionFrame(
    const PendingFusionFrame& pending_frame,
    std::vector<FrameHandle> frame_handles, FrameMetrics frame_metrics) {
  const uint64_t processing_start_ns = cyber::Time::Now().ToNanosecond();
  for (const auto& frame_handle : frame_handles) {
    if (!frame_handle.is_primary) {
      UpdateSensorTimingModel(frame_handle,
                              pending_frame.reference_timestamp_sec,
                              &frame_metrics);
    }
  }

  if (degrade_policy_.CurrentMode() == DegradeMode::kSingleLidar) {
    frame_handles.erase(
        std::remove_if(
            frame_handles.begin(), frame_handles.end(),
            [](const FrameHandle& handle) { return !handle.is_primary; }),
        frame_handles.end());
    frame_metrics.expected_sensor_count = 1;
    frame_metrics.matched_sensor_count = frame_handles.size();
    frame_metrics.missing_auxiliary_count = 0;
    frame_metrics.time_delta_exceeded_count = 0;
  }

  std::shared_ptr<::apollo::drivers::PointCloud> unified_output;
  if (!BuildUnifiedPointCloud(pending_frame.main_frame, frame_handles,
                              &frame_metrics, &unified_output)) {
    AERROR << "Failed to build unified point cloud";
    return false;
  }

  dtc_reporter_.ReportDegradeTransition(degrade_policy_.OnFrameOk());

  const uint64_t writer_start_ns = cyber::Time::Now().ToNanosecond();
  writer_->Write(unified_output);
  frame_metrics.writer_ms = ElapsedMilliseconds(writer_start_ns);
  frame_metrics.processing_ms = ElapsedMilliseconds(processing_start_ns);
  frame_metrics.end_to_end_ms =
      std::max(0.0, (cyber::Time::Now().ToSecond() -
                     pending_frame.enqueue_time_sec) *
                        1000.0);
  LogFrameMetrics(frame_metrics);
  return true;
}

void LidarUnifiedComponent::EnqueuePendingFusionFrame(
    const PointCloudConstPtr& main_frame, const std::string& primary_sensor_id,
    const std::shared_ptr<const BufferedFrame>& primary_buffered_frame) {
  if (main_frame == nullptr || primary_sensor_id.empty() ||
      primary_buffered_frame == nullptr) {
    return;
  }

  PendingFusionFrame pending_frame;
  pending_frame.main_frame = main_frame;
  pending_frame.primary_sensor_id = primary_sensor_id;
  pending_frame.primary_buffered_frame = primary_buffered_frame;
  pending_frame.reference_timestamp_sec =
      primary_buffered_frame->time_contract.CanonicalAnchorSec();
  pending_frame.enqueue_time_sec = cyber::Time::Now().ToSecond();
  pending_frame.deadline_sec =
      pending_frame.enqueue_time_sec +
      static_cast<double>(config_.fusion_wait_timeout_ms()) / 1000.0;

  std::lock_guard<std::mutex> lock(pending_fusion_mutex_);
  const size_t queue_limit = std::max<size_t>(
      1, static_cast<size_t>(config_.pending_fusion_queue_size()));
  if (pending_fusion_frames_.size() >= queue_limit) {
    total_pending_fusion_dropped_.fetch_add(1);
    AWARN << "Drop primary lidar frame because pending fusion queue is full. "
          << "queue_size=" << pending_fusion_frames_.size()
          << ", limit=" << queue_limit
          << ", ref_timestamp=" << pending_frame.reference_timestamp_sec;
    return;
  }
  pending_fusion_frames_.push_back(std::move(pending_frame));
}

void LidarUnifiedComponent::OnFusionFlushTimer() {
  TryFlushPendingFusionFrames(true);
}

void LidarUnifiedComponent::TryFlushPendingFusionFrames(
    bool flush_expired_only) {
  std::lock_guard<std::mutex> process_lock(fusion_process_mutex_);

  while (true) {
    PendingFusionFrame pending_frame;
    {
      std::lock_guard<std::mutex> lock(pending_fusion_mutex_);
      if (pending_fusion_frames_.empty()) {
        return;
      }
      pending_frame = pending_fusion_frames_.front();
    }

    FrameMetrics frame_metrics;
    std::vector<FrameHandle> frame_handles;
    const uint64_t frame_selection_start_ns =
        cyber::Time::Now().ToNanosecond();
    const bool collected = CollectNearestFrames(
        pending_frame.primary_sensor_id, pending_frame.primary_buffered_frame,
        &frame_handles, &frame_metrics);
    const double now_sec = cyber::Time::Now().ToSecond();
    const bool deadline_exceeded = now_sec >= pending_frame.deadline_sec;
    const bool all_sensors_matched =
        collected && frame_metrics.matched_sensor_count >=
                         frame_metrics.expected_sensor_count;
    const bool should_flush =
        all_sensors_matched ||
        (!flush_expired_only && auxiliary_inputs_.empty()) ||
        deadline_exceeded;

    if (!should_flush) {
      return;
    }

    {
      std::lock_guard<std::mutex> lock(pending_fusion_mutex_);
      if (pending_fusion_frames_.empty() ||
          pending_fusion_frames_.front().main_frame !=
              pending_frame.main_frame) {
        continue;
      }
      pending_fusion_frames_.pop_front();
    }

    if (!collected) {
      AERROR << "Failed to collect frames for pending fusion. ref_timestamp="
             << pending_frame.reference_timestamp_sec
             << ", deadline_exceeded=" << deadline_exceeded;
      continue;
    }
    CommitSelectedFrames(frame_handles);
    frame_metrics.frame_selection_ms =
        ElapsedMilliseconds(frame_selection_start_ns);

    frame_metrics.fusion_deadline_exceeded = deadline_exceeded;
    frame_metrics.fusion_wait_ms =
        std::max(0.0, (now_sec - pending_frame.enqueue_time_sec) * 1000.0);
    if (deadline_exceeded && !all_sensors_matched) {
      total_fusion_deadline_exceeded_.fetch_add(1);
      AWARN << "Publish lidar fusion frame after wait timeout. matched_sensors="
            << frame_metrics.matched_sensor_count << "/"
            << frame_metrics.expected_sensor_count
            << ", wait_ms=" << frame_metrics.fusion_wait_ms
            << ", ref_timestamp=" << pending_frame.reference_timestamp_sec;
    }

    if (!ProcessFusionFrame(pending_frame, std::move(frame_handles),
                            frame_metrics)) {
      AERROR << "Failed to process pending lidar fusion frame. ref_timestamp="
             << pending_frame.reference_timestamp_sec;
    }
  }
}

void LidarUnifiedComponent::PushToBuffer(
    const std::string& sensor_id,
    const std::shared_ptr<BufferedFrame>& buffered_frame) {
  const auto sensor_state = GetSensorState(sensor_id);
  if (sensor_state == nullptr || buffered_frame == nullptr) {
    return;
  }
  std::lock_guard<std::mutex> lock(sensor_state->mutex);
  if (sensor_state->frames.full() &&
      sensor_state->frames.front() != nullptr) {
    sensor_state->consumed_frame_ids.erase(
        sensor_state->frames.front()->frame_id);
  }
  sensor_state->frames.push_back(buffered_frame);
  AINFO_EVERY(20) << "Buffered lidar frame. sensor=" << sensor_id
                  << ", frame_id=" << buffered_frame->frame_id
                  << ", buffer_size=" << sensor_state->frames.size()
                  << ", canonical_anchor_ns="
                  << buffered_frame->time_contract.canonical_anchor_ns;
}

bool LidarUnifiedComponent::CollectNearestFrames(
    const std::string& primary_sensor_id,
    const std::shared_ptr<const BufferedFrame>& primary_buffered_frame,
    std::vector<FrameHandle>* frame_handles, FrameMetrics* frame_metrics) {
  if (frame_handles == nullptr || frame_metrics == nullptr ||
      primary_buffered_frame == nullptr ||
      primary_buffered_frame->point_cloud == nullptr) {
    return false;
  }
  std::vector<std::string> auxiliary_topics;
  auxiliary_topics.reserve(auxiliary_inputs_.size());
  for (const auto& auxiliary_input : auxiliary_inputs_) {
    auxiliary_topics.push_back(auxiliary_input.topic_name);
  }

  SyncGateMetrics sync_metrics;
  FrameHandle primary_handle;
  primary_handle.sensor_id = primary_sensor_id;
  primary_handle.point_cloud = primary_buffered_frame->point_cloud;
  primary_handle.buffered_frame = primary_buffered_frame;
  primary_handle.frame_id = primary_buffered_frame->frame_id;
  primary_handle.time_contract = primary_buffered_frame->time_contract;
  primary_handle.is_primary = true;
  const bool ok = sync_gate_.SelectFrames(
      primary_handle, auxiliary_topics,
      config_.max_ref_time_delta_ms(), config_.strict_auxiliary_sync(),
      [this](const std::string& topic_name, std::string* sensor_id) {
        if (sensor_id == nullptr) {
          return false;
        }
        std::lock_guard<std::mutex> lock(sensor_registry_mutex_);
        const auto it = auxiliary_sensor_ids_by_topic_.find(topic_name);
        if (it == auxiliary_sensor_ids_by_topic_.end()) {
          return false;
        }
        *sensor_id = it->second;
        return !sensor_id->empty();
      },
      [this, primary_sensor_id](
          const std::string& sensor_id, const TimeContract& reference_time,
          uint32_t max_delta_ms,
          FrameHandle* frame_handle, bool* time_delta_exceeded) {
        if (time_delta_exceeded != nullptr) {
          *time_delta_exceeded = false;
        }
        FrameLookupFailureReason failure_reason =
            FrameLookupFailureReason::kNone;
        const bool found =
            FindNearestFrame(GetSensorState(sensor_id), sensor_id,
                             reference_time,
                             max_delta_ms, frame_handle, &failure_reason);
        if (found && sensor_id != primary_sensor_id &&
            frame_handle != nullptr &&
            config_.enable_overlap_quality_gate() &&
            frame_handle->overlap_quality_weight <
                config_.auxiliary_min_overlap_quality_weight()) {
          return false;
        }
        if (!found && time_delta_exceeded != nullptr) {
          *time_delta_exceeded =
              failure_reason == FrameLookupFailureReason::kTimeDeltaExceeded;
        }
        return found;
      },
      frame_handles, &sync_metrics);

  frame_metrics->expected_sensor_count = sync_metrics.expected_sensor_count;
  frame_metrics->matched_sensor_count = sync_metrics.matched_sensor_count;
  frame_metrics->missing_auxiliary_count = sync_metrics.missing_auxiliary_count;
  frame_metrics->time_delta_exceeded_count =
      sync_metrics.time_delta_exceeded_count;
  return ok;
}

bool LidarUnifiedComponent::FindNearestFrame(
    const std::shared_ptr<SensorState>& sensor_state,
    const std::string& sensor_id, const TimeContract& reference_time,
    uint32_t max_ref_time_delta_ms, FrameHandle* frame_handle,
    FrameLookupFailureReason* failure_reason) const {
  if (frame_handle == nullptr || failure_reason == nullptr) {
    return false;
  }

  *failure_reason = FrameLookupFailureReason::kNone;
  if (sensor_state == nullptr) {
    *failure_reason = FrameLookupFailureReason::kBufferEmpty;
    return false;
  }

  std::lock_guard<std::mutex> lock(sensor_state->mutex);
  if (sensor_state->frames.empty()) {
    AINFO_EVERY(20) << "Lidar frame buffer is empty. sensor=" << sensor_id
                    << ", reference_anchor_ns="
                    << reference_time.canonical_anchor_ns;
    *failure_reason = FrameLookupFailureReason::kBufferEmpty;
    return false;
  }

  std::shared_ptr<const BufferedFrame> best_frame;
  bool has_unconsumed_frame = false;
  int64_t best_overlap_ns = -1;
  int64_t best_delta_ns = std::numeric_limits<int64_t>::max();
  int64_t best_anchor_ns = 0;
  const int64_t online_offset_ns =
      config_.enable_online_time_offset_update() &&
              sensor_state->fixed_delay_initialized
          ? static_cast<int64_t>(
                std::llround(sensor_state->fixed_delay_sec * 1e9))
          : 0;

  for (const auto& candidate : sensor_state->frames) {
    if (candidate == nullptr || candidate->point_cloud == nullptr ||
        sensor_state->consumed_frame_ids.count(candidate->frame_id) != 0U) {
      continue;
    }
    has_unconsumed_frame = true;

    TimeContract adjusted = candidate->time_contract;
    adjusted.scan_begin_ns += online_offset_ns;
    adjusted.scan_end_ns += online_offset_ns;
    adjusted.canonical_anchor_ns += online_offset_ns;
    const int64_t overlap_ns = IntervalOverlapNs(reference_time, adjusted);
    const int64_t delta_ns = std::llabs(
        adjusted.canonical_anchor_ns - reference_time.canonical_anchor_ns);
    if (delta_ns >
        static_cast<int64_t>(max_ref_time_delta_ms) * 1000000LL) {
      continue;
    }
    if (overlap_ns > best_overlap_ns ||
        (overlap_ns == best_overlap_ns && delta_ns < best_delta_ns) ||
        (overlap_ns == best_overlap_ns && delta_ns == best_delta_ns &&
         adjusted.canonical_anchor_ns < best_anchor_ns)) {
      best_frame = candidate;
      best_overlap_ns = overlap_ns;
      best_delta_ns = delta_ns;
      best_anchor_ns = adjusted.canonical_anchor_ns;
    }
  }

  if (best_frame == nullptr) {
    *failure_reason = has_unconsumed_frame
                          ? FrameLookupFailureReason::kTimeDeltaExceeded
                          : FrameLookupFailureReason::kBufferEmpty;
    return false;
  }

  frame_handle->sensor_id = sensor_id;
  frame_handle->point_cloud = best_frame->point_cloud;
  frame_handle->buffered_frame = best_frame;
  frame_handle->frame_id = best_frame->frame_id;
  frame_handle->time_contract = best_frame->time_contract;
  frame_handle->clock_offset_residual_ms =
      static_cast<double>(reference_time.canonical_anchor_ns -
                          best_frame->time_contract.canonical_anchor_ns -
                          online_offset_ns) /
      1e6;
  frame_handle->overlap_quality_weight = sensor_state->overlap_quality_weight;
  return true;
}

void LidarUnifiedComponent::CommitSelectedFrames(
    const std::vector<FrameHandle>& frame_handles) {
  for (const auto& handle : frame_handles) {
    if (handle.is_primary || handle.frame_id == 0U) {
      continue;
    }
    const auto sensor_state = GetSensorState(handle.sensor_id);
    if (sensor_state == nullptr) {
      continue;
    }
    std::lock_guard<std::mutex> lock(sensor_state->mutex);
    sensor_state->consumed_frame_ids.insert(handle.frame_id);
  }
}

bool LidarUnifiedComponent::ResolveMapToBase(double ref_timestamp_sec,
                                             Eigen::Affine3d* map2base_ref) {
  if (map2base_ref == nullptr || base_link_pose_resolver_ == nullptr) {
    return false;
  }

  if (!base_link_pose_resolver_->Prefetch(ref_timestamp_sec)) {
    total_tf_query_failures_.fetch_add(1);
  }

  Eigen::Affine3d map_from_base = Eigen::Affine3d::Identity();
  if (!base_link_pose_resolver_->QueryCached(ref_timestamp_sec,
                                             &map_from_base)) {
    total_tf_query_failures_.fetch_add(1);
    return false;
  }

  *map2base_ref = map_from_base.inverse();
  return true;
}

void LidarUnifiedComponent::UpdateSensorTimingModel(
    const FrameHandle& frame_handle, double ref_timestamp_sec,
    FrameMetrics* frame_metrics) {
  if (frame_handle.is_primary || frame_handle.point_cloud == nullptr) {
    return;
  }

  const auto sensor_state = GetSensorState(frame_handle.sensor_id);
  if (sensor_state == nullptr) {
    return;
  }

  const double observed_delay_sec =
      ref_timestamp_sec - frame_handle.time_contract.CanonicalAnchorSec();
  std::lock_guard<std::mutex> lock(sensor_state->mutex);
  if (config_.enable_online_time_offset_update()) {
    if (!sensor_state->fixed_delay_initialized) {
      sensor_state->fixed_delay_sec = observed_delay_sec;
      sensor_state->fixed_delay_initialized = true;
    } else if (std::fabs(
                   (observed_delay_sec - sensor_state->fixed_delay_sec) *
                   1000.0) <= config_.fixed_delay_update_limit_ms()) {
      const double alpha = config_.fixed_delay_ema_alpha();
      sensor_state->fixed_delay_sec =
          sensor_state->fixed_delay_sec * (1.0 - alpha) +
          observed_delay_sec * alpha;
    }
  }

  if (sensor_state->timing_observation_count == 0U) {
    sensor_state->smoothed_clock_offset_ms =
        frame_handle.clock_offset_residual_ms;
  } else {
    const double alpha = config_.clock_offset_ema_alpha();
    sensor_state->smoothed_clock_offset_ms =
        sensor_state->smoothed_clock_offset_ms * (1.0 - alpha) +
        frame_handle.clock_offset_residual_ms * alpha;
  }
  sensor_state->last_clock_offset_ms = frame_handle.clock_offset_residual_ms;
  ++sensor_state->timing_observation_count;

  if (frame_metrics != nullptr) {
    frame_metrics->max_abs_clock_offset_ms =
        std::max(frame_metrics->max_abs_clock_offset_ms,
                 std::fabs(frame_handle.clock_offset_residual_ms));
  }
}

void LidarUnifiedComponent::UpdateOverlapQualityWeights(
    const std::vector<FrameHandle>& frame_handles,
    const Eigen::Affine3d& map2base_ref, FrameMetrics* frame_metrics) {
  for (const auto& frame_handle : frame_handles) {
    if (frame_handle.is_primary || frame_handle.buffered_frame == nullptr) {
      continue;
    }

    const auto sensor_state = GetSensorState(frame_handle.sensor_id);
    if (sensor_state == nullptr) {
      continue;
    }

    const double observed_weight = EstimateOverlapQualityWeight(
        *frame_handle.buffered_frame, map2base_ref);
    std::lock_guard<std::mutex> lock(sensor_state->mutex);
    if (sensor_state->overlap_quality_samples == 0U) {
      sensor_state->overlap_quality_weight = observed_weight;
    } else {
      const double alpha = config_.overlap_quality_ema_alpha();
      sensor_state->overlap_quality_weight =
          sensor_state->overlap_quality_weight * (1.0 - alpha) +
          observed_weight * alpha;
    }
    ++sensor_state->overlap_quality_samples;

    if (frame_metrics != nullptr) {
      frame_metrics->min_overlap_quality_weight =
          std::min(frame_metrics->min_overlap_quality_weight,
                   sensor_state->overlap_quality_weight);
    }
  }
}

double LidarUnifiedComponent::EstimateOverlapQualityWeight(
    const BufferedFrame& buffered_frame,
    const Eigen::Affine3d& map2base_ref) const {
  if (buffered_frame.point_cloud == nullptr ||
      buffered_frame.motion_sample_times.empty() ||
      buffered_frame.motion_sample_times.size() !=
          buffered_frame.motion_poses.size()) {
    return 1.0;
  }

  const size_t stride = std::max<size_t>(
      1, static_cast<size_t>(config_.overlap_quality_sample_stride()));
  size_t sampled_points = 0;
  size_t overlap_points = 0;
  for (size_t index = 0;
       index < static_cast<size_t>(buffered_frame.point_cloud->point_size());
       index += stride) {
    PointXYZIT transformed_point;
    if (!TransformPointToBase(
            buffered_frame.point_cloud->point(static_cast<int>(index)),
            static_cast<double>(
                buffered_frame.time_contract.canonical_anchor_ns -
                buffered_frame.time_contract.static_offset_ns) /
                1e9,
            static_cast<double>(buffered_frame.time_contract.static_offset_ns) /
                1e9,
            buffered_frame.motion_sample_times, buffered_frame.motion_poses,
            map2base_ref, &transformed_point)) {
      continue;
    }
    ++sampled_points;
    if (IsPointInOverlapRegion(transformed_point)) {
      ++overlap_points;
    }
  }

  if (sampled_points == 0U) {
    return 1.0;
  }
  return static_cast<double>(overlap_points) /
         static_cast<double>(sampled_points);
}

bool LidarUnifiedComponent::IsPointInOverlapRegion(
    const ::apollo::drivers::PointXYZIT& point) const {
  return point.x() <= config_.overlap_region_forward_x() &&
         point.x() >= config_.overlap_region_backward_x() &&
         point.y() <= config_.overlap_region_left_y() &&
         point.y() >= config_.overlap_region_right_y() &&
         point.z() <= config_.overlap_region_max_z() &&
         point.z() >= config_.overlap_region_min_z();
}

bool LidarUnifiedComponent::BuildUnifiedPointCloud(
    const PointCloudConstPtr& main_frame,
    const std::vector<FrameHandle>& frame_handles, FrameMetrics* frame_metrics,
    std::shared_ptr<::apollo::drivers::PointCloud>* output) {
  if (main_frame == nullptr || frame_metrics == nullptr || output == nullptr ||
      frame_handles.empty() || deskew_policy_ == nullptr ||
      fusion_policy_ == nullptr || filter_policy_ == nullptr) {
    return false;
  }

  std::vector<FrameHandle> prepared_handles = frame_handles;
  if (config_.compensation_mode() == LidarUnifiedComponentConfig::OFF) {
    apollo::transform::TransformQuery transform_query(tf_buffer_);
    for (auto& handle : prepared_handles) {
      if (handle.buffered_frame == nullptr) {
        if (handle.is_primary) {
          return false;
        }
        handle.point_cloud.reset();
        continue;
      }
      Eigen::Affine3d base_from_sensor = Eigen::Affine3d::Identity();
      const auto cached = static_extrinsics_.find(handle.sensor_id);
      if (cached != static_extrinsics_.end()) {
        base_from_sensor = *cached->second;
      } else if (!transform_query.GetLatestStaticTransformToAffine(
                     config_.base_link_frame_id(), handle.sensor_id,
                     &base_from_sensor)) {
        if (handle.is_primary) {
          AERROR << "Static extrinsic unavailable from " << handle.sensor_id
                 << " to " << config_.base_link_frame_id();
          return false;
        }
        AWARN << "Skip auxiliary with unavailable static extrinsic: "
              << handle.sensor_id;
        handle.point_cloud.reset();
        continue;
      } else {
        static_extrinsics_.emplace(
            handle.sensor_id,
            std::make_shared<const Eigen::Affine3d>(base_from_sensor));
      }
      auto frame = std::make_shared<BufferedFrame>(*handle.buffered_frame);
      frame->motion_sample_times = {
          handle.time_contract.CanonicalAnchorSec()};
      frame->motion_poses = {base_from_sensor};
      frame->pose_prefetch_ok = true;
      handle.buffered_frame = std::move(frame);
    }
  }

  std::vector<SensorFrameContext> contexts;
  std::vector<std::vector<double>> motion_sample_times;
  std::vector<std::vector<Eigen::Affine3d>> motion_poses;
  contexts.reserve(prepared_handles.size());
  motion_sample_times.reserve(prepared_handles.size());
  motion_poses.reserve(prepared_handles.size());

  size_t required_points = 0;
  const uint64_t pose_bins_start_ns = cyber::Time::Now().ToNanosecond();
  if (!pose_bins_builder_.Build(prepared_handles, deskew_policy_.get(),
                                &contexts,
                                &motion_sample_times, &motion_poses,
                                &required_points)) {
    AERROR << "No valid sensor context for fusion";
    return false;
  }
  frame_metrics->pose_bins_ms = ElapsedMilliseconds(pose_bins_start_ns);

  frame_metrics->total_input_points = required_points;
  frame_metrics->matched_sensor_count = contexts.size();

  if (required_points > full_pointcloud_buffer_.size()) {
    AWARN << "Input points exceed preallocated buffer, required="
          << required_points << ", capacity=" << full_pointcloud_buffer_.size();
  }

  Eigen::Affine3d map2base_ref = Eigen::Affine3d::Identity();
  const double reference_time_sec =
      frame_handles.front().time_contract.CanonicalAnchorSec();
  const uint64_t reference_pose_start_ns = cyber::Time::Now().ToNanosecond();
  if (config_.compensation_mode() != LidarUnifiedComponentConfig::OFF &&
      !ResolveMapToBase(reference_time_sec, &map2base_ref)) {
    AERROR << "Failed to resolve cached base pose at ref timestamp "
           << reference_time_sec;
    return false;
  }
  frame_metrics->reference_pose_ms =
      ElapsedMilliseconds(reference_pose_start_ns);
  if (config_.enable_overlap_quality_gate()) {
    UpdateOverlapQualityWeights(prepared_handles, map2base_ref, frame_metrics);
  }

  PointCloudBuffer fused_buffer;
  fused_buffer.data_ptr = full_pointcloud_buffer_.data();
  fused_buffer.capacity = full_pointcloud_buffer_.size();
  fused_buffer.valid_count = 0;
  fused_buffer.item_size = sizeof(apollo::drivers::PointXYZIT);
  fused_buffer.device_type = MemoryDeviceType::kHost;
  fused_buffer.device_id =
      config_.compute_mode() == LidarUnifiedComponentConfig::COMPUTE_MODE_GPU
          ? static_cast<int>(config_.gpu_device_id())
          : -1;

  const uint64_t fusion_start_ns = cyber::Time::Now().ToNanosecond();
  if (!fusion_policy_->FuseToBaseLink(
          reference_time_sec, map2base_ref, contexts, motion_poses,
          motion_sample_times, &fused_buffer)) {
    AERROR << "Failed to fuse point clouds for ref timestamp "
           << reference_time_sec;
    return false;
  }
  frame_metrics->fusion_ms = ElapsedMilliseconds(fusion_start_ns);

  const size_t compact_points = fused_buffer.valid_count;
  size_t ego_filtered_points = 0;
  size_t voxel_filtered_points = 0;
  const uint64_t filter_start_ns = cyber::Time::Now().ToNanosecond();
  const size_t valid_size = filter_policy_->ApplyFilters(
      &fused_buffer, &ego_filtered_points, &voxel_filtered_points);
  frame_metrics->filter_ms = ElapsedMilliseconds(filter_start_ns);
  frame_metrics->compact_points = compact_points;
  frame_metrics->ego_filtered_points = ego_filtered_points;
  frame_metrics->voxel_filtered_points = voxel_filtered_points;
  frame_metrics->output_points = valid_size;

  const uint64_t output_build_start_ns = cyber::Time::Now().ToNanosecond();
  auto unified = std::make_shared<::apollo::drivers::PointCloud>();
  unified->mutable_header()->CopyFrom(main_frame->header());
  unified->mutable_header()->set_frame_id(config_.base_link_frame_id());
  unified->mutable_header()->set_module_name("lidar_unified_processor");
  unified->mutable_header()->set_timestamp_sec(cyber::Time::Now().ToSecond());
  unified->mutable_header()->set_sequence_num(sequence_num_.fetch_add(1) + 1);
  unified->set_frame_id(config_.base_link_frame_id());
  unified->set_measurement_time(reference_time_sec);
  unified->set_is_dense(true);
  unified->mutable_point()->Reserve(static_cast<int>(valid_size));

  for (size_t i = 0; i < valid_size; ++i) {
    const auto& point = full_pointcloud_buffer_[i];
    auto* out_pt = unified->add_point();
    out_pt->set_x(point.x());
    out_pt->set_y(point.y());
    out_pt->set_z(point.z());
    out_pt->set_intensity(point.intensity());
    out_pt->set_timestamp(point.timestamp());
  }

  unified->set_height(1);
  unified->set_width(unified->point_size());
  frame_metrics->output_build_ms =
      ElapsedMilliseconds(output_build_start_ns);

  *output = unified;
  return true;
}
}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
