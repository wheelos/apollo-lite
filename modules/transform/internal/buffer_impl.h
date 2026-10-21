// Copyright 2026 WheelOS. All Rights Reserved.
#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "tf2/buffer_core.h"

#include "modules/transform/buffer.h"
#include "modules/transform/internal/transform_listener.h"

namespace apollo {
namespace transform {

class Buffer::Impl {
 public:
  int Init();
  void Receive(const std::shared_ptr<const TransformStampeds>& message,
               bool is_static);

  TransformStamped lookupTransform(const std::string& target_frame,
                                   const std::string& source_frame,
                                   const cyber::Time& time,
                                   float timeout) const;
  TransformStamped lookupTransform(const std::string& target_frame,
                                   const cyber::Time& target_time,
                                   const std::string& source_frame,
                                   const cyber::Time& source_time,
                                   const std::string& fixed_frame,
                                   float timeout) const;
  bool canTransform(const std::string& target_frame,
                    const std::string& source_frame, const cyber::Time& time,
                    float timeout, std::string* error) const;
  bool canTransform(const std::string& target_frame,
                    const cyber::Time& target_time,
                    const std::string& source_frame,
                    const cyber::Time& source_time,
                    const std::string& fixed_frame, float timeout,
                    std::string* error) const;
  bool GetLatestStaticTransform(const std::string& target_frame,
                                const std::string& source_frame,
                                TransformStamped* transform) const;
  TransformBufferDiagnostics GetDiagnosticsSnapshot() const;
  uint64_t GetTimeEpoch();
  void clear();

 private:
  friend class BufferTestPeer;
  struct ChildState {
    geometry_msgs::TransformStamped transform;
    bool is_static = false;
    std::string authority;
  };
  struct EdgeDiagnosticsState {
    bool is_static = false;
    double latest_timestamp_sec = 0.0;
    uint64_t received_count = 0;
  };

  static std::string BuildEdgeKey(const std::string& parent,
                                  const std::string& child);
  static void TF2MsgToCyber(const geometry_msgs::TransformStamped& input,
                            TransformStamped& output);  // NOLINT
  void ClearDynamicHistoryLocked();
  void CheckTimeJumpLocked();

  tf2::BufferCore buffer_;
  tf2::BufferCore static_buffer_;
  cyber::Time last_update_;
  cyber::Time last_clock_;
  std::unordered_map<std::string, ChildState> children_;
  mutable std::mutex diagnostics_mutex_;
  std::atomic<uint64_t> dynamic_message_count_{0};
  std::atomic<uint64_t> static_message_count_{0};
  std::atomic<uint64_t> dynamic_transform_count_{0};
  std::atomic<uint64_t> static_transform_count_{0};
  std::atomic<uint64_t> time_jump_count_{0};
  uint64_t rejected_message_count_ = 0;
  std::unordered_map<std::string, EdgeDiagnosticsState> edge_diagnostics_;
  // Destroy subscriptions before the state their callbacks access.
  internal::TransformListener listener_;
};

}  // namespace transform
}  // namespace apollo
