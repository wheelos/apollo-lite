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

#include "modules/transform/internal/buffer_impl.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "absl/strings/str_cat.h"
#include "tf2/exceptions.h"

#include "cyber/cyber.h"
#include "cyber/time/clock.h"

using Clock = ::apollo::cyber::Clock;

namespace {
constexpr double kSecondToNanoFactor = 1e9;

template <typename Query>
bool WaitForTransform(Query query, float timeout_sec, std::string* error) {
  if (!std::isfinite(timeout_sec) || timeout_sec < 0.0f ||
      timeout_sec > std::chrono::duration<double>(
                        std::chrono::steady_clock::duration::max() / 2)
                        .count()) {
    *error = "invalid transform timeout";
    AERROR << *error << ": " << timeout_sec;
    return false;
  }
  using SteadyClock = std::chrono::steady_clock;
  const auto deadline =
      SteadyClock::now() + std::chrono::duration_cast<SteadyClock::duration>(
                               std::chrono::duration<double>(timeout_sec));
  do {
    error->clear();
    if (query()) {
      return true;
    }
    const auto now = SteadyClock::now();
    if (apollo::cyber::IsShutdown() || now >= deadline) {
      break;
    }
    std::this_thread::sleep_until(
        std::min(deadline, now + std::chrono::milliseconds(1)));
  } while (true);
  *error += ":timeout";
  return false;
}

bool SameStaticTransform(const geometry_msgs::TransformStamped& first,
                         const geometry_msgs::TransformStamped& second) {
  const auto& a = first.transform;
  const auto& b = second.transform;
  const double dot = a.rotation.x * b.rotation.x + a.rotation.y * b.rotation.y +
                     a.rotation.z * b.rotation.z + a.rotation.w * b.rotation.w;
  return first.header.frame_id == second.header.frame_id &&
         std::abs(a.translation.x - b.translation.x) <= 1e-12 &&
         std::abs(a.translation.y - b.translation.y) <= 1e-12 &&
         std::abs(a.translation.z - b.translation.z) <= 1e-12 &&
         std::abs(a.rotation.x - (dot < 0.0 ? -b.rotation.x : b.rotation.x)) <=
             1e-12 &&
         std::abs(a.rotation.y - (dot < 0.0 ? -b.rotation.y : b.rotation.y)) <=
             1e-12 &&
         std::abs(a.rotation.z - (dot < 0.0 ? -b.rotation.z : b.rotation.z)) <=
             1e-12 &&
         std::abs(a.rotation.w - (dot < 0.0 ? -b.rotation.w : b.rotation.w)) <=
             1e-12;
}

void ThrowLookupTimeout(const std::string& target_frame,
                        const std::string& source_frame,
                        const std::string& error) {
  if (!error.empty()) {
    throw tf2::LookupException(error);
  }

  throw tf2::LookupException(absl::StrCat("Lookup transform timeout from ",
                                          source_frame, " to ", target_frame));
}
}  // namespace

namespace apollo {
namespace transform {

std::string Buffer::Impl::BuildEdgeKey(const std::string& frame_id,
                                       const std::string& child_frame_id) {
  return frame_id + "\n" + child_frame_id;
}

int Buffer::Impl::Init() {
  return listener_.Init(
      [this](const std::shared_ptr<const TransformStampeds>& message,
             bool is_static) { Receive(message, is_static); });
}

void Buffer::Impl::ClearDynamicHistoryLocked() {
  buffer_.clear();
  // Preserve calibration explicitly instead of relying on tf2 clear semantics.
  for (const auto& child : children_) {
    if (child.second.is_static) {
      CHECK(buffer_.setTransform(child.second.transform, child.second.authority,
                                 true));
    }
  }
}

void Buffer::Impl::CheckTimeJumpLocked() {
  const auto now = Clock::Now();
  if (now < last_clock_) {
    AINFO << "Detected jump back in time. Clearing dynamic TF history.";
    ClearDynamicHistoryLocked();
    time_jump_count_.fetch_add(1, std::memory_order_relaxed);
  }
  last_clock_ = now;
}

uint64_t Buffer::Impl::GetTimeEpoch() {
  std::lock_guard<std::mutex> lock(diagnostics_mutex_);
  CheckTimeJumpLocked();
  return time_jump_count_.load(std::memory_order_relaxed);
}

void Buffer::Impl::clear() {
  std::lock_guard<std::mutex> lock(diagnostics_mutex_);
  ClearDynamicHistoryLocked();
  time_jump_count_.fetch_add(1, std::memory_order_relaxed);
  last_clock_ = Clock::Now();
}

void Buffer::Impl::Receive(
    const std::shared_ptr<const TransformStampeds>& msg_evt, bool is_static) {
  std::lock_guard<std::mutex> lock(diagnostics_mutex_);
  CheckTimeJumpLocked();
  last_update_ = Clock::Now();
  if (!msg_evt || msg_evt->transforms().empty()) {
    AERROR << "Received empty transform message.";
    ++rejected_message_count_;
    return;
  }
  const std::string authority = msg_evt->header().module_name();
  if (is_static) {
    static_message_count_.fetch_add(1, std::memory_order_relaxed);
  } else {
    dynamic_message_count_.fetch_add(1, std::memory_order_relaxed);
  }

  std::vector<geometry_msgs::TransformStamped> pending;
  std::unordered_map<std::string, std::string> parents;
  for (int i = 0; i < msg_evt->transforms_size(); ++i) {
    geometry_msgs::TransformStamped trans_stamped;

    // header
    const auto& header = msg_evt->transforms(i).header();
    if (!std::isfinite(header.timestamp_sec()) ||
        header.timestamp_sec() < 0.0 ||
        header.timestamp_sec() >=
            static_cast<double>(std::numeric_limits<uint64_t>::max()) /
                kSecondToNanoFactor) {
      AERROR << "Invalid transform timestamp.";
      ++rejected_message_count_;
      return;
    }
    trans_stamped.header.stamp =
        static_cast<uint64_t>(header.timestamp_sec() * kSecondToNanoFactor);
    trans_stamped.header.frame_id = header.frame_id();
    trans_stamped.header.seq = header.sequence_num();

    // child_frame_id
    trans_stamped.child_frame_id = msg_evt->transforms(i).child_frame_id();

    // translation
    const auto& transform = msg_evt->transforms(i).transform();
    trans_stamped.transform.translation.x = transform.translation().x();
    trans_stamped.transform.translation.y = transform.translation().y();
    trans_stamped.transform.translation.z = transform.translation().z();

    // rotation
    trans_stamped.transform.rotation.x = transform.rotation().qx();
    trans_stamped.transform.rotation.y = transform.rotation().qy();
    trans_stamped.transform.rotation.z = transform.rotation().qz();
    trans_stamped.transform.rotation.w = transform.rotation().qw();

    const auto& parent = trans_stamped.header.frame_id;
    const auto& child = trans_stamped.child_frame_id;
    const auto& translation = trans_stamped.transform.translation;
    auto& rotation = trans_stamped.transform.rotation;
    const double norm = rotation.x * rotation.x + rotation.y * rotation.y +
                        rotation.z * rotation.z + rotation.w * rotation.w;
    if (parent.empty() || child.empty() || parent.front() == '/' ||
        child.front() == '/' || parent == child ||
        !parents.emplace(child, parent).second ||
        !std::isfinite(translation.x) || !std::isfinite(translation.y) ||
        !std::isfinite(translation.z) || !std::isfinite(norm) ||
        std::abs(norm - 1.0) > 1e-3) {
      AERROR << "Invalid transform edge: " << parent << " -> " << child;
      ++rejected_message_count_;
      return;
    }
    const double scale = 1.0 / std::sqrt(norm);
    rotation.x *= scale;
    rotation.y *= scale;
    rotation.z *= scale;
    rotation.w *= scale;
    const auto existing = children_.find(child);
    if (existing != children_.end() &&
        (existing->second.is_static != is_static ||
         existing->second.transform.header.frame_id != parent ||
         (is_static &&
          (existing->second.authority != authority ||
           !SameStaticTransform(existing->second.transform, trans_stamped))))) {
      AERROR << "Conflicting transform authority, type or value for child: "
             << child;
      ++rejected_message_count_;
      return;
    }
    pending.push_back(trans_stamped);
  }
  for (const auto& edge : parents) {
    std::unordered_set<std::string> visited;
    std::string frame = edge.first;
    while (true) {
      if (!visited.insert(frame).second) {
        AERROR << "Received cyclic transform graph at: " << frame;
        ++rejected_message_count_;
        return;
      }
      const auto incoming = parents.find(frame);
      const auto existing = children_.find(frame);
      if (incoming != parents.end()) {
        frame = incoming->second;
      } else if (existing != children_.end()) {
        frame = existing->second.transform.header.frame_id;
      } else {
        break;
      }
    }
  }
  for (const auto& trans_stamped : pending) {
    if (buffer_.setTransform(trans_stamped, authority, is_static)) {
      children_[trans_stamped.child_frame_id] =
          ChildState{trans_stamped, is_static, authority};
      if (is_static) {
        CHECK(static_buffer_.setTransform(trans_stamped, authority, true));
        static_transform_count_.fetch_add(1, std::memory_order_relaxed);
      } else {
        dynamic_transform_count_.fetch_add(1, std::memory_order_relaxed);
      }
      auto& edge = edge_diagnostics_[BuildEdgeKey(
          trans_stamped.header.frame_id, trans_stamped.child_frame_id)];
      edge.is_static = is_static;
      edge.latest_timestamp_sec =
          static_cast<double>(trans_stamped.header.stamp) / 1e9;
      ++edge.received_count;
    } else {
      AERROR << "TF rejected received transform for child: "
             << trans_stamped.child_frame_id;
      ++rejected_message_count_;
    }
  }
}

bool Buffer::Impl::GetLatestStaticTransform(const std::string& frame_id,
                                            const std::string& child_frame_id,
                                            TransformStamped* tf) const {
  if (tf == nullptr) {
    AERROR << "Null static transform output.";
    return false;
  }
  std::lock_guard<std::mutex> lock(diagnostics_mutex_);
  try {
    const auto transform =
        static_buffer_.lookupTransform(frame_id, child_frame_id, tf2::Time(0));
    TF2MsgToCyber(transform, *tf);
    return true;
  } catch (const tf2::TransformException&) {
    // The requested static edge or chain is not available.
  }
  return false;
}

TransformBufferDiagnostics Buffer::Impl::GetDiagnosticsSnapshot() const {
  TransformBufferDiagnostics diagnostics;
  diagnostics.dynamic_message_count =
      dynamic_message_count_.load(std::memory_order_relaxed);
  diagnostics.static_message_count =
      static_message_count_.load(std::memory_order_relaxed);
  diagnostics.dynamic_transform_count =
      dynamic_transform_count_.load(std::memory_order_relaxed);
  diagnostics.static_transform_count =
      static_transform_count_.load(std::memory_order_relaxed);
  diagnostics.time_jump_count =
      time_jump_count_.load(std::memory_order_relaxed);
  {
    std::lock_guard<std::mutex> lock(diagnostics_mutex_);
    diagnostics.last_update_sec = last_update_.ToSecond();
    diagnostics.rejected_message_count = rejected_message_count_;
    diagnostics.edges.reserve(edge_diagnostics_.size());
    for (const auto& entry : edge_diagnostics_) {
      const size_t separator = entry.first.find('\n');
      TransformEdgeDiagnostics edge;
      edge.frame_id = entry.first.substr(0, separator);
      if (separator != std::string::npos) {
        edge.child_frame_id = entry.first.substr(separator + 1);
      }
      edge.is_static = entry.second.is_static;
      edge.latest_timestamp_sec = entry.second.latest_timestamp_sec;
      edge.received_count = entry.second.received_count;
      diagnostics.edges.push_back(edge);
    }
    diagnostics.frames_as_string = buffer_.allFramesAsString();
    diagnostics.frames_as_yaml = buffer_.allFramesAsYAML(Clock::Now().ToSecond());
  }
  return diagnostics;
}

void Buffer::Impl::TF2MsgToCyber(
    const geometry_msgs::TransformStamped& tf2_trans_stamped,
    TransformStamped& trans_stamped) {
  // header
  trans_stamped.mutable_header()->set_timestamp_sec(
      static_cast<double>(tf2_trans_stamped.header.stamp) / 1e9);
  trans_stamped.mutable_header()->set_frame_id(
      tf2_trans_stamped.header.frame_id);

  // child_frame_id
  trans_stamped.set_child_frame_id(tf2_trans_stamped.child_frame_id);

  // translation
  trans_stamped.mutable_transform()->mutable_translation()->set_x(
      tf2_trans_stamped.transform.translation.x);
  trans_stamped.mutable_transform()->mutable_translation()->set_y(
      tf2_trans_stamped.transform.translation.y);
  trans_stamped.mutable_transform()->mutable_translation()->set_z(
      tf2_trans_stamped.transform.translation.z);

  // rotation
  trans_stamped.mutable_transform()->mutable_rotation()->set_qx(
      tf2_trans_stamped.transform.rotation.x);
  trans_stamped.mutable_transform()->mutable_rotation()->set_qy(
      tf2_trans_stamped.transform.rotation.y);
  trans_stamped.mutable_transform()->mutable_rotation()->set_qz(
      tf2_trans_stamped.transform.rotation.z);
  trans_stamped.mutable_transform()->mutable_rotation()->set_qw(
      tf2_trans_stamped.transform.rotation.w);
}

TransformStamped Buffer::Impl::lookupTransform(
    const std::string& target_frame, const std::string& source_frame,
    const cyber::Time& time, const float timeout_second) const {
  std::string errstr;
  if (!canTransform(target_frame, source_frame, time, timeout_second,
                    &errstr)) {
    ThrowLookupTimeout(target_frame, source_frame, errstr);
  }

  std::lock_guard<std::mutex> lock(diagnostics_mutex_);
  tf2::Time tf2_time(time.ToNanosecond());
  geometry_msgs::TransformStamped tf2_trans_stamped =
      buffer_.lookupTransform(target_frame, source_frame, tf2_time);
  TransformStamped trans_stamped;
  TF2MsgToCyber(tf2_trans_stamped, trans_stamped);
  return trans_stamped;
}

TransformStamped Buffer::Impl::lookupTransform(
    const std::string& target_frame, const cyber::Time& target_time,
    const std::string& source_frame, const cyber::Time& source_time,
    const std::string& fixed_frame, const float timeout_second) const {
  std::string errstr;
  if (!canTransform(target_frame, target_time, source_frame, source_time,
                    fixed_frame, timeout_second, &errstr)) {
    ThrowLookupTimeout(target_frame, source_frame, errstr);
  }

  std::lock_guard<std::mutex> lock(diagnostics_mutex_);
  geometry_msgs::TransformStamped tf2_trans_stamped = buffer_.lookupTransform(
      target_frame, target_time.ToNanosecond(), source_frame,
      source_time.ToNanosecond(), fixed_frame);
  TransformStamped trans_stamped;
  TF2MsgToCyber(tf2_trans_stamped, trans_stamped);
  return trans_stamped;
}

bool Buffer::Impl::canTransform(const std::string& target_frame,
                                const std::string& source_frame,
                                const cyber::Time& time,
                                const float timeout_second,
                                std::string* errstr) const {
  std::string local_errstr;
  std::string* error = errstr != nullptr ? errstr : &local_errstr;
  return WaitForTransform(
      [&]() {
        std::lock_guard<std::mutex> lock(diagnostics_mutex_);
        return buffer_.canTransform(target_frame, source_frame,
                                    time.ToNanosecond(), error);
      },
      timeout_second, error);
}

bool Buffer::Impl::canTransform(const std::string& target_frame,
                                const cyber::Time& target_time,
                                const std::string& source_frame,
                                const cyber::Time& source_time,
                                const std::string& fixed_frame,
                                const float timeout_second,
                                std::string* errstr) const {
  std::string local_errstr;
  std::string* error = errstr != nullptr ? errstr : &local_errstr;
  return WaitForTransform(
      [&]() {
        std::lock_guard<std::mutex> lock(diagnostics_mutex_);
        return buffer_.canTransform(target_frame, target_time.ToNanosecond(),
                                    source_frame, source_time.ToNanosecond(),
                                    fixed_frame, error);
      },
      timeout_second, error);
}

}  // namespace transform
}  // namespace apollo
