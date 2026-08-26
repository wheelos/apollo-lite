// Copyright 2025 WheelOS All Rights Reserved.
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

//  Created Date: 2026-04-10
//  Author: daohu527

#include "modules/transform/transform_query.h"

#include <string>

#include "tf2/exceptions.h"

#include "cyber/common/log.h"
#include "modules/transform/buffer.h"
#include "modules/transform/buffer_interface.h"

namespace apollo {
namespace transform {

TransformQuery::TransformQuery() : buffer_(Buffer::Instance()) {}

TransformQuery::TransformQuery(BufferInterface* buffer)
  : buffer_(buffer) {}

void TransformQuery::RecordCanTransform(bool success,
                                        const std::string& error) const {
  can_transform_calls_.fetch_add(1, std::memory_order_relaxed);
  if (success) {
    can_transform_success_.fetch_add(1, std::memory_order_relaxed);
  } else {
    std::lock_guard<std::mutex> lock(error_mutex_);
    last_error_ = error;
  }
}

void TransformQuery::RecordLookupTransform(bool success,
                                           const std::string& error) const {
  lookup_transform_calls_.fetch_add(1, std::memory_order_relaxed);
  if (success) {
    lookup_transform_success_.fetch_add(1, std::memory_order_relaxed);
  } else {
    std::lock_guard<std::mutex> lock(error_mutex_);
    last_error_ = error;
  }
}

void TransformQuery::RecordAffineLookup(bool success,
                                        const std::string& error) const {
  affine_lookup_calls_.fetch_add(1, std::memory_order_relaxed);
  if (success) {
    affine_lookup_success_.fetch_add(1, std::memory_order_relaxed);
  } else {
    std::lock_guard<std::mutex> lock(error_mutex_);
    last_error_ = error;
  }
}

void TransformQuery::RecordStaticLookup(bool success,
                                        const std::string& error) const {
  static_lookup_calls_.fetch_add(1, std::memory_order_relaxed);
  if (success) {
    static_lookup_success_.fetch_add(1, std::memory_order_relaxed);
  } else {
    std::lock_guard<std::mutex> lock(error_mutex_);
    last_error_ = error;
  }
}

bool TransformQuery::CanTransform(const std::string& target_frame_id,
                                  const std::string& source_frame_id,
                                  const cyber::Time& query_time,
                                  float timeout_sec,
                                  std::string* err_msg) const {
  CHECK_NOTNULL(buffer_);
  std::string local_error;
  std::string* error = err_msg != nullptr ? err_msg : &local_error;
  const bool success = buffer_->canTransform(
      target_frame_id, source_frame_id, query_time, timeout_sec, error);
  RecordCanTransform(success, *error);
  return success;
}

bool TransformQuery::CanTransform(const std::string& target_frame_id,
                                  const cyber::Time& target_time,
                                  const std::string& source_frame_id,
                                  const cyber::Time& source_time,
                                  const std::string& fixed_frame_id,
                                  float timeout_sec,
                                  std::string* err_msg) const {
  CHECK_NOTNULL(buffer_);
  std::string local_error;
  std::string* error = err_msg != nullptr ? err_msg : &local_error;
  const bool success = buffer_->canTransform(
      target_frame_id, target_time, source_frame_id, source_time,
      fixed_frame_id, timeout_sec, error);
  RecordCanTransform(success, *error);
  return success;
}

bool TransformQuery::LookupTransform(const std::string& target_frame_id,
                                     const std::string& source_frame_id,
                                     const cyber::Time& query_time,
                                     TransformStamped* transform,
                                     float timeout_sec,
                                     std::string* err_msg) const {
  CHECK_NOTNULL(transform);
  CHECK_NOTNULL(buffer_);

  try {
    *transform = buffer_->lookupTransform(target_frame_id, source_frame_id,
                                          query_time, timeout_sec);
  } catch (const tf2::TransformException& ex) {
    if (err_msg != nullptr) {
      *err_msg = ex.what();
    }
    RecordLookupTransform(false, ex.what());
    return false;
  }

  RecordLookupTransform(true, "");
  return true;
}

bool TransformQuery::LookupTransform(
    const std::string& target_frame_id, const cyber::Time& target_time,
    const std::string& source_frame_id, const cyber::Time& source_time,
    const std::string& fixed_frame_id, TransformStamped* transform,
    float timeout_sec, std::string* err_msg) const {
  CHECK_NOTNULL(transform);
  CHECK_NOTNULL(buffer_);

  try {
    *transform =
        buffer_->lookupTransform(target_frame_id, target_time, source_frame_id,
                                 source_time, fixed_frame_id, timeout_sec);
  } catch (const tf2::TransformException& ex) {
    if (err_msg != nullptr) {
      *err_msg = ex.what();
    }
    RecordLookupTransform(false, ex.what());
    return false;
  }

  RecordLookupTransform(true, "");
  return true;
}

bool TransformQuery::LookupTransformToAffine(const std::string& target_frame_id,
                                             const std::string& source_frame_id,
                                             const cyber::Time& query_time,
                                             Eigen::Affine3d* transform,
                                             float timeout_sec,
                                             std::string* err_msg) const {
  CHECK_NOTNULL(transform);

  TransformStamped stamped_transform;
  if (!LookupTransform(target_frame_id, source_frame_id, query_time,
                       &stamped_transform, timeout_sec, err_msg)) {
    RecordAffineLookup(false, err_msg != nullptr ? *err_msg : "");
    return false;
  }

  *transform = ToAffine(stamped_transform);
  RecordAffineLookup(true, "");
  return true;
}

bool TransformQuery::LookupTransformToAffine(
    const std::string& target_frame_id, const cyber::Time& target_time,
    const std::string& source_frame_id, const cyber::Time& source_time,
    const std::string& fixed_frame_id, Eigen::Affine3d* transform,
    float timeout_sec, std::string* err_msg) const {
  CHECK_NOTNULL(transform);

  TransformStamped stamped_transform;
  if (!LookupTransform(target_frame_id, target_time, source_frame_id,
                       source_time, fixed_frame_id, &stamped_transform,
                       timeout_sec, err_msg)) {
    RecordAffineLookup(false, err_msg != nullptr ? *err_msg : "");
    return false;
  }

  *transform = ToAffine(stamped_transform);
  RecordAffineLookup(true, "");
  return true;
}

bool TransformQuery::GetLatestStaticTransform(
    const std::string& target_frame_id, const std::string& source_frame_id,
    TransformStamped* transform) const {
  CHECK_NOTNULL(transform);
  CHECK_NOTNULL(buffer_);
  const bool success = buffer_->GetLatestStaticTransform(
      target_frame_id, source_frame_id, transform);
  RecordStaticLookup(success, success ? "" : "static transform not found");
  return success;
}

bool TransformQuery::GetLatestStaticTransformToAffine(
    const std::string& target_frame_id, const std::string& source_frame_id,
    Eigen::Affine3d* transform) const {
  CHECK_NOTNULL(transform);

  TransformStamped stamped_transform;
  if (!GetLatestStaticTransform(target_frame_id, source_frame_id,
                                &stamped_transform)) {
    RecordAffineLookup(false, "static transform not found");
    return false;
  }

  *transform = ToAffine(stamped_transform);
  RecordAffineLookup(true, "");
  return true;
}

TransformQueryDiagnostics TransformQuery::GetDiagnosticsSnapshot() const {
  TransformQueryDiagnostics diag;
  diag.can_transform_calls =
      can_transform_calls_.load(std::memory_order_relaxed);
  diag.can_transform_success =
      can_transform_success_.load(std::memory_order_relaxed);
  diag.lookup_transform_calls =
      lookup_transform_calls_.load(std::memory_order_relaxed);
  diag.lookup_transform_success =
      lookup_transform_success_.load(std::memory_order_relaxed);
  diag.affine_lookup_calls =
      affine_lookup_calls_.load(std::memory_order_relaxed);
  diag.affine_lookup_success =
      affine_lookup_success_.load(std::memory_order_relaxed);
  diag.static_lookup_calls =
      static_lookup_calls_.load(std::memory_order_relaxed);
  diag.static_lookup_success =
      static_lookup_success_.load(std::memory_order_relaxed);
  {
    std::lock_guard<std::mutex> lock(error_mutex_);
    diag.last_error = last_error_;
  }
  return diag;
}

Eigen::Affine3d TransformQuery::ToAffine(const TransformStamped& transform) {
  const auto& translation = transform.transform().translation();
  const auto& rotation = transform.transform().rotation();
  return Eigen::Translation3d(translation.x(), translation.y(),
                              translation.z()) *
         Eigen::Quaterniond(rotation.qw(), rotation.qx(), rotation.qy(),
                            rotation.qz());
}

}  // namespace transform
}  // namespace apollo
