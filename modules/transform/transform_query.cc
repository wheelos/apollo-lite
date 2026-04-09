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

#include "modules/transform/transform_query.h"

#include "cyber/common/log.h"
#include "modules/transform/buffer.h"

namespace apollo {
namespace transform {

TransformQuery::TransformQuery() : buffer_(Buffer::Instance()) {}

TransformQuery::TransformQuery(Buffer* buffer)
    : buffer_(buffer != nullptr ? buffer : Buffer::Instance()) {}

bool TransformQuery::CanTransform(const std::string& target_frame_id,
                                  const std::string& source_frame_id,
                                  const cyber::Time& query_time,
                                  float timeout_sec,
                                  std::string* err_msg) const {
  CHECK_NOTNULL(buffer_);
  return buffer_->canTransform(target_frame_id, source_frame_id, query_time,
                               timeout_sec, err_msg);
}

bool TransformQuery::LookupTransform(const std::string& target_frame_id,
                                     const std::string& source_frame_id,
                                     const cyber::Time& query_time,
                                     TransformStamped* transform,
                                     float timeout_sec,
                                     std::string* err_msg) const {
  CHECK_NOTNULL(transform);

  if (!CanTransform(target_frame_id, source_frame_id, query_time, timeout_sec,
                    err_msg)) {
    return false;
  }

  try {
    *transform = buffer_->lookupTransform(target_frame_id, source_frame_id,
                                          query_time, timeout_sec);
  } catch (const tf2::TransformException& ex) {
    if (err_msg != nullptr) {
      *err_msg = ex.what();
    }
    return false;
  }

  return true;
}

bool TransformQuery::LookupTransformToAffine(
    const std::string& target_frame_id, const std::string& source_frame_id,
    const cyber::Time& query_time, Eigen::Affine3d* transform,
    float timeout_sec, std::string* err_msg) const {
  CHECK_NOTNULL(transform);

  TransformStamped stamped_transform;
  if (!LookupTransform(target_frame_id, source_frame_id, query_time,
                       &stamped_transform, timeout_sec, err_msg)) {
    return false;
  }

  *transform = ToAffine(stamped_transform);
  return true;
}

bool TransformQuery::GetLatestStaticTransform(
    const std::string& target_frame_id, const std::string& source_frame_id,
    TransformStamped* transform) const {
  CHECK_NOTNULL(transform);
  CHECK_NOTNULL(buffer_);
  return buffer_->GetLatestStaticTF(target_frame_id, source_frame_id,
                                    transform);
}

bool TransformQuery::GetLatestStaticTransformToAffine(
    const std::string& target_frame_id, const std::string& source_frame_id,
    Eigen::Affine3d* transform) const {
  CHECK_NOTNULL(transform);

  TransformStamped stamped_transform;
  if (!GetLatestStaticTransform(target_frame_id, source_frame_id,
                                &stamped_transform)) {
    return false;
  }

  *transform = ToAffine(stamped_transform);
  return true;
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