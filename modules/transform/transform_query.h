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

#pragma once

#include <string>

#include "Eigen/Geometry"

#include "cyber/time/time.h"
#include "modules/common_msgs/transform_msgs/transform.pb.h"

namespace apollo {
namespace transform {

class Buffer;

class TransformQuery {
 public:
  TransformQuery();
  explicit TransformQuery(Buffer* buffer);

  bool CanTransform(const std::string& target_frame_id,
                    const std::string& source_frame_id,
                    const cyber::Time& query_time,
                    float timeout_sec = 0.01f,
                    std::string* err_msg = nullptr) const;

  bool CanTransform(const std::string& target_frame_id,
                    const cyber::Time& target_time,
                    const std::string& source_frame_id,
                    const cyber::Time& source_time,
                    const std::string& fixed_frame_id,
                    float timeout_sec = 0.01f,
                    std::string* err_msg = nullptr) const;

  bool LookupTransform(const std::string& target_frame_id,
                       const std::string& source_frame_id,
                       const cyber::Time& query_time,
                       TransformStamped* transform,
                       float timeout_sec = 0.01f,
                       std::string* err_msg = nullptr) const;

  bool LookupTransform(const std::string& target_frame_id,
                       const cyber::Time& target_time,
                       const std::string& source_frame_id,
                       const cyber::Time& source_time,
                       const std::string& fixed_frame_id,
                       TransformStamped* transform,
                       float timeout_sec = 0.01f,
                       std::string* err_msg = nullptr) const;

  bool LookupTransformToAffine(const std::string& target_frame_id,
                               const std::string& source_frame_id,
                               const cyber::Time& query_time,
                               Eigen::Affine3d* transform,
                               float timeout_sec = 0.01f,
                               std::string* err_msg = nullptr) const;

  bool LookupTransformToAffine(const std::string& target_frame_id,
                               const cyber::Time& target_time,
                               const std::string& source_frame_id,
                               const cyber::Time& source_time,
                               const std::string& fixed_frame_id,
                               Eigen::Affine3d* transform,
                               float timeout_sec = 0.01f,
                               std::string* err_msg = nullptr) const;

  bool GetLatestStaticTransform(const std::string& target_frame_id,
                                const std::string& source_frame_id,
                                TransformStamped* transform) const;

  bool GetLatestStaticTransformToAffine(const std::string& target_frame_id,
                                        const std::string& source_frame_id,
                                        Eigen::Affine3d* transform) const;

 private:
  static Eigen::Affine3d ToAffine(const TransformStamped& transform);

  Buffer* buffer_ = nullptr;
};

}  // namespace transform
}  // namespace apollo