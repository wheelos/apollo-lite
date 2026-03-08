/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
#include "modules/map/tools/map_datachecker/server/pj_transformer.h"

#include <cmath>
#include <iostream>
#include <sstream>

#include "cyber/cyber.h"

namespace apollo {
namespace hdmap {

PJTransformer::PJTransformer(int zone_id) : pj_(nullptr) {
  // init projPJ
  std::stringstream stream;
  stream << "+proj=utm +zone=" << zone_id << " +ellps=WGS84";

  PJ* tmp_pj =
      proj_create_crs_to_crs(PJ_DEFAULT_CTX, "+proj=latlong +ellps=WGS84",
                             stream.str().c_str(), nullptr);
  if (tmp_pj == nullptr) {
    AERROR << "proj4 init failed! " << stream.str() << std::endl;
    return;
  }

  pj_ = proj_normalize_for_visualization(PJ_DEFAULT_CTX, tmp_pj);
  proj_destroy(tmp_pj);

  if (pj_ == nullptr) {
    AERROR << "proj4 normalize failed!";
    return;
  }
  AINFO << "proj4 init success" << std::endl;
}

PJTransformer::~PJTransformer() {
  if (pj_) {
    proj_destroy(pj_);
    pj_ = nullptr;
  }
}
int PJTransformer::LatlongToUtm(int64_t point_count, int point_offset,
                                double* x, double* y, double* z) {
  if (!pj_) {
    AERROR << "pj_ is null";
    return -1;
  }

  // proj_trans expects degrees but the caller in Apollo sends radians,
  // so we need to multiply by 180.0 / M_PI before transforming.
  for (int i = 0; i < point_count; ++i) {
    int idx = i * point_offset;
    x[idx] *= 180.0 / M_PI;
    y[idx] *= 180.0 / M_PI;
  }

  size_t stride = sizeof(double) * point_offset;
  size_t ret =
      proj_trans_generic(pj_, PJ_FWD, x, stride, point_count, y, stride,
                         point_count, z, stride, point_count, nullptr, 0, 0);

  // proj_trans_generic returns the number of successfully transformed points.
  // In the old API, pj_transform returned 0 on success.
  if (ret != static_cast<size_t>(point_count)) {
    return -1;
  }
  return 0;
}

}  // namespace hdmap
}  // namespace apollo
