/* Copyright 2017 The Apollo Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
=========================================================================*/
#include "modules/map/hdmap/adapter/xml_parser/coordinate_convert_tool.h"

#include <cmath>

#include "glog/logging.h"

namespace apollo {
namespace hdmap {
namespace adapter {

CoordinateConvertTool::CoordinateConvertTool() : pj_(nullptr) {}

CoordinateConvertTool::~CoordinateConvertTool() {
  if (pj_) {
    proj_destroy(pj_);
    pj_ = nullptr;
  }
}

CoordinateConvertTool* CoordinateConvertTool::GetInstance() {
  static CoordinateConvertTool instance;
  return &instance;
}

Status CoordinateConvertTool::SetConvertParam(const std::string& source_param,
                                              const std::string& dst_param) {
  source_convert_param_ = source_param;
  dst_convert_param_ = dst_param;

  if (pj_) {
    proj_destroy(pj_);
    pj_ = nullptr;
  }

  PJ* tmp_pj =
      proj_create_crs_to_crs(PJ_DEFAULT_CTX, source_convert_param_.c_str(),
                             dst_convert_param_.c_str(), nullptr);
  if (!tmp_pj) {
    std::string err_msg =
        "Fail to proj_create_crs_to_crs with: " + source_convert_param_ +
        " and " + dst_convert_param_;
    return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
  }

  // Normalize to use (longitude, latitude) axis order instead of strict EPSG
  // order if applicable
  pj_ = proj_normalize_for_visualization(PJ_DEFAULT_CTX, tmp_pj);
  proj_destroy(tmp_pj);

  if (!pj_) {
    std::string err_msg = "Fail to proj_normalize_for_visualization";
    return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
  }

  return Status::OK();
}

Status CoordinateConvertTool::CoordiateConvert(const double longitude,
                                               const double latitude,
                                               const double height_ellipsoid,
                                               double* utm_x, double* utm_y,
                                               double* utm_z) {
  CHECK_NOTNULL(utm_x);
  CHECK_NOTNULL(utm_y);
  CHECK_NOTNULL(utm_z);
  if (!pj_) {
    std::string err_msg = "no transform param";
    return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
  }

  PJ_COORD c;
  c.xyzt.x = longitude;
  c.xyzt.y = latitude;
  c.xyzt.z = height_ellipsoid;
  c.xyzt.t = 0.0;

  PJ_COORD res = proj_trans(pj_, PJ_FWD, c);

  // If projection fails, proj_trans returns HUGE_VAL for coordinates
  if (res.xyzt.x == HUGE_VAL || res.xyzt.y == HUGE_VAL) {
    std::string err_msg = "fail to transform coordinate";
    return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
  }

  *utm_x = res.xyzt.x;
  *utm_y = res.xyzt.y;
  *utm_z = res.xyzt.z;

  return Status::OK();
}

}  // namespace adapter
}  // namespace hdmap
}  // namespace apollo
