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

#include "modules/localization/msf/common/util/frame_transform.h"

#include <string>
#include <cmath>

#include "absl/strings/str_cat.h"
#include "modules/common/util/proj_helper.h"

namespace apollo {
namespace localization {
namespace msf {

static const double RAD_TO_DEG = 180.0 / M_PI;
static const double DEG_TO_RAD = M_PI / 180.0;

bool FrameTransform::LatlonToUtmXY(double lon_rad, double lat_rad,
                                   UTMCoor *utm_xy) {
  int zone = static_cast<int>((lon_rad * RAD_TO_DEG + 180) / 6) + 1;
  std::string latlon_src =
      "+proj=longlat +ellps=GRS80 +towgs84=0,0,0,0,0,0,0 +no_defs";
  std::string utm_dst =
      absl::StrCat("+proj=utm +zone=", zone, " +ellps=GRS80 +units=m +no_defs");

  PJ* pj = apollo::common::util::ProjHelper::CreateNormalizedCrsToCrs(
      PJ_DEFAULT_CTX, latlon_src, utm_dst);
  if (!pj) return false;

  PJ_COORD c;
  c.xyzt.x = lon_rad * RAD_TO_DEG;
  c.xyzt.y = lat_rad * RAD_TO_DEG;
  c.xyzt.z = 0.0;
  c.xyzt.t = 0.0;

  PJ_COORD res = proj_trans(pj, PJ_FWD, c);
  proj_destroy(pj);

  if (res.xyzt.x == HUGE_VAL || res.xyzt.y == HUGE_VAL) return false;

  utm_xy->x = res.xyzt.x;
  utm_xy->y = res.xyzt.y;
  return true;
}

bool FrameTransform::UtmXYToLatlon(double x, double y, int zone, bool southhemi,
                                   WGS84Corr *latlon) {
  std::string latlon_src =
      "+proj=longlat +ellps=GRS80 +towgs84=0,0,0,0,0,0,0 +no_defs";
  std::string utm_dst =
      absl::StrCat("+proj=utm +zone=", zone, " +ellps=GRS80 +units=m +no_defs");

  // We are mapping from UTM to LatLon, so we invert the transformation.
  PJ* pj = apollo::common::util::ProjHelper::CreateNormalizedCrsToCrs(
      PJ_DEFAULT_CTX, latlon_src, utm_dst);
  if (!pj) return false;

  PJ_COORD c;
  c.xyzt.x = x;
  c.xyzt.y = y;
  c.xyzt.z = 0.0;
  c.xyzt.t = 0.0;

  // Use PJ_INV because our transform was created as latlon -> utm
  PJ_COORD res = proj_trans(pj, PJ_INV, c);
  proj_destroy(pj);

  if (res.xyzt.x == HUGE_VAL || res.xyzt.y == HUGE_VAL) return false;

  latlon->log = res.xyzt.x * DEG_TO_RAD;
  latlon->lat = res.xyzt.y * DEG_TO_RAD;
  return true;
}

bool FrameTransform::XYZToBlh(const Vector3d &xyz, Vector3d *blh) {
  std::string xyz_src = "+proj=geocent +datum=WGS84";
  std::string blh_dst = "+proj=latlong +datum=WGS84";

  PJ* pj = apollo::common::util::ProjHelper::CreateNormalizedCrsToCrs(
      PJ_DEFAULT_CTX, xyz_src, blh_dst);
  if (!pj) return false;

  PJ_COORD c;
  c.xyz.x = xyz[0];
  c.xyz.y = xyz[1];
  c.xyz.z = xyz[2];

  PJ_COORD res = proj_trans(pj, PJ_FWD, c);
  proj_destroy(pj);

  if (res.xyz.x == HUGE_VAL) return false;

  (*blh)[0] = res.xyz.x * DEG_TO_RAD; // Longitude to rad
  (*blh)[1] = res.xyz.y * DEG_TO_RAD; // Latitude to rad
  (*blh)[2] = res.xyz.z;
  return true;
}

bool FrameTransform::BlhToXYZ(const Vector3d &blh, Vector3d *xyz) {
  std::string blh_src = "+proj=latlong +datum=WGS84";
  std::string xyz_dst = "+proj=geocent +datum=WGS84";

  PJ* pj = apollo::common::util::ProjHelper::CreateNormalizedCrsToCrs(
      PJ_DEFAULT_CTX, blh_src, xyz_dst);
  if (!pj) return false;

  PJ_COORD c;
  // blh is in radians, convert to degrees for PROJ
  c.xyz.x = blh[0] * RAD_TO_DEG;
  c.xyz.y = blh[1] * RAD_TO_DEG;
  c.xyz.z = blh[2];

  PJ_COORD res = proj_trans(pj, PJ_FWD, c);
  proj_destroy(pj);

  if (res.xyz.x == HUGE_VAL) return false;

  (*xyz)[0] = res.xyz.x;
  (*xyz)[1] = res.xyz.y;
  (*xyz)[2] = res.xyz.z;
  return true;
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
