/******************************************************************************
 * Copyright 2026 The WheelOS Team. All Rights Reserved.
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

#include "gst/gst.h"

#include "modules/drivers/camera_gst/frame_types.h"

namespace apollo {
namespace drivers {
namespace camera_gst {

PublishedFrame ExtractCpuFrame(GstSample* sample);
GpuFrame ExtractNvmmFrame(GstSample* sample, const std::string& source_name);

}  // namespace camera_gst
}  // namespace drivers
}  // namespace apollo
