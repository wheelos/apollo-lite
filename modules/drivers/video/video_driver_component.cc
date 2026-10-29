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

#include "modules/drivers/video/video_driver_component.h"

#include <sys/stat.h>

#include <cstdio>
#include <memory>

#include "cyber/common/file.h"

namespace apollo {
namespace drivers {
namespace video {

using cyber::common::EnsureDirectory;

bool CompCameraH265Compressed::Init() {
  AINFO << "Initialize video driver component.";

  CameraH265Config video_config;
  if (!GetProtoConfig(&video_config)) {
    return false;
  }

  AINFO << "Velodyne config: " << video_config.DebugString();

  camera_deivce_.reset(new CameraDriver(&video_config));
  if (!camera_deivce_->Init()) {
    return false;
  }

  if (camera_deivce_->Record()) {
    // Use current directory to save record file if H265_SAVE_FOLDER environment
    // is not set.
    record_folder_ = cyber::common::GetEnv("H265_SAVE_FOLDER", ".");
    AINFO << "Record folder: " << record_folder_;

    struct stat st;
    if (stat(record_folder_.c_str(), &st) < 0) {
      bool ret = EnsureDirectory(record_folder_);
      AINFO_IF(ret) << "Record folder is created successfully.";
    }
  }
  writer_ = node_->CreateWriter<CompressedImage>(
      video_config.compress_conf().output_channel());
  if (!writer_) {
    AERROR << "Failed to create video image writer";
    return false;
  }

  running_.store(true);
  video_thread_ = std::shared_ptr<std::thread>(
      new std::thread(std::bind(&CompCameraH265Compressed::VideoPoll, this)));
  return true;
}

void CompCameraH265Compressed::Clear() {
  running_.store(false);
  if (camera_deivce_) {
    camera_deivce_->Stop();
  }
  if (video_thread_ && video_thread_->joinable()) {
    video_thread_->join();
  }
  camera_deivce_.reset();
}

void CompCameraH265Compressed::VideoPoll() {
  std::ofstream fout;
  if (camera_deivce_->Record()) {
    char name[256];
    snprintf(name, sizeof(name), "%s/encode_%d.h265", record_folder_.c_str(),
             camera_deivce_->Port());
    AINFO << "Output file: " << name;
    fout.open(name, std::ios::binary);
    if (!fout.good()) {
      AERROR << "Failed to open output file: " << name;
    }
  }
  int poll_failure_number = 0;
  while (running_.load() && !apollo::cyber::IsShutdown()) {
    // Cyber readers may retain the shared message after Write returns.
    pb_image_ = std::make_shared<CompressedImage>();
    if (!camera_deivce_->Poll(pb_image_)) {
      if (!running_.load()) {
        break;
      }
      AERROR << "H265 poll failed on port: " << camera_deivce_->Port();
      static constexpr int kTolerance = 256;
      if (++poll_failure_number > kTolerance) {
        AERROR << "H265 poll keep failing for " << kTolerance << " times, Exit";
        break;
      }
      continue;
    }
    if (!running_.load()) {
      break;
    }
    poll_failure_number = 0;
    pb_image_->mutable_header()->set_timestamp_sec(
        cyber::Time::Now().ToSecond());
    AINFO << "Send compressed image.";
    writer_->Write(pb_image_);

    if (camera_deivce_->Record()) {
      fout.write(pb_image_->data().c_str(), pb_image_->data().size());
    }
  }

  if (camera_deivce_->Record()) {
    fout.close();
  }
}

}  // namespace video
}  // namespace drivers
}  // namespace apollo
