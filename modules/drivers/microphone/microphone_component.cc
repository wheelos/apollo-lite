/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include "modules/drivers/microphone/microphone_component.h"

#include <memory>

namespace apollo {
namespace drivers {
namespace microphone {

bool MicrophoneComponent::Init() {
  microphone_config_ptr_ = std::make_shared<MicrophoneConfig>();
  if (!apollo::cyber::common::GetProtoFromFile(config_file_path_,
                                               microphone_config_ptr_.get())) {
    return false;
  }
  AINFO << "Microphone config: " << microphone_config_ptr_->DebugString();
  if (!assembler_.Init(*microphone_config_ptr_)) {
    return false;
  }

  // new microphone device
  microphone_device_ptr_.reset(new Respeaker());
  if (!microphone_device_ptr_->init(microphone_config_ptr_)) {
    return false;
  }
  buffer_.resize(assembler_.chunk_bytes());

  writer_ptr_ =
      node_->CreateWriter<AudioData>(microphone_config_ptr_->channel_name());
  if (!writer_ptr_) {
    AERROR << "Failed to create microphone audio writer";
    return false;
  }
  running_.store(true);
  capture_thread_ = std::thread(&MicrophoneComponent::run, this);
  return true;
}

void MicrophoneComponent::run() {
  while (running_.load() && !cyber::IsShutdown()) {
    if (!microphone_device_ptr_->read_stream(microphone_config_ptr_->chunk(),
                                             buffer_.data())) {
      if (running_.load()) {
        AERROR << "Microphone capture stopped after a read failure";
      }
      break;
    }
    if (!running_.load()) {
      break;
    }
    auto audio = std::make_shared<AudioData>();
    if (assembler_.AddChunk(buffer_.data(), audio.get())) {
      FillHeader(node_->Name(), audio.get());
      writer_ptr_->Write(audio);
    }
  }
}

void MicrophoneComponent::Clear() {
  running_.store(false);
  if (microphone_device_ptr_) {
    microphone_device_ptr_->Stop();
  }
  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }
  microphone_device_ptr_.reset();
}

MicrophoneComponent::~MicrophoneComponent() { Clear(); }

}  // namespace microphone
}  // namespace drivers
}  // namespace apollo
