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

#include "modules/drivers/microphone/respeaker.h"

#include <memory>
#include <string>

namespace apollo {
namespace drivers {
namespace microphone {

// Helper functions
void report_error(PaError err, const std::string &func_name) {
  AERROR << "an error occured while calling " << func_name;
  AERROR << "error number: " << err;
  AERROR << "error message: " << Pa_GetErrorText(err);
}

// Stream
Stream::~Stream() {
  if (pastream_ptr_) {
    const auto error = Pa_CloseStream(pastream_ptr_);
    if (error != paNoError) {
      report_error(error, "Pa_CloseStream");
    }
  }
}

bool Stream::init_stream(int rate, int channels, int chunk,
                         int input_device_index, PaSampleFormat format) {
  // Init parameters of input device
  const auto *device = Pa_GetDeviceInfo(input_device_index);
  if (!device || channels > device->maxInputChannels) {
    AERROR << "Invalid microphone device or unsupported channel count";
    return false;
  }
  PaStreamParameters parameters{};
  parameters.device = input_device_index;
  parameters.channelCount = channels;
  parameters.sampleFormat = format;
  parameters.suggestedLatency = device->defaultLowInputLatency;

  auto err = Pa_OpenStream(
      &pastream_ptr_, &parameters, nullptr, rate, chunk,
      paClipOff,  // we only use input so don't bother clipping them *
      nullptr, nullptr);
  if (err != paNoError) {
    report_error(err, "Pa_OpenStream");
    return false;
  }
  err = Pa_StartStream(pastream_ptr_);
  if (err != paNoError) {
    report_error(err, "Pa_StartStream");
    return false;
  }
  return true;
}

bool Stream::read_stream(int n_frames, char *buffer) const {
  const auto err =
      Pa_ReadStream(pastream_ptr_, reinterpret_cast<void *>(buffer), n_frames);
  if (err != paNoError) {
    report_error(err, "Pa_ReadStream");
    return false;
  }
  return true;
}

void Stream::Stop() {
  if (pastream_ptr_) {
    const auto error = Pa_AbortStream(pastream_ptr_);
    if (error != paNoError && error != paStreamIsStopped) {
      report_error(error, "Pa_AbortStream");
    }
  }
}

// Respeaker
Respeaker::~Respeaker() {
  stream_ptr_.reset();
  if (initialized_) {
    const auto error = Pa_Terminate();
    if (error != paNoError) {
      report_error(error, "Pa_Terminate");
    }
  }
}
bool Respeaker::init(
    const std::shared_ptr<const MicrophoneConfig> &microphone_config) {
  if (microphone_config->microphone_model() != MicrophoneConfig::RESPEAKER) {
    AERROR << "Microphone driver only supports respeaker model in config file";
    return false;
  }
  const auto err = Pa_Initialize();
  if (err != paNoError) {
    report_error(err, "Pa_Initialize");
    return false;
  }
  initialized_ = true;

  const PaDeviceIndex device_index = get_respeaker_index();
  if (device_index < 0) {
    return false;
  }
  const auto format = get_format_from_width(microphone_config->sample_width());
  if (format == static_cast<PaSampleFormat>(-1)) {
    return false;
  }
  stream_ptr_.reset(new Stream());
  return stream_ptr_->init_stream(
      microphone_config->sample_rate(), microphone_config->channel_type_size(),
      microphone_config->chunk(), device_index, format);
}

const PaSampleFormat Respeaker::get_format_from_width(int width,
                                                      bool is_unsigned) const {
  switch (width) {
    case 1:
      return is_unsigned ? paUInt8 : paInt8;
    case 2:
      return paInt16;
    case 3:
      return paInt24;
    case 4:
      return paFloat32;
    default:
      break;
  }
  AERROR << "invalid width: " << width;
  return -1;
}

const PaDeviceIndex Respeaker::get_respeaker_index() const {
  // return index of respeaker
  const PaHostApiInfo *host_api_info = get_host_api_info(0);
  if (!host_api_info) {
    return paInvalidHostApi;
  }
  const PaDeviceInfo *device_info = nullptr;
  PaDeviceIndex real_index;
  for (PaDeviceIndex i = 0; i < host_api_info->deviceCount; ++i) {
    real_index = host_api_device_index_to_device_index(0, i);
    if (real_index < 0) {
      continue;
    }
    device_info = get_device_info(real_index);
    if (device_info && device_info->name &&
        std::string(device_info->name).find("ReSpeaker") != std::string::npos) {
      return real_index;
    }
  }
  AERROR << "respeaker device not found";
  return -1;
}

const PaDeviceInfo *Respeaker::get_device_info(
    const PaDeviceIndex index) const {
  const PaDeviceInfo *device_info =
      reinterpret_cast<const PaDeviceInfo *>(Pa_GetDeviceInfo(index));
  if (!device_info) {
    AERROR << "internal error: invalid device index" << index;
  }

  return device_info;
}

const PaDeviceIndex Respeaker::host_api_device_index_to_device_index(
    const PaHostApiIndex host_api, const int host_api_device_index) const {
  // Get standard device index from host-API-specific device index
  PaDeviceIndex device_index =
      Pa_HostApiDeviceIndexToDeviceIndex(host_api, host_api_device_index);
  if (device_index < 0) {
    report_error(device_index, "Pa_HostApiDeviceIndexToDeviceIndex");
  }
  return device_index;
}

const PaHostApiInfo *Respeaker::get_host_api_info(
    const PaHostApiIndex index) const {
  // Get host api info by it's index
  const PaHostApiInfo *pa_host_api_info =
      reinterpret_cast<const PaHostApiInfo *>(Pa_GetHostApiInfo(index));
  if (!pa_host_api_info) {
    AERROR << "internal error: invalid Host Api Index " << index;
  }
  return pa_host_api_info;
}

bool Respeaker::read_stream(int n_frames, char *buffer) const {
  return stream_ptr_->read_stream(n_frames, buffer);
}

void Respeaker::Stop() {
  if (stream_ptr_) {
    stream_ptr_->Stop();
  }
}

}  // namespace microphone
}  // namespace drivers
}  // namespace apollo
