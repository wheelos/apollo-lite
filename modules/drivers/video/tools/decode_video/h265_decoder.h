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

#pragma once

#include <memory>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/error.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

namespace apollo {
namespace drivers {
namespace video {

/**
 * @class H265Decoder
 * @brief H265Decoder uses FFmpeg 7.x API for H.265 decoding and MJPEG encoding.
 */
class H265Decoder {
 public:
  enum class DecodingResult {
    SUCCESS,
    FATAL,
    WARN,
  };

 public:
  H265Decoder() = default;

  // Init decoder by acquiring resources
  bool Init();

  /**
   * @brief Process frames: Decodes H265 -> Encodes to MJPEG
   * Note: This function assumes 1 input packet produces 1 output packet,
   * which is generally true for low-latency video but not guaranteed by the
   * standard.
   */
  DecodingResult Process(const uint8_t* indata, const int32_t insize,
                         std::vector<uint8_t>* outdata) const;

  // Destructor, releasing the resources
  ~H265Decoder();

  // Getter of codec_ctx_h265_
  AVCodecContext* GetCodecCtxH265() const { return codec_ctx_h265_; }

 private:
  void Release();

  // Helper for internal error logging
  void LogError(const char* func_name, int error_code) const;

  AVCodecContext* codec_ctx_h265_ = nullptr;
  AVCodecContext* codec_ctx_jpeg_ = nullptr;
  AVFrame* yuv_frame_ = nullptr;

  // Reuse packets to avoid frequent malloc/free
  // Note: Mutable because Process is const but these are internal buffers
  mutable AVPacket* input_packet_ = nullptr;
  mutable AVPacket* output_packet_ = nullptr;
};

}  // namespace video
}  // namespace drivers
}  // namespace apollo
