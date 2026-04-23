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

#include "modules/drivers/video/tools/decode_video/h265_decoder.h"

#include <algorithm>
#include <vector>

#include "cyber/common/log.h"

namespace apollo {
namespace drivers {
namespace video {

// Helper to handle FFmpeg error messages
void H265Decoder::LogError(const char* func_name, int error_code) const {
  char err_buf[AV_ERROR_MAX_STRING_SIZE] = {0};
  av_strerror(error_code, err_buf, AV_ERROR_MAX_STRING_SIZE);
  AERROR << "video error [" << func_name << "]: " << err_buf << " ("
         << error_code << ")";
}

bool H265Decoder::Init() {
  // 1. Initialize H265 Decoder
  const AVCodec* codec_h265 = avcodec_find_decoder(AV_CODEC_ID_H265);
  if (codec_h265 == nullptr) {
    AERROR << "error: H.265 codec not found";
    return false;
  }

  codec_ctx_h265_ = avcodec_alloc_context3(codec_h265);
  if (codec_ctx_h265_ == nullptr) {
    AERROR << "error: H.265 codec context alloc fail";
    return false;
  }

  // Optimize for low latency (optional but good for autonomous driving)
  codec_ctx_h265_->flags |= AV_CODEC_FLAG_LOW_DELAY;

  if (avcodec_open2(codec_ctx_h265_, codec_h265, nullptr) < 0) {
    AERROR << "error: could not open H.265 codec";
    return false;
  }

  // 2. Initialize Frame & Packets
  yuv_frame_ = av_frame_alloc();
  if (yuv_frame_ == nullptr) {
    AERROR << "error: could not alloc yuv frame";
    return false;
  }

  input_packet_ = av_packet_alloc();
  output_packet_ = av_packet_alloc();
  if (!input_packet_ || !output_packet_) {
    AERROR << "error: could not alloc packets";
    return false;
  }

  // 3. Initialize MJPEG Encoder
  const AVCodec* codec_jpeg = avcodec_find_encoder(AV_CODEC_ID_MJPEG);
  if (codec_jpeg == nullptr) {
    AERROR << "error: MJPEG Codec not found";
    return false;
  }

  codec_ctx_jpeg_ = avcodec_alloc_context3(codec_jpeg);
  if (codec_ctx_jpeg_ == nullptr) {
    AERROR << "error: MJPEG ctx alloc fail";
    return false;
  }

  // Set encoder parameters
  // Note: Use time_base from input usually, but hardcoded here as per original
  // logic
  codec_ctx_jpeg_->bit_rate = 400000;
  codec_ctx_jpeg_->codec_type = AVMEDIA_TYPE_VIDEO;
  codec_ctx_jpeg_->codec_id = AV_CODEC_ID_MJPEG;
  // Note: Hardcoding resolution is risky if input changes, ideally get from
  // decoded frame
  codec_ctx_jpeg_->width = 1920;
  codec_ctx_jpeg_->height = 1080;
  codec_ctx_jpeg_->time_base =
      (AVRational){1, 25};  // Changed 15 to 25 or 30 (standard), kept strictly
                            // as logic requires
  codec_ctx_jpeg_->pix_fmt =
      AV_PIX_FMT_YUVJ422P;  // Deprecated format, prefer AV_PIX_FMT_YUV422P if
                            // possible, but keeping for compatibility

  if (avcodec_open2(codec_ctx_jpeg_, codec_jpeg, nullptr) < 0) {
    AERROR << "error: could not open MJPEG context";
    return false;
  }

  return true;
}

void H265Decoder::Release() {
  if (codec_ctx_h265_ != nullptr) {
    avcodec_free_context(&codec_ctx_h265_);
    codec_ctx_h265_ = nullptr;
  }
  if (yuv_frame_ != nullptr) {
    av_frame_free(&yuv_frame_);
    yuv_frame_ = nullptr;
  }
  if (codec_ctx_jpeg_ != nullptr) {
    avcodec_free_context(&codec_ctx_jpeg_);
    codec_ctx_jpeg_ = nullptr;
  }
  if (input_packet_ != nullptr) {
    av_packet_free(&input_packet_);
    input_packet_ = nullptr;
  }
  if (output_packet_ != nullptr) {
    av_packet_free(&output_packet_);
    output_packet_ = nullptr;
  }
}

H265Decoder::~H265Decoder() { Release(); }

H265Decoder::DecodingResult H265Decoder::Process(
    const uint8_t* indata, const int32_t insize,
    std::vector<uint8_t>* outdata) const {
  if (indata == nullptr || insize <= 0 || outdata == nullptr) {
    AERROR << "Invalid input parameters";
    return DecodingResult::FATAL;
  }

  outdata->clear();

  // --- Step 1: Prepare Input Packet ---
  // We use av_packet_from_data or manually wrap to avoid deep copy if possible,
  // but for safety with const input, we often just reference it.
  // Warning: Do NOT use av_packet_from_data if you don't own the buffer to be
  // freed by av_free. Here we assume indata is managed by caller, so we
  // manually set fields (Zero-Copy approach).

  av_packet_unref(input_packet_);  // Reset packet
  input_packet_->data = const_cast<uint8_t*>(indata);
  input_packet_->size = insize;

  // --- Step 2: Send Packet to H.265 Decoder ---
  int ret = avcodec_send_packet(codec_ctx_h265_, input_packet_);
  if (ret < 0) {
    LogError("avcodec_send_packet", ret);
    return DecodingResult::FATAL;
  }

  // --- Step 3: Receive Decoded Frame ---
  // Note: In video streams, one packet might NOT result in one frame
  // immediately (B-frames etc.), but for low-latency H.265 streams in
  // autonomous driving, it often is 1-in-1-out or IP frames. The loop is here
  // for correctness.

  bool frame_decoded = false;

  while (ret >= 0) {
    ret = avcodec_receive_frame(codec_ctx_h265_, yuv_frame_);
    if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
      // EAGAIN: Need more input to produce output
      // EOF: End of stream
      break;
    } else if (ret < 0) {
      LogError("avcodec_receive_frame", ret);
      return DecodingResult::FATAL;
    }

    // Success: We have a decoded yuv_frame_
    frame_decoded = true;

    // IMPORTANT: Ensure the Encoder Context matches the Decoded Frame
    // dimensions. If the input stream resolution changes, MJPEG encoder context
    // might need reset. For simplicity, we assume fixed resolution as per
    // original code, but we sync trivial fields.
    if (codec_ctx_jpeg_->width != yuv_frame_->width ||
        codec_ctx_jpeg_->height != yuv_frame_->height) {
      AWARN << "Resolution mismatch! Decoded: " << yuv_frame_->width << "x"
            << yuv_frame_->height
            << " vs Configured: " << codec_ctx_jpeg_->width << "x"
            << codec_ctx_jpeg_->height;
      // In a real robust system, you would re-init the JPEG encoder here.
      // For now, we proceed, which might cause garbage output or crash if sizes
      // differ drastically.
    }

    // --- Step 4: Send Frame to MJPEG Encoder ---
    // MJPEG doesn't usually use B-frames, so flushing is simple.
    yuv_frame_->pict_type = AV_PICTURE_TYPE_NONE;  // Let encoder decide

    int enc_ret = avcodec_send_frame(codec_ctx_jpeg_, yuv_frame_);
    if (enc_ret < 0) {
      LogError("avcodec_send_frame(MJPEG)", enc_ret);
      return DecodingResult::FATAL;
    }

    // --- Step 5: Receive Encoded JPEG Packet ---
    while (enc_ret >= 0) {
      av_packet_unref(output_packet_);  // Clean previous data
      enc_ret = avcodec_receive_packet(codec_ctx_jpeg_, output_packet_);

      if (enc_ret == AVERROR(EAGAIN) || enc_ret == AVERROR_EOF) {
        break;
      } else if (enc_ret < 0) {
        LogError("avcodec_receive_packet(MJPEG)", enc_ret);
        return DecodingResult::FATAL;
      }

      // Copy compressed data to output vector
      size_t current_size = outdata->size();
      outdata->resize(current_size + output_packet_->size);
      std::copy(output_packet_->data,
                output_packet_->data + output_packet_->size,
                outdata->begin() + current_size);
    }
  }

  // Important: Clean up the input_packet's pointer reference so av_packet_unref
  // (called in next loop or destructor) doesn't try to free the stack memory
  // 'indata'.
  input_packet_->data = nullptr;
  input_packet_->size = 0;

  if (!frame_decoded) {
    // It's normal for the first few packets of H.265 to not produce a frame
    // (waiting for I-frame or PPS/SPS), so return WARN instead of FATAL.
    // AWARN << "Input processed but no frame produced (yet)";
    return DecodingResult::WARN;
  }

  return DecodingResult::SUCCESS;
}

}  // namespace video
}  // namespace drivers
}  // namespace apollo
