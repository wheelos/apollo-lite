#!/usr/bin/env bash

set -euo pipefail

DEVICE="${DEVICE:-/dev/video2}"
WIDTH="${WIDTH:-1920}"
HEIGHT="${HEIGHT:-1080}"
FPS="${FPS:-30}"
BITRATE="${BITRATE:-8000000}"
NUM_BUFFERS="${NUM_BUFFERS:-180}"
OUT_FILE="${OUT_FILE:-}"

PIPE=(
  nvv4l2camerasrc "device=${DEVICE}" "num-buffers=${NUM_BUFFERS}"
  "!" "video/x-raw(memory:NVMM),format=(string)UYVY,width=(int)${WIDTH},height=(int)${HEIGHT},framerate=(fraction)${FPS}/1"
  "!" nvvidconv
  "!" "video/x-raw(memory:NVMM),format=(string)NV12"
  "!" nvv4l2h264enc "bitrate=${BITRATE}" "iframeinterval=${FPS}" "idrinterval=${FPS}" "control-rate=1" "preset-level=1" "output-io-mode=5" "maxperf-enable=1" "insert-sps-pps=true"
  "!" h264parse
)

if [[ -n "${OUT_FILE}" ]]; then
  PIPE+=("!" qtmux "!" filesink "location=${OUT_FILE}" "sync=false")
else
  PIPE+=("!" fakesink "sync=false")
fi

echo "Running NVIDIA HW H.264 transcode smoke pipeline..."
echo "device=${DEVICE} size=${WIDTH}x${HEIGHT} fps=${FPS} bitrate=${BITRATE} num_buffers=${NUM_BUFFERS}"

# shellcheck disable=SC2086
gst-launch-1.0 -v ${PIPE[*]}
