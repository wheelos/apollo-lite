#!/usr/bin/env bash

set -euo pipefail

DEVICE="${DEVICE:-/dev/video2}"
WIDTH="${WIDTH:-1920}"
HEIGHT="${HEIGHT:-1080}"
FPS="${FPS:-30}"
TIMEOUT_SEC="${TIMEOUT_SEC:-12}"
RESULT_DIR="${RESULT_DIR:-/apollo/data/benchmarks/camera_gst_dma_uyvy}"
EXPECTED_PIXFMT="${EXPECTED_PIXFMT:-UYVY}"

mkdir -p "${RESULT_DIR}"

FMT_LOG="${RESULT_DIR}/device_fmt.log"
PIPELINE_LOG="${RESULT_DIR}/gst_pipeline.log"
JPEG_FILE="${RESULT_DIR}/camera_gst_dma_uyvy.jpg"

for cmd in v4l2-ctl gst-launch-1.0 timeout gst-inspect-1.0; do
  if ! command -v "${cmd}" >/dev/null 2>&1; then
    echo "missing required command: ${cmd}" >&2
    exit 1
  fi
done

for factory in nvv4l2camerasrc nvvidconv nvjpegenc; do
  if ! gst-inspect-1.0 "${factory}" >/dev/null 2>&1; then
    echo "missing required GStreamer factory: ${factory}" >&2
    exit 2
  fi
done

fmt_info="$(v4l2-ctl -d "${DEVICE}" --get-fmt-video 2>/dev/null || true)"
printf '%s\n' "${fmt_info}" > "${FMT_LOG}"

actual_pixfmt="$(printf '%s\n' "${fmt_info}" | awk -F': ' '/Pixel Format/ {gsub(/\047/, "", $2); print toupper($2); exit}')"
if [[ -z "${actual_pixfmt}" ]]; then
  echo "failed to read pixel format for ${DEVICE}" >&2
  exit 3
fi
if [[ "${actual_pixfmt}" != "${EXPECTED_PIXFMT}" ]]; then
  echo "expected ${EXPECTED_PIXFMT} on ${DEVICE}, got ${actual_pixfmt}" >&2
  exit 4
fi

rm -f "${JPEG_FILE}"
timeout "${TIMEOUT_SEC}" gst-launch-1.0 -e -v \
  nvv4l2camerasrc device="${DEVICE}" num-buffers=1 do-timestamp=true \
  ! "video/x-raw(memory:NVMM),format=(string)${EXPECTED_PIXFMT},width=(int)${WIDTH},height=(int)${HEIGHT},framerate=(fraction)${FPS}/1" \
  ! nvvidconv \
  ! "video/x-raw(memory:NVMM),format=(string)NV12" \
  ! nvjpegenc \
  ! filesink location="${JPEG_FILE}" sync=false > "${PIPELINE_LOG}" 2>&1

if [[ ! -s "${JPEG_FILE}" ]]; then
  echo "jpeg output was not created: ${JPEG_FILE}" >&2
  exit 5
fi
if ! grep -q "memory:NVMM" "${PIPELINE_LOG}"; then
  echo "pipeline did not negotiate NVMM memory" >&2
  exit 6
fi
if ! grep -Eq "UYVY|${EXPECTED_PIXFMT}" "${PIPELINE_LOG}"; then
  echo "pipeline log did not confirm ${EXPECTED_PIXFMT} negotiation" >&2
  exit 7
fi

jpeg_size_bytes="$(stat -c '%s' "${JPEG_FILE}")"

cat <<EOF
camera_gst DMA/UYVY validation passed
device=${DEVICE}
pixel_format=${actual_pixfmt}
jpeg=${JPEG_FILE}
jpeg_size_bytes=${jpeg_size_bytes}
logs=${RESULT_DIR}
EOF
