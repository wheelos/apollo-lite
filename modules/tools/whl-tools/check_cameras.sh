#!/usr/bin/env bash

# ==============================================================================
# Jetson V4L2 Hardware Stream Detector
# Best Practice Implementation for Embedded Vision & Autonomous Driving Systems
# ==============================================================================

set -u

DATA_DIR="data/video_test"
TIMEOUT_SEC=3

# ANSI Color codes for clean CLI reporting
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# 1. Dependency Check
for cmd in v4l2-ctl ffmpeg timeout; do
  if ! command -v "$cmd" &> /dev/null; then
    echo -e "${RED}[ERROR] Required command '$cmd' is not installed.${NC}" >&2
    exit 1
  fi
done

mkdir -p "$DATA_DIR"

# 2. Cleanup Trap (Automatically remove temporary raw frames on exit/SIGINT)
trap 'rm -f "$DATA_DIR"/*_temp.raw 2>/dev/null' EXIT INT TERM

echo "=================================================="
echo " Jetson V4L2 Hardware Stream Detector"
echo " Target Directory: $DATA_DIR"
echo "=================================================="

active_devs=()

# 3. Iterate over available video devices
for dev in /dev/video*; do
  [ -e "$dev" ] || continue

  dev_name=$(basename "$dev")
  raw_file="$DATA_DIR/${dev_name}_temp.raw"
  jpg_file="$DATA_DIR/${dev_name}_test.jpg"

  # Dynamically probe device resolution and pixel format
  fmt_info=$(v4l2-ctl -d "$dev" --get-fmt-video 2>/dev/null || true)
  width=$(echo "$fmt_info" | awk '/Width\/Height/ {print $3}' | cut -d'/' -f1)
  height=$(echo "$fmt_info" | awk '/Width\/Height/ {print $3}' | cut -d'/' -f2)
  pixfmt=$(echo "$fmt_info" | awk '/Pixel Format/ {print $3}' | tr -d "'" | tr '[:upper:]' '[:lower:]')

  # Fallback values if probing fails
  width=${width:-1920}
  height=${height:-1080}

  case "$pixfmt" in
    uyvy) ff_pixfmt="uyvy422" ;;
    nv12) ff_pixfmt="nv12" ;;
    *)    ff_pixfmt="yuyv422" ;;
  esac

  # Attempt MMAP direct frame capture
  timeout "$TIMEOUT_SEC" v4l2-ctl -d "$dev" --stream-mmap --stream-count=1 --stream-to="$raw_file" > /dev/null 2>&1 || true

  # Validate frame capture
  if [ -s "$raw_file" ]; then
    size=$(du -h "$raw_file" | cut -f1)

    # Convert captured RAW frame to JPEG
    ffmpeg -hide_banner -loglevel error \
      -f rawvideo \
      -pixel_format "$ff_pixfmt" \
      -video_size "${width}x${height}" \
      -i "$raw_file" \
      -vframes 1 \
      "$jpg_file" -y > /dev/null 2>&1 || true

    if [ -s "$jpg_file" ]; then
      printf "[${GREEN}+${NC}] %-12s : OK (Size: %s | %dx%d | %s)\n" "$dev" "$size" "$width" "$height" "$ff_pixfmt"
      active_devs+=("$dev")
    else
      # If raw succeeded but conversion failed
      printf "[${GREEN}+${NC}] %-12s : OK (Size: %s | Raw Only)\n" "$dev" "$size"
      active_devs+=("$dev")
    fi
    rm -f "$raw_file"
  else
    printf "[${RED}-${NC}] %-12s : NO OUTPUT\n" "$dev"
    rm -f "$raw_file" "$jpg_file" 2>/dev/null
  fi
done

echo "--------------------------------------------------"
if [ ${#active_devs[@]} -gt 0 ]; then
  echo -e "Active Devices (${#active_devs[@]}):"
  for d in "${active_devs[@]}"; do
    echo -e "  ✔ $d -> Image saved to $DATA_DIR/$(basename "$d")_test.jpg"
  done
else
  echo -e "${RED}❌ No active camera streams found.${NC}"
fi
echo "=================================================="
