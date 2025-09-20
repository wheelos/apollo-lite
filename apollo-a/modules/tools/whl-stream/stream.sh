#!/usr/bin/env bash

set -euo pipefail

# --- Configuration ---
DEFAULT_HOST_IP="192.168.1.20"
DEFAULT_BASE_PORT=5000
DEFAULT_VIDEO_DEVICES=(0 1 2 3)
LOG_DIR="./gst_logs"
PID_DIR="./gst_pids"
LOG_MAX_SIZE=$((10 * 1024 * 1024))   # 10 MB per log
DEBUG_MODE="${DEBUG_MODE-0}"

# --- Utility Functions ---

usage() {
  cat <<EOF
Usage: $0 [start|stop|status] [HOST_IP] [BASE_PORT] [VIDEO_DEVICE...]
  start        Start camera streams
  stop         Stop all streams
  status       Query current stream status
  HOST_IP      Target IP (default: $DEFAULT_HOST_IP)
  BASE_PORT    Starting port (default: $DEFAULT_BASE_PORT)
  VIDEO_DEVICE List of camera device numbers (default: ${DEFAULT_VIDEO_DEVICES[*]})

Environment variables:
  DEBUG_MODE=1 to show detailed debug info
EOF
  exit 1
}

debug() {
  [[ "${DEBUG_MODE}" == "1" ]] && echo "DEBUG: $*" >&2
}

info() {
  echo -e "\033[1;32m[INFO]\033[0m $*"
}

warn() {
  echo -e "\033[1;33m[WARN]\033[0m $*"
}

error() {
  echo -e "\033[1;31m[ERROR]\033[0m $*" >&2
}

validate_ip() {
  if ! [[ $1 =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]]; then
    error "Invalid IP address: $1"; exit 1
  fi
}

validate_port() {
  if ! [[ $1 =~ ^[0-9]+$ ]] || (( $1 < 1024 || $1 > 65535 )); then
    error "Port must be in range 1024–65535: $1"; exit 1
  fi
}

check_tools() {
  for tool in gst-launch-1.0 lsof ps kill; do
    command -v $tool &>/dev/null || { error "$tool not installed!"; exit 1; }
  done
}

log_rotate() {
  local log="$1"
  if [[ -f "$log" && $(stat -c%s "$log") -gt $LOG_MAX_SIZE ]]; then
    mv "$log" "${log}.old.$(date "+%Y%m%d_%H%M%S")"
    info "Log $log reached max size, rotated"
  fi
}

# --- Core Functions ---

stream_cam() {
  local dev="$1" port="$2"
  local log="${LOG_DIR}/gst_cam_${dev}_${port}.log"
  local pidf="${PID_DIR}/gst_cam_${dev}_${port}.pid"

  info "Preparing to start /dev/video${dev} → ${HOST_IP}:${port}, log: ${log}"
  mkdir -p "$LOG_DIR" "$PID_DIR"

  # Check device
  if [[ ! -c "/dev/video${dev}" ]]; then
    warn "Device /dev/video${dev} does not exist, skipping"
    return
  fi

  # Check device availability
  if fuser "/dev/video${dev}" &>/dev/null; then
    warn "/dev/video${dev} is occupied by another process, skipping"
    return
  fi

  # Check if port is available
  if lsof -i UDP:"$port" &>/dev/null; then
    warn "Port ${port} is occupied, trying to recover zombie process"
    # Exception recovery: find corresponding pid, try to kill
    for upid in $(lsof -t -i UDP:"$port" || :); do
      warn "Killing process occupying port $port: $upid"
      kill -9 $upid || true
    done
    sleep 1
    if lsof -i UDP:"$port" &>/dev/null; then
      error "Port ${port} still occupied, giving up starting this stream"
      return
    fi
  fi

  log_rotate "$log"

  gst-launch-1.0 -e \
    v4l2src device="/dev/video${dev}" ! \
    "video/x-raw,format=YUY2,width=1920,height=1080,framerate=30/1" ! \
    nvvidconv ! \
    "video/x-raw(memory:NVMM),format=NV12,width=1920,height=1080" ! \
    nvv4l2h264enc maxperf-enable=1 preset=5 ! \
    rtph264pay config-interval=1 pt=96 ! \
    udpsink host="${HOST_IP}" port="${port}" \
    >"$log" 2>&1 &
  local pid=$!

  # Check if gst started successfully
  sleep 2
  if ! ps -p "$pid" &>/dev/null; then
    error "Failed to start /dev/video${dev} (port ${port}), see $log"
    return
  fi

  echo "$pid" > "$pidf"
  info "Recorded PID $pid → $pidf"
}

stop_all_streams() {
  info "Preparing to stop all streams"
  [[ -d "$PID_DIR" ]] || { warn "$PID_DIR does not exist"; return; }
  local pids=()
  for f in "$PID_DIR"/*.pid; do
    [[ -f "$f" ]] || continue
    local pid
    pid=$(<"$f")
    if ps -p "$pid" &>/dev/null; then
      # Only stop gst-launch processes associated with this script
      local pname
      pname=$(ps -p "$pid" -o comm= 2>/dev/null || echo "")
      if [[ "$pname" == *gst-launch* ]]; then
        pids+=("$pid")
      else
        warn "PID $pid ($pname) is not gst-launch process, skipping"
      fi
    else
      warn "Skipping invalid/ended PID file: $f"
      rm -f "$f"
    fi
  done
  if ((${#pids[@]} == 0)); then
    info "No running streams"
    return
  fi
  info "Trying to gracefully terminate PID: ${pids[*]}"
  kill "${pids[@]}" || true
  sleep 2
  for pid in "${pids[@]}"; do
    if ps -p "$pid" &>/dev/null; then
      warn "Force killing $pid"
      kill -9 "$pid" || true
    fi
  done
  rm -f "$PID_DIR"/*.pid
  info "All stopped"
}

status_streams() {
  [[ -d "$PID_DIR" ]] || { info "$PID_DIR does not exist"; return; }
  local found=0
  for f in "$PID_DIR"/*.pid; do
    [[ -f "$f" ]] || continue
    local pid cam port
    pid=$(<"$f")
    cam=$(basename "$f" | awk -F'[_]' '{print $3}')
    port=$(basename "$f" | awk -F'[_]' '{print $4}' | cut -d'.' -f1)
    if ps -p "$pid" &>/dev/null; then
      info "Stream [Camera $cam] → Port $port is running, PID=$pid"
      found=1
    else
      warn "Stream [Camera $cam] → Port $port not running, cleaning up stale PID file $f"
      rm -f "$f"
    fi
  done
  ((found)) || info "No running streams"
}

# --- Signal handling: graceful exit, cleanup child processes ---
cleanup() {
  info "Signal received, cleaning up..."
  stop_all_streams
  exit
}
trap cleanup INT TERM
trap cleanup EXIT

# --- Main ---

check_tools

[[ $# -ge 1 ]] || usage
COMMAND="$1"; shift

case "$COMMAND" in
  start)
    HOST_IP="${1:-$DEFAULT_HOST_IP}"; shift || :
    validate_ip "$HOST_IP"

    BASE_PORT="${1:-$DEFAULT_BASE_PORT}"; shift || :
    validate_port "$BASE_PORT"

    video_devices=("${@:-${DEFAULT_VIDEO_DEVICES[@]}}")
    cam_count=${#video_devices[@]}

    # Check port validity (not exceeding 65535)
    if (( BASE_PORT + cam_count - 1 > 65535 )); then
      error "Starting port and camera count exceed port range"
      exit 1
    fi

    info "Config → IP=$HOST_IP, starting port=$BASE_PORT, cameras=${video_devices[*]}"
    mkdir -p "$LOG_DIR" "$PID_DIR"

    for i in "${!video_devices[@]}"; do
      stream_cam "${video_devices[i]}" $((BASE_PORT + i))
    done

    info "All streams started. Ctrl+C to stop; or use '$0 stop'"
    wait
    ;;
  stop)
    stop_all_streams
    ;;
  status)
    status_streams
    ;;
  *)
    error "Invalid command: '$COMMAND'"
    usage
    ;;
esac
