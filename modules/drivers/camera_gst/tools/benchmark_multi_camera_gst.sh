#!/usr/bin/env bash

set -euo pipefail

CAMERA_COUNTS="${CAMERA_COUNTS:-1,2,4,6}"
BENCH_SECONDS="${BENCH_SECONDS:-30}"
WARMUP_SECONDS="${WARMUP_SECONDS:-10}"
RESULT_ROOT="${RESULT_ROOT:-/apollo/data/benchmarks/camera_gst_multi}"
CYBER_LAUNCH_BIN="${CYBER_LAUNCH_BIN:-/apollo/bazel-bin/cyber/tools/cyber_launch/cyber_launch}"
CYBER_LAUNCH_PY="${CYBER_LAUNCH_PY:-/apollo/cyber/tools/cyber_launch/cyber_launch.py}"
TEGRATSTATS_BIN="${TEGRATSTATS_BIN:-$(command -v tegrastats || true)}"

CAMERA_NAMES_DEFAULT="cam0,cam1,cam2,cam3,cam4,cam5"
CAMERA_URIS_DEFAULT="/dev/video2,/dev/video2,/dev/video2,/dev/video2,/dev/video2,/dev/video2"
CAMERA_NAMES="${CAMERA_NAMES:-${CAMERA_NAMES_DEFAULT}}"
CAMERA_URIS="${CAMERA_URIS:-${CAMERA_URIS_DEFAULT}}"
CAMERA_FOURCC="${CAMERA_FOURCC:-YUYV}"
CAMERA_WIDTH="${CAMERA_WIDTH:-1920}"
CAMERA_HEIGHT="${CAMERA_HEIGHT:-1080}"
CAMERA_FPS="${CAMERA_FPS:-30.0}"
OUTPUT_FORMAT="${OUTPUT_FORMAT:-RGB}"
CAPTURE_BACKEND="${CAPTURE_BACKEND:-AUTO}"
ALLOW_BACKEND_FALLBACK="${ALLOW_BACKEND_FALLBACK:-true}"
USE_REFERENCE_SINGLE_CASE="${USE_REFERENCE_SINGLE_CASE:-true}"

REFERENCE_SINGLE_NAME="${REFERENCE_SINGLE_NAME:-camera_gst_video2}"
REFERENCE_SINGLE_LAUNCH="${REFERENCE_SINGLE_LAUNCH:-/apollo/modules/drivers/camera_gst/launch/camera_gst_video2.launch}"
REFERENCE_SINGLE_PROCESS="${REFERENCE_SINGLE_PROCESS:-camera_gst_video2}"

TMP_DIR="$(mktemp -d)"
trap 'rm -rf "${TMP_DIR}"' EXIT

CYBER_LAUNCH_CMD=()

resolve_cyber_launch() {
  if [[ -x "${CYBER_LAUNCH_BIN}" ]]; then
    CYBER_LAUNCH_CMD=("${CYBER_LAUNCH_BIN}")
    return 0
  fi
  if [[ -x "${CYBER_LAUNCH_PY}" ]]; then
    CYBER_LAUNCH_CMD=(python3 "${CYBER_LAUNCH_PY}")
    return 0
  fi
  echo "missing cyber_launch executable and python entrypoint" >&2
  return 1
}

run_cyber_launch() {
  "${CYBER_LAUNCH_CMD[@]}" "$@"
}

set +u
source /apollo/cyber/setup.bash
set -u

resolve_cyber_launch

IFS=',' read -r -a COUNT_LIST <<< "${CAMERA_COUNTS}"
IFS=',' read -r -a NAME_LIST <<< "${CAMERA_NAMES}"
IFS=',' read -r -a URI_LIST <<< "${CAMERA_URIS}"

if [[ ${#NAME_LIST[@]} -lt 6 || ${#URI_LIST[@]} -lt 6 ]]; then
  echo "CAMERA_NAMES and CAMERA_URIS must provide at least 6 entries" >&2
  exit 2
fi

collect_proc_stats() {
  local pid="$1"
  local output_file="$2"
  : > "${output_file}"
  for _ in $(seq 1 "${BENCH_SECONDS}"); do
    [[ -d "/proc/${pid}" ]] || break
    local rss_kb
    local cpu_pct
    rss_kb="$(awk '/VmRSS/ {print $2}' "/proc/${pid}/status" 2>/dev/null || echo 0)"
    cpu_pct="$(ps -p "${pid}" -o %cpu= | awk '{print $1}' || echo 0)"
    printf '%s,%s,%s\n' "$(date +%s)" "${cpu_pct}" "${rss_kb}" >> "${output_file}"
    sleep 1
  done
}

collect_tegrastats() {
  local output_file="$1"
  if [[ -z "${TEGRATSTATS_BIN}" ]]; then
    echo "tegrastats_unavailable=1" > "${output_file}"
    return 0
  fi
  timeout "$((BENCH_SECONDS + 2))" "${TEGRATSTATS_BIN}" --interval 1000 > "${output_file}" 2>&1 || true
}

write_case_proto() {
  local case_count="$1"
  local proto_file="$2"
  : > "${proto_file}"
  for ((idx=0; idx<case_count; ++idx)); do
    cat >> "${proto_file}" <<EOF
sources {
  name: "${NAME_LIST[$idx]}"
  uri: "${URI_LIST[$idx]}"
  width: ${CAMERA_WIDTH}
  height: ${CAMERA_HEIGHT}
  fps: ${CAMERA_FPS}
  fourcc: "${CAMERA_FOURCC}"
  capture_backend: "${CAPTURE_BACKEND}"
  allow_backend_fallback: ${ALLOW_BACKEND_FALLBACK}
  publish {
    channel_name: "/apollo/sensor/camera/${NAME_LIST[$idx]}/image"
    frame_id: "${NAME_LIST[$idx]}"
    queue_capacity: 1
    output_format: "${OUTPUT_FORMAT}"
    output_width: 1280
    output_height: 720
    output_fps: 15.0
  }
}

layout_slots {
  source_name: "${NAME_LIST[$idx]}"
  row: $((idx / 2))
  col: $((idx % 2))
}

EOF
  done

  cat >> "${proto_file}" <<EOF
rows: $(((case_count + 1) / 2))
cols: 2
tile_width: 1280
tile_height: 720
fps: ${CAMERA_FPS}
publish_gpu_channel: false
validate_nvidia_plugins: true
zero_copy_required: false

platform {
  target: "jetson-orin"
  require_nvmm: true
  require_nvidia_plugins: true
}
EOF
}

write_case_dag() {
  local case_name="$1"
  local proto_file="$2"
  local dag_file="$3"
  cat > "${dag_file}" <<EOF
module_config {
  module_library: "/apollo/bazel-bin/modules/drivers/camera_gst/libcamera_gst_component.so"

  components {
    class_name: "CameraGstComponent"
    config {
      name: "${case_name}"
      config_file_path: "${proto_file}"
    }
  }
}
EOF
}

write_case_launch() {
  local case_name="$1"
  local dag_file="$2"
  local launch_file="$3"
  cat > "${launch_file}" <<EOF
<cyber>
  <module>
    <name>${case_name}</name>
    <dag_conf>${dag_file}</dag_conf>
    <process_name>${case_name}</process_name>
  </module>
</cyber>
EOF
}

stop_case() {
  local launch_file="$1"
  local process_name="$2"
  run_cyber_launch stop "${launch_file}" >/dev/null 2>&1 || true
  pkill -f "mainboard -d .*${process_name}" >/dev/null 2>&1 || true
}

wait_for_pid() {
  local process_name="$1"
  local pid=""
  for _ in $(seq 1 30); do
    pid="$(pgrep -f "mainboard -d .*${process_name} -p ${process_name}" | head -n1 || true)"
    if [[ -n "${pid}" ]]; then
      echo "${pid}"
      return 0
    fi
    sleep 1
  done
  return 1
}

wait_for_launch_log_pid() {
  local launch_log="$1"
  local pid=""
  for _ in $(seq 1 30); do
    pid="$(grep -oE 'pid: [0-9]+' "${launch_log}" 2>/dev/null | tail -n1 | awk '{print $2}' || true)"
    if [[ -n "${pid}" && -d "/proc/${pid}" ]]; then
      echo "${pid}"
      return 0
    fi
    sleep 1
  done
  return 1
}

summarize_case() {
  local proc_file="$1"
  local tegra_file="$2"
  local summary_file="$3"
  local case_count="$4"
  awk -F, -v count="${case_count}" '
    { cpu += $2; rss += $3; rows += 1 }
    END {
      printf("camera_count=%d\n", count)
      printf("avg_cpu_percent=%.2f\n", rows ? cpu / rows : 0)
      printf("avg_rss_mb=%.2f\n", rows ? rss / rows / 1024 : 0)
      printf("fps_notes=measure with recorder per source channel\n")
      printf("timestamp_skew_notes=collect from synchronized source timestamps\n")
      printf("drop_notes=use source_stats queue_drop_frames/cpu_drop_frames/gpu_drop_frames\n")
    }
  ' "${proc_file}" > "${summary_file}"
  if [[ -s "${tegra_file}" ]]; then
    cat "${tegra_file}" >> "${summary_file}"
  fi
}

run_existing_case() {
  local case_dir="$1"
  local launch_file="$2"
  local process_name="$3"
  local case_count="$4"

  stop_case "${launch_file}" "${process_name}"
  run_cyber_launch start "${launch_file}" > "${case_dir}/launch.log" 2>&1 &
  local launch_pid="$!"
  local module_pid
  module_pid="$(wait_for_pid "${process_name}" || true)"
  if [[ -z "${module_pid}" ]]; then
    module_pid="$(wait_for_launch_log_pid "${case_dir}/launch.log" || true)"
  fi
  if [[ -z "${module_pid}" ]]; then
    tail -n 120 "${case_dir}/launch.log" >&2 || true
    kill "${launch_pid}" >/dev/null 2>&1 || true
    wait "${launch_pid}" >/dev/null 2>&1 || true
    return 1
  fi

  sleep "${WARMUP_SECONDS}"
  collect_proc_stats "${module_pid}" "${case_dir}/proc_stats.csv" &
  local proc_pid="$!"
  collect_tegrastats "${case_dir}/tegrastats.log" &
  local tegra_pid="$!"
  wait "${proc_pid}" || true
  wait "${tegra_pid}" || true
  summarize_case "${case_dir}/proc_stats.csv" "${case_dir}/tegrastats.log" \
    "${case_dir}/summary.txt" "${case_count}"

  stop_case "${launch_file}" "${process_name}"
  kill "${launch_pid}" >/dev/null 2>&1 || true
  wait "${launch_pid}" >/dev/null 2>&1 || true
  return 0
}

mkdir -p "${RESULT_ROOT}"

for count in "${COUNT_LIST[@]}"; do
  case_name="camera_gst_multi_${count}"
  case_dir="${RESULT_ROOT}/${case_name}"
  mkdir -p "${case_dir}"

  if [[ "${count}" == "1" && "${USE_REFERENCE_SINGLE_CASE}" == "true" ]]; then
    run_existing_case "${case_dir}" "${REFERENCE_SINGLE_LAUNCH}" \
      "${REFERENCE_SINGLE_PROCESS}" "${count}"
    continue
  fi

  proto_file="${TMP_DIR}/${case_name}.pb.txt"
  dag_file="${TMP_DIR}/${case_name}.dag"
  launch_file="${TMP_DIR}/${case_name}.launch"

  write_case_proto "${count}" "${proto_file}"
  write_case_dag "${case_name}" "${proto_file}" "${dag_file}"
  write_case_launch "${case_name}" "${dag_file}" "${launch_file}"

  stop_case "${launch_file}" "${case_name}"
  run_cyber_launch start "${launch_file}" > "${case_dir}/launch.log" 2>&1 &
  launch_pid="$!"
  module_pid="$(wait_for_pid "${case_name}" || true)"
  if [[ -z "${module_pid}" ]]; then
    tail -n 120 "${case_dir}/launch.log" >&2 || true
    continue
  fi

  sleep "${WARMUP_SECONDS}"
  collect_proc_stats "${module_pid}" "${case_dir}/proc_stats.csv" &
  proc_pid="$!"
  collect_tegrastats "${case_dir}/tegrastats.log" &
  tegra_pid="$!"
  wait "${proc_pid}" || true
  wait "${tegra_pid}" || true
  summarize_case "${case_dir}/proc_stats.csv" "${case_dir}/tegrastats.log" \
    "${case_dir}/summary.txt" "${count}"

  stop_case "${launch_file}" "${case_name}"
  kill "${launch_pid}" >/dev/null 2>&1 || true
  wait "${launch_pid}" >/dev/null 2>&1 || true
done

echo "multi-camera benchmark summaries written to ${RESULT_ROOT}"
