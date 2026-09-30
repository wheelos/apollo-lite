#!/usr/bin/env bash

set -euo pipefail

CAMERA_COUNTS="${CAMERA_COUNTS:-1,3}"
BENCH_SECONDS="${BENCH_SECONDS:-30}"
WARMUP_SECONDS="${WARMUP_SECONDS:-10}"
RESULT_ROOT="${RESULT_ROOT:-/apollo/data/benchmarks/camera_gst_multi}"
CYBER_LAUNCH_BIN="${CYBER_LAUNCH_BIN:-/apollo/bazel-bin/cyber/tools/cyber_launch/cyber_launch}"
CYBER_LAUNCH_PY="${CYBER_LAUNCH_PY:-/apollo/cyber/tools/cyber_launch/cyber_launch.py}"
TEGRATSTATS_BIN="${TEGRATSTATS_BIN:-$(command -v tegrastats || true)}"

CAMERA_NAMES_DEFAULT="video1,video2,video3"
CAMERA_URIS_DEFAULT="/dev/video1,/dev/video2,/dev/video3"
CAMERA_NAMES="${CAMERA_NAMES:-${CAMERA_NAMES_DEFAULT}}"
CAMERA_URIS="${CAMERA_URIS:-${CAMERA_URIS_DEFAULT}}"
CAMERA_FOURCC="${CAMERA_FOURCC:-UYVY}"
CAMERA_WIDTH="${CAMERA_WIDTH:-1920}"
CAMERA_HEIGHT="${CAMERA_HEIGHT:-1080}"
CAMERA_FPS="${CAMERA_FPS:-30.0}"
CAPTURE_BACKEND="${CAPTURE_BACKEND:-NVV4L2_DMABUF}"

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

max_count=0
for count in "${COUNT_LIST[@]}"; do
  ((count > max_count)) && max_count="${count}"
done
if [[ ${#NAME_LIST[@]} -lt ${max_count} || ${#URI_LIST[@]} -lt ${max_count} ]]; then
  echo "CAMERA_NAMES and CAMERA_URIS must cover the largest camera count" >&2
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
}

EOF
  done

  cat >> "${proto_file}" <<EOF
fps: ${CAMERA_FPS}
publish_gpu_channel: true
validate_nvidia_plugins: true
zero_copy_required: true

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
  pkill -f "mainboard.*-p ${process_name}" >/dev/null 2>&1 || true
}

wait_for_pid() {
  local process_name="$1"
  local launch_log="${2:-}"
  local pid=""
  for _ in $(seq 1 30); do
    pid="$(pgrep -f "mainboard.*-p ${process_name}" | head -n1 || true)"
    if [[ -n "${pid}" ]]; then
      echo "${pid}"
      return 0
    fi
    if [[ -n "${launch_log}" && -f "${launch_log}" ]]; then
      pid="$(grep -oE 'pid: [0-9]+' "${launch_log}" 2>/dev/null | tail -n1 | awk '{print $2}' || true)"
      if [[ -n "${pid}" && -d "/proc/${pid}" ]]; then
        echo "${pid}"
        return 0
      fi
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
      printf("drop_notes=use source_stats gpu_drop_frames\n")
    }
  ' "${proc_file}" > "${summary_file}"
  if [[ -s "${tegra_file}" ]]; then
    cat "${tegra_file}" >> "${summary_file}"
  fi
}

mkdir -p "${RESULT_ROOT}"

for count in "${COUNT_LIST[@]}"; do
  case_name="camera_gst_multi_${count}"
  case_dir="${RESULT_ROOT}/${case_name}"
  mkdir -p "${case_dir}"

  proto_file="${TMP_DIR}/${case_name}.pb.txt"
  dag_file="${TMP_DIR}/${case_name}.dag"
  launch_file="${TMP_DIR}/${case_name}.launch"

  write_case_proto "${count}" "${proto_file}"
  write_case_dag "${case_name}" "${proto_file}" "${dag_file}"
  write_case_launch "${case_name}" "${dag_file}" "${launch_file}"

  stop_case "${launch_file}" "${case_name}"
  run_cyber_launch start "${launch_file}" > "${case_dir}/launch.log" 2>&1 &
  launch_pid="$!"
  module_pid="$(wait_for_pid "${case_name}" "${case_dir}/launch.log" || true)"
  if [[ -z "${module_pid}" ]]; then
    tail -n 120 "${case_dir}/launch.log" >&2 || true
    stop_case "${launch_file}" "${case_name}"
    kill "${launch_pid}" >/dev/null 2>&1 || true
    wait "${launch_pid}" >/dev/null 2>&1 || true
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
