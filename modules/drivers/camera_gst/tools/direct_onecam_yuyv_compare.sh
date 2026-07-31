#!/usr/bin/env bash

set -euo pipefail

set +u
source /apollo/cyber/setup.bash
set -u

RESULT_ROOT="${RESULT_ROOT:-/apollo/data/benchmarks/direct_onecam_yuyv}"
BENCH_SECONDS="${BENCH_SECONDS:-6}"
WARMUP_SECONDS="${WARMUP_SECONDS:-2}"
CYBER_RECORDER="/apollo/bazel-bin/cyber/tools/cyber_recorder/cyber_recorder"
CYBER_LAUNCH_PY="/apollo/cyber/tools/cyber_launch/cyber_launch.py"

mkdir -p "${RESULT_ROOT}"

wait_log_pid() {
  local log_file="$1"
  local pid=""
  for _ in $(seq 1 30); do
    pid="$(grep -oE 'pid: [0-9]+' "${log_file}" 2>/dev/null | tail -n1 | awk '{print $2}' || true)"
    if [[ -n "${pid}" && -d "/proc/${pid}" ]]; then
      echo "${pid}"
      return 0
    fi
    sleep 1
  done
  return 1
}

read_io() {
  local pid="$1"
  local key="$2"
  awk -v k="${key}:" '$1 == k { print $2; exit }' "/proc/${pid}/io" 2>/dev/null || echo 0
}

summarize_proc() {
  local proc_csv="$1"
  local fps="$2"
  local io_start_read="$3"
  local io_end_read="$4"
  local io_start_write="$5"
  local io_end_write="$6"
  local output_file="$7"

  awk -F, -v fps="${fps}" -v rs="${io_start_read}" -v re="${io_end_read}" \
      -v ws="${io_start_write}" -v we="${io_end_write}" '
    { cpu += $2; rss += $3; rows += 1 }
    END {
      printf("avg_cpu_percent=%.2f\n", rows ? cpu / rows : 0)
      printf("avg_rss_mb=%.2f\n", rows ? rss / rows / 1024 : 0)
      printf("avg_fps=%.2f\n", fps)
      printf("io_read_mb=%.2f\n", ((re - rs) > 0 ? (re - rs) : 0) / 1024 / 1024)
      printf("io_write_mb=%.2f\n", ((we - ws) > 0 ? (we - ws) : 0) / 1024 / 1024)
    }
  ' "${proc_csv}" > "${output_file}"
}

sample_proc() {
  local pid="$1"
  local proc_csv="$2"
  : > "${proc_csv}"
  for _ in $(seq 1 "${BENCH_SECONDS}"); do
    [[ -d "/proc/${pid}" ]] || break
    printf '%s,%s,%s\n' \
      "$(date +%s)" \
      "$(ps -p "${pid}" -o %cpu= | awk '{print $1}' || echo 0)" \
      "$(awk '/VmRSS/ {print $2}' "/proc/${pid}/status" 2>/dev/null || echo 0)" \
      >> "${proc_csv}"
    sleep 1
  done
}

run_legacy() {
  local dir="${RESULT_ROOT}/legacy"
  rm -rf "${dir}"
  mkdir -p "${dir}"

  python3 "${CYBER_LAUNCH_PY}" stop /apollo/modules/drivers/camera_gst/tools/baseline/legacy_camera_video2_yuyv.launch >/dev/null 2>&1 || true
  pkill -f 'mainboard -d /apollo/modules/drivers/camera_gst/tools/baseline/legacy_camera_video2_yuyv.dag' >/dev/null 2>&1 || true

  python3 "${CYBER_LAUNCH_PY}" start /apollo/modules/drivers/camera_gst/tools/baseline/legacy_camera_video2_yuyv.launch > "${dir}/launch.log" 2>&1 &
  local launch_pid="$!"
  local pid
  pid="$(wait_log_pid "${dir}/launch.log")"
  echo "${pid}" > "${dir}/pid"

  local io_start_read io_start_write io_end_read io_end_write
  io_start_read="$(read_io "${pid}" read_bytes)"
  io_start_write="$(read_io "${pid}" write_bytes)"

  sleep "${WARMUP_SECONDS}"
  sample_proc "${pid}" "${dir}/proc.csv" &
  local sampler_pid="$!"
  timeout "${BENCH_SECONDS}" "${CYBER_RECORDER}" record -c /apollo/sensor/camera/video2_legacy_yuyv/image -o "${dir}/fps" > "${dir}/record.log" 2>&1 || true
  wait "${sampler_pid}" || true

  io_end_read="$(read_io "${pid}" read_bytes)"
  io_end_write="$(read_io "${pid}" write_bytes)"

  python3 "${CYBER_LAUNCH_PY}" stop /apollo/modules/drivers/camera_gst/tools/baseline/legacy_camera_video2_yuyv.launch >/dev/null 2>&1 || true
  kill "${launch_pid}" >/dev/null 2>&1 || true
  wait "${launch_pid}" >/dev/null 2>&1 || true

  local record_file fps count
  record_file="$(find "${dir}" -maxdepth 1 -type f -name 'fps*' | head -n1 || true)"
  fps="0"
  if [[ -n "${record_file}" ]]; then
    count="$("${CYBER_RECORDER}" info "${record_file}" 2>/dev/null | awk '/message count/ {print $3; exit}' || true)"
    count="${count:-0}"
    if [[ "${count}" == "0" ]]; then
      count="$(grep -oE 'Progress: [0-9]+ channels, [0-9]+ messages' "${dir}/record.log" 2>/dev/null | tail -n1 | awk '{print $4}' || true)"
      count="${count:-0}"
    fi
    fps="$(awk -v c="${count}" -v s="${BENCH_SECONDS}" 'BEGIN { printf("%.2f", s ? c / s : 0) }')"
  fi

  summarize_proc "${dir}/proc.csv" "${fps}" "${io_start_read}" "${io_end_read}" \
    "${io_start_write}" "${io_end_write}" "${dir}/summary.txt"
}

run_camera_gst() {
  local dir="${RESULT_ROOT}/camera_gst"
  rm -rf "${dir}"
  mkdir -p "${dir}"

  python3 "${CYBER_LAUNCH_PY}" stop /apollo/modules/drivers/camera_gst/launch/camera_gst_video2_yuyv.launch >/dev/null 2>&1 || true
  pkill -f 'mainboard -d /apollo/modules/drivers/camera_gst/dag/camera_gst_video2_yuyv.dag' >/dev/null 2>&1 || true

  python3 "${CYBER_LAUNCH_PY}" start /apollo/modules/drivers/camera_gst/launch/camera_gst_video2_yuyv.launch > "${dir}/launch.log" 2>&1 &
  local launch_pid="$!"
  local pid
  pid="$(wait_log_pid "${dir}/launch.log")"
  echo "${pid}" > "${dir}/pid"

  local io_start_read io_start_write io_end_read io_end_write
  io_start_read="$(read_io "${pid}" read_bytes)"
  io_start_write="$(read_io "${pid}" write_bytes)"

  sleep "${WARMUP_SECONDS}"
  sample_proc "${pid}" "${dir}/proc.csv"

  io_end_read="$(read_io "${pid}" read_bytes)"
  io_end_write="$(read_io "${pid}" write_bytes)"

  python3 "${CYBER_LAUNCH_PY}" stop /apollo/modules/drivers/camera_gst/launch/camera_gst_video2_yuyv.launch >/dev/null 2>&1 || true
  kill "${launch_pid}" >/dev/null 2>&1 || true
  wait "${launch_pid}" >/dev/null 2>&1 || true

  local component_log frames fps
  component_log="$(find /apollo/data/log -maxdepth 1 -type f -name "camera_gst.log.INFO.*.${pid}" -print -quit || true)"
  if [[ -n "${component_log}" ]]; then
    cp "${component_log}" "${dir}/component.log"
  fi
  frames="$(grep -oE 'camera_gst gpu frames observed=[0-9]+' "${dir}/component.log" 2>/dev/null | tail -n1 | awk -F= '{print $2}' || true)"
  frames="${frames:-0}"
  fps="$(awk -v c="${frames}" -v s="$((BENCH_SECONDS + WARMUP_SECONDS))" 'BEGIN { printf("%.2f", s ? c / s : 0) }')"

  summarize_proc "${dir}/proc.csv" "${fps}" "${io_start_read}" "${io_end_read}" \
    "${io_start_write}" "${io_end_write}" "${dir}/summary.txt"
}

run_legacy
run_camera_gst

echo '===legacy==='
cat "${RESULT_ROOT}/legacy/summary.txt"
echo '===camera_gst==='
cat "${RESULT_ROOT}/camera_gst/summary.txt"
