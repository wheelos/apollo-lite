#!/usr/bin/env bash

set -euo pipefail

BASE_CASE_NAME="${BASE_CASE_NAME:-camera_gst_video2}"
BASE_LAUNCH="${BASE_LAUNCH:-/apollo/modules/drivers/camera_gst/launch/camera_gst_video2.launch}"
BASE_PROCESS_PATTERN="${BASE_PROCESS_PATTERN:-mainboard -d /apollo/modules/drivers/camera_gst/dag/camera_gst_video2.dag -p camera_gst_video2}"

DUAL_CASE_NAME="${DUAL_CASE_NAME:-camera_gst_video2_dual_role}"
DUAL_LAUNCH="${DUAL_LAUNCH:-/apollo/modules/drivers/camera_gst/launch/camera_gst_video2_dual_role.launch}"
DUAL_PROCESS_PATTERN="${DUAL_PROCESS_PATTERN:-mainboard -d /apollo/modules/drivers/camera_gst/dag/camera_gst_video2_dual_role.dag -p camera_gst_video2_dual_role}"

BENCH_SECONDS="${BENCH_SECONDS:-20}"
WARMUP_SECONDS="${WARMUP_SECONDS:-5}"
RESULT_DIR="${RESULT_DIR:-/apollo/data/benchmarks/camera_gst_dual_role_baseline}"
CYBER_LAUNCH_BIN="${CYBER_LAUNCH_BIN:-/apollo/bazel-bin/cyber/tools/cyber_launch/cyber_launch}"
CYBER_LAUNCH_PY="${CYBER_LAUNCH_PY:-/apollo/cyber/tools/cyber_launch/cyber_launch.py}"

set +u
source /apollo/cyber/setup.bash
set -u

mkdir -p "${RESULT_DIR}"

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

stop_case() {
  local launch_file="$1"
  local process_pattern="$2"
  run_cyber_launch stop "${launch_file}" >/dev/null 2>&1 || true
  pkill -f "${process_pattern}" >/dev/null 2>&1 || true
}

wait_for_pid() {
  local process_pattern="$1"
  local pid=""
  for _ in $(seq 1 30); do
    pid="$(pgrep -f "${process_pattern}" | head -n1 || true)"
    if [[ -n "${pid}" ]]; then
      echo "${pid}"
      return 0
    fi
    sleep 1
  done
  return 1
}

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

write_summary() {
  local proc_file="$1"
  local summary_file="$2"
  awk -F, '
    { cpu += $2; rss += $3; rows += 1 }
    END {
      printf("avg_cpu_percent=%.2f\n", rows ? cpu / rows : 0)
      printf("avg_rss_mb=%.2f\n", rows ? rss / rows / 1024 : 0)
    }
  ' "${proc_file}" > "${summary_file}"
}

verify_correctness() {
  local log_file="$1"
  local output_file="$2"
  local expect_nvenc="$3"
  {
    echo "correctness_checks"
    echo "=================="
    if [[ "${expect_nvenc}" == "1" ]]; then
      if grep -q "NvVideo: NVENC" "${log_file}"; then
        echo "nvenc_init=pass"
      else
        echo "nvenc_init=fail"
      fi
    else
      echo "nvenc_init=not_required"
    fi
    if grep -q "zero-copy GPU frame extraction failed" "${log_file}"; then
      echo "gpu_publish_path=fail"
    else
      echo "gpu_publish_path=pass"
    fi
    if grep -qE "GStreamer error|Segmentation fault|has died|not-linked|not-negotiated" "${log_file}"; then
      echo "runtime_stability=fail"
    else
      echo "runtime_stability=pass"
    fi
  } > "${output_file}"
}

run_case() {
  local case_name="$1"
  local launch_file="$2"
  local process_pattern="$3"
  local expect_nvenc="$4"

  local case_dir="${RESULT_DIR}/${case_name}"
  rm -rf "${case_dir}"
  mkdir -p "${case_dir}"

  stop_case "${launch_file}" "${process_pattern}"
  trap 'stop_case "'"${launch_file}"'" "'"${process_pattern}"'"' RETURN

  run_cyber_launch start "${launch_file}" > "${case_dir}/launch.log" 2>&1 &
  local launch_pid="$!"
  local module_pid
  module_pid="$(wait_for_pid "${process_pattern}" || true)"
  if [[ -z "${module_pid}" ]]; then
    tail -n 120 "${case_dir}/launch.log" >&2 || true
    return 1
  fi

  sleep "${WARMUP_SECONDS}"
  collect_proc_stats "${module_pid}" "${case_dir}/proc_stats.csv" &
  local proc_pid="$!"
  sleep "${BENCH_SECONDS}"
  wait "${proc_pid}" || true

  stop_case "${launch_file}" "${process_pattern}"
  kill "${launch_pid}" >/dev/null 2>&1 || true
  wait "${launch_pid}" >/dev/null 2>&1 || true
  trap - RETURN

  write_summary "${case_dir}/proc_stats.csv" "${case_dir}/summary.txt"
  verify_correctness "${case_dir}/launch.log" "${case_dir}/correctness.txt" "${expect_nvenc}"
}

resolve_cyber_launch
run_case "${BASE_CASE_NAME}" "${BASE_LAUNCH}" "${BASE_PROCESS_PATTERN}" 0
run_case "${DUAL_CASE_NAME}" "${DUAL_LAUNCH}" "${DUAL_PROCESS_PATTERN}" 1

python3 - "${RESULT_DIR}" "${BASE_CASE_NAME}" "${DUAL_CASE_NAME}" <<'PY'
import pathlib
import sys

result_dir = pathlib.Path(sys.argv[1])
base_name = sys.argv[2]
dual_name = sys.argv[3]


def read_metrics(path):
    metrics = {}
    for line in path.read_text().splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        metrics[key.strip()] = value.strip()
    return metrics

base = read_metrics(result_dir / base_name / "summary.txt")
dual = read_metrics(result_dir / dual_name / "summary.txt")
report = result_dir / "comparison_report.txt"
with report.open("w", encoding="utf-8") as out:
    out.write("camera_gst dual role baseline\n")
    out.write("=============================\n\n")
    for name, metrics in ((base_name, base), (dual_name, dual)):
        out.write(f"[{name}]\n")
        for key in ("avg_cpu_percent", "avg_rss_mb"):
            out.write(f"{key}={metrics.get(key, '0')}\n")
        out.write("\n")
    out.write(f"[delta: {dual_name} - {base_name}]\n")
    for key in ("avg_cpu_percent", "avg_rss_mb"):
        delta = float(dual.get(key, "0")) - float(base.get(key, "0"))
        out.write(f"{key}={delta:.2f}\n")
PY

echo "dual-role baseline report written to ${RESULT_DIR}/comparison_report.txt"
