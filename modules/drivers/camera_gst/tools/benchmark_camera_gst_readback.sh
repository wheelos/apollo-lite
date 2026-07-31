#!/usr/bin/env bash

set -euo pipefail

FORMAT="${FORMAT:-yuyv}"
BENCH_SECONDS="${BENCH_SECONDS:-30}"
WARMUP_SECONDS="${WARMUP_SECONDS:-10}"
RESULT_ROOT="${RESULT_ROOT:-/apollo/data/benchmarks/camera_gst_readback}"
CYBER_LAUNCH_BIN="${CYBER_LAUNCH_BIN:-/apollo/bazel-bin/cyber/tools/cyber_launch/cyber_launch}"
CYBER_LAUNCH_PY="${CYBER_LAUNCH_PY:-/apollo/cyber/tools/cyber_launch/cyber_launch.py}"
BASELINE_SCRIPT="/apollo/modules/drivers/camera_gst/tools/benchmark_camera_format_baseline.sh"

case "${FORMAT}" in
  yuyv)
    READBACK_LAUNCH="/apollo/modules/drivers/camera_gst/launch/camera_gst_video2_yuyv_readback.launch"
    READBACK_DAG="/apollo/modules/drivers/camera_gst/dag/camera_gst_video2_yuyv_readback.dag"
    READBACK_PROCESS="camera_gst_video2_yuyv_readback"
    ;;
  rgb)
    READBACK_LAUNCH="/apollo/modules/drivers/camera_gst/launch/camera_gst_video2_rgb_readback.launch"
    READBACK_DAG="/apollo/modules/drivers/camera_gst/dag/camera_gst_video2_rgb_readback.dag"
    READBACK_PROCESS="camera_gst_video2_rgb_readback"
    ;;
  *)
    echo "FORMAT must be yuyv or rgb, got: ${FORMAT}" >&2
    exit 2
    ;;
esac

RESULT_DIR="${RESULT_ROOT}/${FORMAT}"
PUBLISH_BASELINE_ROOT="${RESULT_DIR}/publish_baseline"
READBACK_DIR="${RESULT_DIR}/camera_gst_readback"
mkdir -p "${RESULT_DIR}" "${READBACK_DIR}"

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

stop_readback() {
  run_cyber_launch stop "${READBACK_LAUNCH}" >/dev/null 2>&1 || true
  pkill -f "mainboard -d ${READBACK_DAG}" >/dev/null 2>&1 || true
}
trap stop_readback EXIT

wait_for_pid() {
  local pid=""
  for _ in $(seq 1 30); do
    pid="$(pgrep -f "mainboard -d ${READBACK_DAG} -p ${READBACK_PROCESS}" | head -n1 || true)"
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
  for sample in $(seq 1 "${BENCH_SECONDS}"); do
    [[ -d "/proc/${pid}" ]] || break
    local rss_kb
    local cpu_pct
    rss_kb="$(awk '/VmRSS/ {print $2}' "/proc/${pid}/status" 2>/dev/null || echo 0)"
    cpu_pct="$(ps -p "${pid}" -o %cpu= | awk '{print $1}' || echo 0)"
    printf '%s,%s,%s\n' "$(date +%s)" "${cpu_pct}" "${rss_kb}" >> "${output_file}"
    echo "${FORMAT} readback ${sample}/${BENCH_SECONDS}: pid=${pid} cpu=${cpu_pct}% rss_kb=${rss_kb}"
    sleep 1
  done
}

run_publish_baseline() {
  RESULT_ROOT="${PUBLISH_BASELINE_ROOT}" FORMAT="${FORMAT}" \
    BENCH_SECONDS="${BENCH_SECONDS}" WARMUP_SECONDS="${WARMUP_SECONDS}" \
    CYBER_LAUNCH_BIN="${CYBER_LAUNCH_BIN}" CYBER_LAUNCH_PY="${CYBER_LAUNCH_PY}" \
    bash "${BASELINE_SCRIPT}"
}

run_readback() {
  stop_readback
  run_cyber_launch start "${READBACK_LAUNCH}" > "${READBACK_DIR}/launch.log" 2>&1 &
  local launch_pid="$!"
  local module_pid
  module_pid="$(wait_for_pid || true)"
  if [[ -z "${module_pid}" ]]; then
    tail -n 120 "${READBACK_DIR}/launch.log" >&2 || true
    return 1
  fi

  echo "${module_pid}" > "${READBACK_DIR}/module.pid"
  sleep "${WARMUP_SECONDS}"
  collect_proc_stats "${module_pid}" "${READBACK_DIR}/proc_stats.csv"
  run_cyber_launch stop "${READBACK_LAUNCH}" >/dev/null 2>&1 || true
  kill "${launch_pid}" >/dev/null 2>&1 || true
  wait "${launch_pid}" >/dev/null 2>&1 || true

  local component_log
  component_log="$(find "${GLOG_log_dir}" -maxdepth 1 -type f \
    -name "camera_gst.log.INFO.*.${module_pid}" -print -quit)"
  if [[ -z "${component_log}" ]]; then
    echo "missing camera_gst log for readback process ${module_pid}" >&2
    return 1
  fi

  local observed_frames
  observed_frames="$(grep -oE 'source readback consumed=[0-9]+' "${component_log}" | \
    tail -n1 | awk -F= '{print $2}' || true)"
  observed_frames="${observed_frames:-0}"
  awk -F, -v frames="${observed_frames}" -v warmup="${WARMUP_SECONDS}" \
    -v seconds="${BENCH_SECONDS}" '
    { cpu += $2; rss += $3; rows += 1 }
    END {
      printf("avg_cpu_percent=%.2f\n", rows ? cpu / rows : 0)
      printf("avg_rss_mb=%.2f\n", rows ? rss / rows / 1024 : 0)
      printf("observed_frames=%d\n", frames)
      printf("observed_fps=%.2f\n", (warmup + seconds) ? frames / (warmup + seconds) : 0)
    }
  ' "${READBACK_DIR}/proc_stats.csv" > "${READBACK_DIR}/summary.txt"
}

run_publish_baseline
run_readback

python3 - "${PUBLISH_BASELINE_ROOT}/${FORMAT}" "${READBACK_DIR}" "${RESULT_DIR}" <<'PY'
import pathlib
import sys

baseline_dir = pathlib.Path(sys.argv[1])
readback_dir = pathlib.Path(sys.argv[2])
result_dir = pathlib.Path(sys.argv[3])


def read_metrics(path):
    return dict(line.strip().split("=", 1) for line in path.read_text().splitlines())


legacy = read_metrics(next(baseline_dir.glob("legacy_camera_*/summary.txt")))
publish = read_metrics(next(baseline_dir.glob("camera_gst_*/summary.txt")))
readback = read_metrics(readback_dir / "summary.txt")
report = result_dir / "comparison_report.txt"
with report.open("w", encoding="utf-8") as out:
    out.write("camera_gst GPU-to-CPU readback benchmark\n")
    out.write("====================================\n\n")
    for name, metrics, fps_key in (
        ("legacy_camera_channel", legacy, "avg_fps"),
        ("camera_gst_channel", publish, "avg_fps"),
        ("camera_gst_readback_no_channel", readback, "observed_fps"),
    ):
        out.write(f"[{name}]\n")
        out.write(f"avg_cpu_percent={float(metrics['avg_cpu_percent']):.2f}\n")
        out.write(f"avg_rss_mb={float(metrics['avg_rss_mb']):.2f}\n")
        out.write(f"fps={float(metrics[fps_key]):.2f}\n")
        if name == "camera_gst_readback_no_channel":
            out.write(f"observed_frames={int(metrics['observed_frames'])}\n")
        out.write("\n")
    out.write("[delta: camera_gst_channel - readback_no_channel]\n")
    out.write(
        f"avg_cpu_percent={float(publish['avg_cpu_percent']) - float(readback['avg_cpu_percent']):.2f}\n"
    )
    out.write(
        f"avg_rss_mb={float(publish['avg_rss_mb']) - float(readback['avg_rss_mb']):.2f}\n"
    )
PY

echo "readback comparison report written to ${RESULT_DIR}/comparison_report.txt"
