#!/usr/bin/env bash

set -euo pipefail

GST_CASE_NAME="${GST_CASE_NAME:-camera_gst_video2}"
GST_LAUNCH="${GST_LAUNCH:-/apollo/modules/drivers/camera_gst/launch/camera_gst_video2.launch}"
GST_DAG="${GST_DAG:-/apollo/modules/drivers/camera_gst/dag/camera_gst_video2.dag}"
GST_PROCESS_PATTERN="${GST_PROCESS_PATTERN:-mainboard -d /apollo/modules/drivers/camera_gst/dag/camera_gst_video2.dag -p camera_gst_video2}"
GST_CHANNEL="${GST_CHANNEL:-/apollo/sensor/camera/video2/image}"

LEGACY_CASE_NAME="${LEGACY_CASE_NAME:-camera_local_video2}"
LEGACY_LAUNCH="${LEGACY_LAUNCH:-/apollo/modules/drivers/camera/launch/camera_local_video2.launch}"
LEGACY_DAG="${LEGACY_DAG:-/apollo/modules/drivers/camera/dag/camera.local.video2.dag}"
LEGACY_PROCESS_PATTERN="${LEGACY_PROCESS_PATTERN:-mainboard -d /apollo/modules/drivers/camera/dag/camera.local.video2.dag -p camera_video2}"
LEGACY_CHANNEL="${LEGACY_CHANNEL:-/apollo/sensor/camera/video2_legacy/image}"

BENCH_SECONDS="${BENCH_SECONDS:-30}"
WARMUP_SECONDS="${WARMUP_SECONDS:-10}"
RESULT_DIR="${RESULT_DIR:-/apollo/data/benchmarks/camera_vs_camera_gst}"
CYBER_LAUNCH_BIN="${CYBER_LAUNCH_BIN:-/apollo/bazel-bin/cyber/tools/cyber_launch/cyber_launch}"
CYBER_RECORDER_BIN="${CYBER_RECORDER_BIN:-/apollo/bazel-bin/cyber/tools/cyber_recorder/cyber_recorder}"
CYBER_LAUNCH_PY="${CYBER_LAUNCH_PY:-/apollo/cyber/tools/cyber_launch/cyber_launch.py}"
TEGRATSTATS_BIN="${TEGRATSTATS_BIN:-$(command -v tegrastats || true)}"

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

resolve_cyber_recorder() {
	if [[ -x "${CYBER_RECORDER_BIN}" ]]; then
		return 0
	fi
	local recorder_in_path
	recorder_in_path="$(command -v cyber_recorder || true)"
	if [[ -n "${recorder_in_path}" && -x "${recorder_in_path}" ]]; then
		CYBER_RECORDER_BIN="${recorder_in_path}"
		return 0
	fi
	CYBER_RECORDER_BIN=""
	return 0
}

run_cyber_launch() {
	"${CYBER_LAUNCH_CMD[@]}" "$@"
}

resolve_cyber_launch
resolve_cyber_recorder

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

collect_tegrastats() {
	local output_file="$1"
	if [[ -z "${TEGRATSTATS_BIN}" ]]; then
		echo "tegrastats_unavailable=1" > "${output_file}"
		return 0
	fi
	timeout "$((BENCH_SECONDS + 2))" "${TEGRATSTATS_BIN}" --interval 1000 > "${output_file}" 2>&1 || true
}

record_channel() {
	local channel="$1"
	local output_prefix="$2"
	if [[ ! -x "${CYBER_RECORDER_BIN}" ]]; then
		return 1
	fi
	timeout "${BENCH_SECONDS}" "${CYBER_RECORDER_BIN}" record -c "${channel}" -o "${output_prefix}" >/dev/null 2>&1 || true
}

extract_avg_fps() {
	local channel="$1"
	local record_prefix="$2"
	local record_log="$(dirname "${record_prefix}")/record.log"
	if [[ ! -x "${CYBER_RECORDER_BIN}" ]]; then
		echo "0"
		return 0
	fi
	local record_file
	record_file="$(find "$(dirname "${record_prefix}")" -maxdepth 1 -type f -name "$(basename "${record_prefix}")*" | head -n1 || true)"
	if [[ -z "${record_file}" ]]; then
		echo "0"
		return 0
	fi
	local message_count
	message_count="$("${CYBER_RECORDER_BIN}" info "${record_file}" 2>/dev/null | awk -v channel="${channel}" '
		$0 ~ channel {found=1}
		found && /message count/ {print $3; exit}
	' || true)"
	message_count="${message_count:-0}"
	if [[ "${message_count}" == "0" && -f "${record_log}" ]]; then
		message_count="$(grep -oE 'Progress: [0-9]+ channels, [0-9]+ messages' "${record_log}" 2>/dev/null | tail -n1 | awk '{print $4}' || true)"
		message_count="${message_count:-0}"
	fi
	awk -v count="${message_count}" -v seconds="${BENCH_SECONDS}" 'BEGIN { printf("%.2f", seconds ? count / seconds : 0) }'
}

write_summary() {
	local proc_file="$1"
	local tegra_file="$2"
	local avg_fps="$3"
	local summary_file="$4"
	awk -F, -v fps="${avg_fps}" '
		{ cpu += $2; rss += $3; rows += 1 }
		END {
			printf("avg_cpu_percent=%.2f\n", rows ? cpu / rows : 0)
			printf("avg_rss_mb=%.2f\n", rows ? rss / rows / 1024 : 0)
			printf("avg_fps=%.2f\n", fps)
		}
	' "${proc_file}" > "${summary_file}"
	if [[ -s "${tegra_file}" ]]; then
		cat "${tegra_file}" >> "${summary_file}"
	fi
}

run_case() {
	local case_name="$1"
	local launch_file="$2"
	local process_pattern="$3"
	local channel="$4"

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
	collect_tegrastats "${case_dir}/tegrastats.log" &
	local tegra_pid="$!"
	record_channel "${channel}" "${case_dir}/record" || true
	wait "${proc_pid}" || true
	wait "${tegra_pid}" || true
	local avg_fps
	avg_fps="$(extract_avg_fps "${channel}" "${case_dir}/record")"
	write_summary "${case_dir}/proc_stats.csv" "${case_dir}/tegrastats.log" "${avg_fps}" "${case_dir}/summary.txt"

	stop_case "${launch_file}" "${process_pattern}"
	kill "${launch_pid}" >/dev/null 2>&1 || true
	wait "${launch_pid}" >/dev/null 2>&1 || true
	trap - RETURN
}

run_case "${LEGACY_CASE_NAME}" "${LEGACY_LAUNCH}" "${LEGACY_PROCESS_PATTERN}" "${LEGACY_CHANNEL}"
run_case "${GST_CASE_NAME}" "${GST_LAUNCH}" "${GST_PROCESS_PATTERN}" "${GST_CHANNEL}"

python3 - "${RESULT_DIR}" "${LEGACY_CASE_NAME}" "${GST_CASE_NAME}" <<'PY'
import pathlib
import sys

result_dir = pathlib.Path(sys.argv[1])
legacy_name = sys.argv[2]
gst_name = sys.argv[3]


def read_metrics(path):
		metrics = {}
		for line in path.read_text().splitlines():
				if "=" not in line:
						continue
				key, value = line.split("=", 1)
				metrics[key.strip()] = value.strip()
		return metrics


legacy = read_metrics(result_dir / legacy_name / "summary.txt")
gst = read_metrics(result_dir / gst_name / "summary.txt")
report = result_dir / "comparison_report.txt"
with report.open("w", encoding="utf-8") as out:
		out.write("camera_vs_camera_gst benchmark\n")
		out.write("=============================\n\n")
		for name, metrics in ((legacy_name, legacy), (gst_name, gst)):
				out.write(f"[{name}]\n")
				for key in ("avg_cpu_percent", "avg_rss_mb", "avg_fps"):
						out.write(f"{key}={metrics.get(key, '0')}\n")
				out.write("\n")
		out.write(f"[delta: {gst_name} - {legacy_name}]\n")
		for key in ("avg_cpu_percent", "avg_rss_mb", "avg_fps"):
				delta = float(gst.get(key, "0")) - float(legacy.get(key, "0"))
				out.write(f"{key}={delta:.2f}\n")
PY

echo "comparison report written to ${RESULT_DIR}/comparison_report.txt"
