#!/usr/bin/env bash

set -euo pipefail

FORMAT="${FORMAT:-yuyv}"
BENCH_SECONDS="${BENCH_SECONDS:-30}"
WARMUP_SECONDS="${WARMUP_SECONDS:-10}"
RESULT_ROOT="${RESULT_ROOT:-/apollo/data/benchmarks/camera_format_baseline}"
CYBER_LAUNCH_BIN="${CYBER_LAUNCH_BIN:-/apollo/bazel-bin/cyber/tools/cyber_launch/cyber_launch}"
CYBER_RECORDER_BIN="${CYBER_RECORDER_BIN:-/apollo/bazel-bin/cyber/tools/cyber_recorder/cyber_recorder}"
CYBER_LAUNCH_PY="${CYBER_LAUNCH_PY:-/apollo/cyber/tools/cyber_launch/cyber_launch.py}"

case "${FORMAT}" in
	yuyv)
		LEGACY_NAME="legacy_camera_video2_yuyv"
		LEGACY_LAUNCH="/apollo/modules/drivers/camera_gst/tools/baseline/legacy_camera_video2_yuyv.launch"
		LEGACY_DAG="/apollo/modules/drivers/camera_gst/tools/baseline/legacy_camera_video2_yuyv.dag"
		LEGACY_PROCESS="legacy_camera_video2_yuyv"
		LEGACY_CHANNEL="/apollo/sensor/camera/video2_legacy_yuyv/image"
		GST_NAME="camera_gst_video2_yuyv"
		GST_LAUNCH="/apollo/modules/drivers/camera_gst/launch/camera_gst_video2_yuyv.launch"
		GST_DAG="/apollo/modules/drivers/camera_gst/dag/camera_gst_video2_yuyv.dag"
		GST_PROCESS="camera_gst_video2_yuyv"
		GST_CHANNEL="/apollo/sensor/camera/video2/image"
		;;
	rgb)
		LEGACY_NAME="legacy_camera_video2_rgb"
		LEGACY_LAUNCH="/apollo/modules/drivers/camera_gst/tools/baseline/legacy_camera_video2_rgb.launch"
		LEGACY_DAG="/apollo/modules/drivers/camera_gst/tools/baseline/legacy_camera_video2_rgb.dag"
		LEGACY_PROCESS="legacy_camera_video2_rgb"
		LEGACY_CHANNEL="/apollo/sensor/camera/video2_legacy_rgb/image"
		GST_NAME="camera_gst_video2_rgb"
		GST_LAUNCH="/apollo/modules/drivers/camera_gst/launch/camera_gst_video2_rgb.launch"
		GST_DAG="/apollo/modules/drivers/camera_gst/dag/camera_gst_video2_rgb.dag"
		GST_PROCESS="camera_gst_video2_rgb"
		GST_CHANNEL="/apollo/sensor/camera/video2/image"
		;;
	*)
		echo "FORMAT must be yuyv or rgb, got: ${FORMAT}" >&2
		exit 2
		;;
esac

RESULT_DIR="${RESULT_ROOT}/${FORMAT}"
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

read_proc_io_value() {
	local pid="$1"
	local field_name="$2"
	awk -v key="${field_name}:" '$1 == key { print $2; exit }' "/proc/${pid}/io" 2>/dev/null || echo 0
}

capture_io_snapshot() {
	local pid="$1"
	local output_file="$2"
	local read_bytes
	local write_bytes
	read_bytes="$(read_proc_io_value "${pid}" read_bytes)"
	write_bytes="$(read_proc_io_value "${pid}" write_bytes)"
	printf 'read_bytes=%s\nwrite_bytes=%s\n' "${read_bytes}" "${write_bytes}" > "${output_file}"
}

extract_io_delta_mb() {
	local start_file="$1"
	local end_file="$2"
	local field_name="$3"
	local start_value
	local end_value
	start_value="$(awk -F= -v key="${field_name}" '$1 == key { print $2; exit }' "${start_file}" 2>/dev/null || echo 0)"
	end_value="$(awk -F= -v key="${field_name}" '$1 == key { print $2; exit }' "${end_file}" 2>/dev/null || echo 0)"
	awk -v start="${start_value:-0}" -v end="${end_value:-0}" 'BEGIN {
		delta = end - start;
		if (delta < 0) delta = 0;
		printf("%.2f", delta / 1024 / 1024);
	}'
}

find_component_log() {
	local pid="$1"
	find "${GLOG_log_dir}" -maxdepth 1 -type f \
		\( -name "camera_gst.log.INFO.*.${pid}" -o -name "camera.log.INFO.*.${pid}" \) \
		-print -quit 2>/dev/null || true
}

extract_gpu_avg_fps() {
	local component_log="$1"
	if [[ -z "${component_log}" || ! -f "${component_log}" ]]; then
		echo "0"
		return 0
	fi
	local frame_count
	frame_count="$(grep -oE 'camera_gst gpu frames observed=[0-9]+' "${component_log}" | tail -n1 | awk -F= '{print $2}' || true)"
	frame_count="${frame_count:-0}"
	awk -v count="${frame_count}" -v seconds="$((BENCH_SECONDS + WARMUP_SECONDS))" 'BEGIN { printf("%.2f", seconds ? count / seconds : 0) }'
}

run_cyber_launch() {
	"${CYBER_LAUNCH_CMD[@]}" "$@"
}

set +u
source /apollo/cyber/setup.bash
set -u

resolve_cyber_launch
resolve_cyber_recorder

TEGRATSTATS_BIN="${TEGRATSTATS_BIN:-$(command -v tegrastats || true)}"

stop_case() {
	local launch_file="$1"
	local dag_file="$2"
	run_cyber_launch stop "${launch_file}" >/dev/null 2>&1 || true
	pkill -f "mainboard -d ${dag_file}" >/dev/null 2>&1 || true
}

wait_for_pid() {
	local dag_file="$1"
	local process_name="$2"
	local pid=""
	for _ in $(seq 1 30); do
		pid="$(pgrep -f "mainboard -d ${dag_file} -p ${process_name}" | head -n1 || true)"
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
	if [[ -z "${channel}" ]]; then
		return 1
	fi
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

summarize_case() {
	local proc_file="$1"
	local tegra_file="$2"
	local avg_fps="$3"
	local io_read_mb="$4"
	local io_write_mb="$5"
	local summary_file="$6"
	awk -F, -v fps="${avg_fps}" '
		{ cpu += $2; rss += $3; rows += 1 }
		END {
			printf("avg_cpu_percent=%.2f\n", rows ? cpu / rows : 0)
			printf("avg_rss_mb=%.2f\n", rows ? rss / rows / 1024 : 0)
			printf("avg_fps=%.2f\n", fps)
		}
	' "${proc_file}" > "${summary_file}"
	printf 'io_read_mb=%s\nio_write_mb=%s\n' "${io_read_mb}" "${io_write_mb}" >> "${summary_file}"
	if [[ -s "${tegra_file}" ]]; then
		cat "${tegra_file}" >> "${summary_file}"
	fi
}

run_case() {
	local name="$1"
	local launch_file="$2"
	local dag_file="$3"
	local process_name="$4"
	local channel="$5"

	local case_dir="${RESULT_DIR}/${name}"
	rm -rf "${case_dir}"
	mkdir -p "${case_dir}"

	stop_case "${launch_file}" "${dag_file}"
	trap 'stop_case "'"${launch_file}"'" "'"${dag_file}"'"' RETURN

	run_cyber_launch start "${launch_file}" > "${case_dir}/launch.log" 2>&1 &
	local launch_pid="$!"
	local module_pid
	module_pid="$(wait_for_pid "${dag_file}" "${process_name}" || true)"
	if [[ -z "${module_pid}" ]]; then
		module_pid="$(wait_for_launch_log_pid "${case_dir}/launch.log" || true)"
	fi
	if [[ -z "${module_pid}" ]]; then
		tail -n 120 "${case_dir}/launch.log" >&2 || true
		return 1
	fi

	capture_io_snapshot "${module_pid}" "${case_dir}/io_start.txt"
	sleep "${WARMUP_SECONDS}"
	collect_proc_stats "${module_pid}" "${case_dir}/proc_stats.csv" &
	local proc_pid="$!"
	collect_tegrastats "${case_dir}/tegrastats.log" &
	local tegra_pid="$!"
	record_channel "${channel}" "${case_dir}/record" || true
	wait "${proc_pid}" || true
	wait "${tegra_pid}" || true
	capture_io_snapshot "${module_pid}" "${case_dir}/io_end.txt"
	local avg_fps
	avg_fps="$(extract_avg_fps "${channel}" "${case_dir}/record")"
	local component_log=""
	if [[ "${avg_fps}" == "0" || -z "${channel}" ]]; then
		component_log="$(find_component_log "${module_pid}")"
		if [[ -n "${component_log}" ]]; then
			cp "${component_log}" "${case_dir}/component.log" 2>/dev/null || true
		fi
	fi
	if [[ "${avg_fps}" == "0" || -z "${channel}" ]]; then
		avg_fps="$(extract_gpu_avg_fps "${component_log}")"
	fi
	local io_read_mb
	local io_write_mb
	io_read_mb="$(extract_io_delta_mb "${case_dir}/io_start.txt" "${case_dir}/io_end.txt" read_bytes)"
	io_write_mb="$(extract_io_delta_mb "${case_dir}/io_start.txt" "${case_dir}/io_end.txt" write_bytes)"
	summarize_case "${case_dir}/proc_stats.csv" "${case_dir}/tegrastats.log" "${avg_fps}" "${io_read_mb}" "${io_write_mb}" "${case_dir}/summary.txt"

	stop_case "${launch_file}" "${dag_file}"
	kill "${launch_pid}" >/dev/null 2>&1 || true
	wait "${launch_pid}" >/dev/null 2>&1 || true
	trap - RETURN
}

run_case "${LEGACY_NAME}" "${LEGACY_LAUNCH}" "${LEGACY_DAG}" "${LEGACY_PROCESS}" "${LEGACY_CHANNEL}"
run_case "${GST_NAME}" "${GST_LAUNCH}" "${GST_DAG}" "${GST_PROCESS}" "${GST_CHANNEL}"

python3 - "${RESULT_DIR}" "${LEGACY_NAME}" "${GST_NAME}" <<'PY'
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
		out.write("camera format baseline\n")
		out.write("======================\n\n")
		for name, metrics in ((legacy_name, legacy), (gst_name, gst)):
			out.write(f"[{name}]\n")
			for key in ("avg_cpu_percent", "avg_rss_mb", "avg_fps", "io_read_mb", "io_write_mb"):
				out.write(f"{key}={metrics.get(key, '0')}\n")
			out.write("\n")
		out.write(f"[delta: {gst_name} - {legacy_name}]\n")
		for key in ("avg_cpu_percent", "avg_rss_mb", "avg_fps", "io_read_mb", "io_write_mb"):
			delta = float(gst.get(key, "0")) - float(legacy.get(key, "0"))
			out.write(f"{key}={delta:.2f}\n")
PY

echo "baseline comparison report written to ${RESULT_DIR}/comparison_report.txt"
