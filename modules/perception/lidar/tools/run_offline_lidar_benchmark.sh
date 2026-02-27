#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Run offline LiDAR perception to generate result txts, then (optionally) run PR/AP evaluator.

Required (generation):
  --pcd_dir=DIR
  --result_dir=DIR
  --lidar_detection_config_file=FILE            (PipelineConfig pb.txt/pb)
  --lidar_tracking_config_file=FILE             (PipelineConfig pb.txt/pb; required by tool even if tracking disabled)

Optional (generation):
  --pose_dir=DIR                                (if empty, tracking is forced off)
  --sensor_name=NAME                            (default: velodyne64)
  --enable_tracking=true|false                  (default: false)
  --use_hdmap=true|false                        (default: false)
  --use_tracking_info=true|false                (default: false)
  --min_life_time=FLOAT                         (default: -1.0)
  --log_dir=DIR                                 (default: RESULT_DIR/logs)
  --tag=NAME                                    (default: timestamp)
  --no_build                                    (skip bazel build)

Optional (evaluation):
  --gt_dir=DIR                                  (if set, run evaluator)
  --reserve=STRING                              (forwarded to evaluator, e.g. "JACCARD=0.9;RANGE=distance")

Examples:
  # 1) Generate result txts (CNNSEG RTNet / TensorRT path decided by pipeline config + model_type)
  ./modules/perception/lidar/tools/run_offline_lidar_benchmark.sh \
    --pcd_dir=/data/pcd \
    --result_dir=/tmp/cnnseg_result \
    --lidar_detection_config_file=modules/perception/pipeline/config/lidar_detection_pipeline.pb.txt \
    --lidar_tracking_config_file=modules/perception/pipeline/config/lidar_tracking_pipeline.pb.txt

  # 2) Regression: use old results as pseudo-GT
  ./modules/perception/lidar/tools/run_offline_lidar_benchmark.sh \
    --pcd_dir=/data/pcd \
    --result_dir=/tmp/result_new \
    --gt_dir=/tmp/result_old \
    --lidar_detection_config_file=modules/perception/pipeline/config/lidar_detection_pipeline_trt.pb.txt \
    --lidar_tracking_config_file=modules/perception/pipeline/config/lidar_tracking_pipeline.pb.txt \
    --reserve="JACCARD=0.9"
EOF
}

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
cd "${ROOT}"

pcd_dir=""
pose_dir=""
result_dir=""
gt_dir=""
sensor_name="velodyne64"
enable_tracking="false"
use_hdmap="false"
use_tracking_info="false"
min_life_time="-1.0"
lidar_detection_config_file=""
lidar_tracking_config_file=""
reserve=""
do_build="true"
tag=""
log_dir=""

for arg in "$@"; do
  case "${arg}" in
    --help|-h) usage; exit 0 ;;
    --pcd_dir=*) pcd_dir="${arg#*=}" ;;
    --pose_dir=*) pose_dir="${arg#*=}" ;;
    --result_dir=*) result_dir="${arg#*=}" ;;
    --gt_dir=*) gt_dir="${arg#*=}" ;;
    --sensor_name=*) sensor_name="${arg#*=}" ;;
    --enable_tracking=*) enable_tracking="${arg#*=}" ;;
    --use_hdmap=*) use_hdmap="${arg#*=}" ;;
    --use_tracking_info=*) use_tracking_info="${arg#*=}" ;;
    --min_life_time=*) min_life_time="${arg#*=}" ;;
    --lidar_detection_config_file=*) lidar_detection_config_file="${arg#*=}" ;;
    --lidar_tracking_config_file=*) lidar_tracking_config_file="${arg#*=}" ;;
    --reserve=*) reserve="${arg#*=}" ;;
    --tag=*) tag="${arg#*=}" ;;
    --log_dir=*) log_dir="${arg#*=}" ;;
    --no_build) do_build="false" ;;
    *)
      echo "Unknown arg: ${arg}" 1>&2
      usage
      exit 2
      ;;
  esac
done

if [[ -z "${pcd_dir}" || -z "${result_dir}" || -z "${lidar_detection_config_file}" || -z "${lidar_tracking_config_file}" ]]; then
  echo "Missing required args." 1>&2
  usage
  exit 2
fi

if [[ -z "${tag}" ]]; then
  tag="$(date +%Y%m%d_%H%M%S)"
fi

mkdir -p "${result_dir}"
if [[ -z "${log_dir}" ]]; then
  log_dir="${result_dir}/logs"
fi
mkdir -p "${log_dir}"

if [[ -z "${pose_dir}" ]]; then
  enable_tracking="false"
fi

offline_target="//modules/perception/lidar/tools:offline_lidar_obstacle_perception"
benchmark_target="//modules/perception/tool/benchmark/lidar:lidar_benchmark"

if [[ "${do_build}" == "true" ]]; then
  bazel build "${offline_target}" "${benchmark_target}"
fi

echo "[1/2] Generate results -> ${result_dir}"
bazel run "${offline_target}" -- \
  --pcd_path="${pcd_dir}" \
  --pose_path="${pose_dir}" \
  --output_path="${result_dir}" \
  --sensor_name="${sensor_name}" \
  --lidar_detection_config_file="${lidar_detection_config_file}" \
  --lidar_tracking_config_file="${lidar_tracking_config_file}" \
  --enable_tracking="${enable_tracking}" \
  --use_hdmap="${use_hdmap}" \
  --use_tracking_info="${use_tracking_info}" \
  --min_life_time="${min_life_time}" \
  --log_dir="${log_dir}" \
  2>&1 | tee "${log_dir}/offline_lidar_obstacle_perception.${tag}.log"

if [[ -z "${gt_dir}" ]]; then
  echo "[2/2] Skip evaluation (no --gt_dir)."
  exit 0
fi

echo "[2/2] Evaluate results vs GT -> ${gt_dir}"

lists_dir="${result_dir}/__benchmark_lists_${tag}"
mkdir -p "${lists_dir}"

pcd_list="${lists_dir}/cloud.list"
result_list="${lists_dir}/result.list"
gt_list="${lists_dir}/groundtruth.list"

shopt -s nullglob
pcd_files=("${pcd_dir}"/*.pcd)
shopt -u nullglob
if [[ "${#pcd_files[@]}" -eq 0 ]]; then
  echo "No .pcd files found under: ${pcd_dir}" 1>&2
  exit 3
fi

: > "${pcd_list}"
: > "${result_list}"
: > "${gt_list}"

missing=0
for pcd in "${pcd_files[@]}"; do
  base="$(basename "${pcd}")"
  stem="${base%.pcd}"
  res_txt="${result_dir}/${stem}.txt"
  gt_txt="${gt_dir}/${stem}.txt"
  if [[ ! -f "${res_txt}" ]]; then
    echo "Missing result txt: ${res_txt}" 1>&2
    missing=$((missing + 1))
    continue
  fi
  if [[ ! -f "${gt_txt}" ]]; then
    echo "Missing gt txt: ${gt_txt}" 1>&2
    missing=$((missing + 1))
    continue
  fi
  printf '%s\n' "${pcd}" >> "${pcd_list}"
  printf '%s\n' "${res_txt}" >> "${result_list}"
  printf '%s\n' "${gt_txt}" >> "${gt_list}"
done

if [[ "${missing}" -ne 0 ]]; then
  echo "Found ${missing} missing files, abort evaluation." 1>&2
  exit 4
fi

eval_log="${log_dir}/lidar_benchmark.${tag}.log"
if [[ -n "${reserve}" ]]; then
  bazel run "${benchmark_target}" -- \
    --cloud="${pcd_list}" \
    --result="${result_list}" \
    --groundtruth="${gt_list}" \
    --is_folder=false \
    --reserve="${reserve}" \
    2>&1 | tee "${eval_log}"
else
  bazel run "${benchmark_target}" -- \
    --cloud="${pcd_list}" \
    --result="${result_list}" \
    --groundtruth="${gt_list}" \
    --is_folder=false \
    2>&1 | tee "${eval_log}"
fi

echo "Done. Logs:"
echo "  Offline: ${log_dir}/offline_lidar_obstacle_perception.${tag}.log"
echo "  Eval   : ${eval_log}"

