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
  --flagfile=FILE                               (default: modules/common/data/global_flagfile.txt; forwarded to offline binary)
  --enable_tracking=true|false                  (default: false)
  --use_hdmap=true|false                        (default: false)
  --use_tracking_info=true|false                (default: false)
  --min_life_time=FLOAT                         (default: -1.0)
  --log_dir=DIR                                 (default: RESULT_DIR/logs; script logs only, not passed to binaries)
  --tag=NAME                                    (default: timestamp)
  --offline_bin=PATH                            (default: bazel-bin/modules/perception/lidar/tools/offline_lidar_obstacle_perception)
  --benchmark_bin=PATH                          (default: bazel-bin/modules/perception/tool/benchmark/lidar/lidar_benchmark)
  --web_vis_dir=DIR                             (if set, export interactive web viewer data/assets here)
  --web_vis_max_points=INT                      (default: 30000)
  --web_exporter_bin=PATH                       (default: bazel-bin/modules/perception/tool/benchmark/lidar/lidar_web_visualizer_exporter)

Optional (evaluation):
  --gt_dir=DIR                                  (if set, run evaluator)
  --reserve=STRING                              (forwarded to evaluator, format: "KEY:VALUE|KEY:VALUE", e.g. "JACCARD:0.9|RANGE:distance")

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
flagfile=""
enable_tracking="false"
use_hdmap="false"
use_tracking_info="false"
min_life_time="-1.0"
lidar_detection_config_file=""
lidar_tracking_config_file=""
reserve=""
tag=""
log_dir=""
offline_bin=""
benchmark_bin=""
web_vis_dir=""
web_vis_max_points="30000"
web_exporter_bin=""

while [[ $# -gt 0 ]]; do
  arg="$1"
  case "${arg}" in
    --help|-h) usage; exit 0 ;;
    --pcd_dir=*) pcd_dir="${arg#*=}" ;;
    --pose_dir=*) pose_dir="${arg#*=}" ;;
    --result_dir=*) result_dir="${arg#*=}" ;;
    --gt_dir=*) gt_dir="${arg#*=}" ;;
    --sensor_name=*) sensor_name="${arg#*=}" ;;
    --flagfile=*) flagfile="${arg#*=}" ;;
    --enable_tracking=*) enable_tracking="${arg#*=}" ;;
    --use_hdmap=*) use_hdmap="${arg#*=}" ;;
    --use_tracking_info=*) use_tracking_info="${arg#*=}" ;;
    --min_life_time=*) min_life_time="${arg#*=}" ;;
    --lidar_detection_config_file=*) lidar_detection_config_file="${arg#*=}" ;;
    --lidar_tracking_config_file=*) lidar_tracking_config_file="${arg#*=}" ;;
    --reserve=*) reserve="${arg#*=}" ;;
    --tag=*) tag="${arg#*=}" ;;
    --log_dir=*) log_dir="${arg#*=}" ;;
    --offline_bin=*) offline_bin="${arg#*=}" ;;
    --benchmark_bin=*) benchmark_bin="${arg#*=}" ;;
    --web_vis_dir=*) web_vis_dir="${arg#*=}" ;;
    --web_vis_max_points=*) web_vis_max_points="${arg#*=}" ;;
    --web_exporter_bin=*) web_exporter_bin="${arg#*=}" ;;
    *)
      echo "Unknown arg: ${arg}" 1>&2
      usage
      exit 2
      ;;
  esac
  shift
done

if [[ -z "${pcd_dir}" || -z "${result_dir}" || -z "${lidar_detection_config_file}" || -z "${lidar_tracking_config_file}" ]]; then
  echo "Missing required args." 1>&2
  usage
  exit 2
fi

if [[ -z "${tag}" ]]; then
  tag="$(date +%Y%m%d_%H%M%S)"
fi

if [[ -z "${flagfile}" ]]; then
  flagfile="${ROOT}/modules/common/data/global_flagfile.txt"
fi

if [[ ! -f "${flagfile}" ]]; then
  echo "Missing flagfile: ${flagfile}" 1>&2
  exit 4
fi

mkdir -p "${result_dir}"
if [[ -z "${log_dir}" ]]; then
  log_dir="${result_dir}/logs"
fi
mkdir -p "${log_dir}"

if [[ -z "${pose_dir}" ]]; then
  enable_tracking="false"
fi

if [[ -z "${offline_bin}" ]]; then
  offline_bin="${ROOT}/bazel-bin/modules/perception/lidar/tools/offline_lidar_obstacle_perception"
fi
if [[ -z "${benchmark_bin}" ]]; then
  benchmark_bin="${ROOT}/bazel-bin/modules/perception/tool/benchmark/lidar/lidar_benchmark"
fi
if [[ -z "${web_exporter_bin}" ]]; then
  web_exporter_bin="${ROOT}/bazel-bin/modules/perception/tool/benchmark/lidar/lidar_web_visualizer_exporter"
fi

if [[ ! -x "${offline_bin}" ]]; then
  echo "Missing offline binary: ${offline_bin}" 1>&2
  echo "Build it first (example): bazel build //modules/perception/lidar/tools:offline_lidar_obstacle_perception" 1>&2
  exit 5
fi
if [[ ! -x "${benchmark_bin}" && -n "${gt_dir}" ]]; then
  echo "Missing benchmark binary: ${benchmark_bin}" 1>&2
  echo "Build it first (example): bazel build //modules/perception/tool/benchmark/lidar:lidar_benchmark" 1>&2
  exit 6
fi
if [[ -n "${web_vis_dir}" && ! -x "${web_exporter_bin}" ]]; then
  echo "Missing web exporter binary: ${web_exporter_bin}" 1>&2
  echo "Build it first (example): bazel build //modules/perception/tool/benchmark/lidar:lidar_web_visualizer_exporter" 1>&2
  exit 7
fi

total_steps=1
if [[ -n "${gt_dir}" ]]; then
  total_steps=$((total_steps + 1))
fi
if [[ -n "${web_vis_dir}" ]]; then
  total_steps=$((total_steps + 1))
fi
current_step=1

echo "[${current_step}/${total_steps}] Generate results -> ${result_dir}"
offline_args=(
  "--flagfile=${flagfile}"
  "--pcd_path=${pcd_dir}"
  "--pose_path=${pose_dir}"
  "--output_path=${result_dir}"
  "--sensor_name=${sensor_name}"
  "--lidar_detection_config_file=${lidar_detection_config_file}"
  "--lidar_tracking_config_file=${lidar_tracking_config_file}"
  "--enable_tracking=${enable_tracking}"
  "--use_hdmap=${use_hdmap}"
  "--use_tracking_info=${use_tracking_info}"
  "--min_life_time=${min_life_time}"
)

# NOTE: Do NOT pass a standalone `--` when running the binary directly.
# gflags treats it as "end of flags", which would make all flags below ignored.
"${offline_bin}" "${offline_args[@]}" \
  2>&1 | tee "${log_dir}/offline_lidar_obstacle_perception.${tag}.log"

pcd_list=""
result_list=""
gt_list=""
if [[ -n "${gt_dir}" || -n "${web_vis_dir}" ]]; then
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
  if [[ -n "${gt_dir}" ]]; then
    : > "${gt_list}"
  fi

  missing=0
  for pcd in "${pcd_files[@]}"; do
    base="$(basename "${pcd}")"
    stem="${base%.pcd}"
    res_txt="${result_dir}/${stem}.txt"
    if [[ ! -f "${res_txt}" ]]; then
      echo "Missing result txt: ${res_txt}" 1>&2
      missing=$((missing + 1))
      continue
    fi
    if [[ -n "${gt_dir}" ]]; then
      gt_txt="${gt_dir}/${stem}.txt"
      if [[ ! -f "${gt_txt}" ]]; then
        echo "Missing gt txt: ${gt_txt}" 1>&2
        missing=$((missing + 1))
        continue
      fi
      printf '%s\n' "${gt_txt}" >> "${gt_list}"
    fi
    printf '%s\n' "${pcd}" >> "${pcd_list}"
    printf '%s\n' "${res_txt}" >> "${result_list}"
  done

  if [[ "${missing}" -ne 0 ]]; then
    echo "Found ${missing} missing files, abort downstream steps." 1>&2
    exit 4
  fi
fi

if [[ -n "${gt_dir}" ]]; then
  current_step=$((current_step + 1))
  echo "[${current_step}/${total_steps}] Evaluate results vs GT -> ${gt_dir}"

eval_log="${log_dir}/lidar_benchmark.${tag}.log"
eval_args=(
  "--cloud=${pcd_list}"
  "--result=${result_list}"
  "--groundtruth=${gt_list}"
  "--is_folder=false"
)
if [[ -n "${reserve}" ]]; then
  eval_args+=("--reserve=${reserve}")
fi

"${benchmark_bin}" "${eval_args[@]}" 2>&1 | tee "${eval_log}"
fi

if [[ -n "${web_vis_dir}" ]]; then
  current_step=$((current_step + 1))
  echo "[${current_step}/${total_steps}] Export interactive web viewer -> ${web_vis_dir}"
  mkdir -p "${web_vis_dir}"
  web_args=(
    "--cloud=${pcd_list}"
    "--result=${result_list}"
    "--output=${web_vis_dir}"
    "--is_folder=false"
    "--max_points=${web_vis_max_points}"
  )
  if [[ -n "${gt_dir}" ]]; then
    web_args+=("--groundtruth=${gt_list}")
  fi
  if [[ -n "${reserve}" ]]; then
    web_args+=("--reserve=${reserve}")
  fi
  "${web_exporter_bin}" "${web_args[@]}" \
    2>&1 | tee "${log_dir}/lidar_web_visualizer_exporter.${tag}.log"
  cp "${ROOT}/modules/perception/lidar/tools/web_viewer/index.html" "${web_vis_dir}/index.html"
  cp "${ROOT}/modules/perception/lidar/tools/web_viewer/app.js" "${web_vis_dir}/app.js"
  cp "${ROOT}/modules/perception/lidar/tools/web_viewer/styles.css" "${web_vis_dir}/styles.css"
fi

echo "Done. Logs:"
echo "  Offline: ${log_dir}/offline_lidar_obstacle_perception.${tag}.log"
if [[ -n "${gt_dir}" ]]; then
  echo "  Eval   : ${eval_log}"
fi
if [[ -n "${web_vis_dir}" ]]; then
  echo "  Web    : ${web_vis_dir}/index.html"
  echo "           Serve with: cd ${web_vis_dir} && python3 -m http.server 8765"
fi
