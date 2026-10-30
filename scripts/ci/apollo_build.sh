#! /usr/bin/env bash

###############################################################################
# Copyright 2020 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################


# Responsibility: expand Apollo build targets and execute Bazel with the
# selected configuration and resource limits. It does not initialize runtime.

set -e

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"
source "${TOP_DIR}/scripts/apollo_base.sh"
source "${TOP_DIR}/scripts/cyber_targets.sh"

ARCH="$(uname -m)"

USE_GPU=-1

CMDLINE_OPTIONS=()
SHOW_ONLY=0
EXPLICIT_SELECTION=0
SHORTHAND_TARGETS=
DISABLED_TARGETS=
CUSTOM_JOBS=${CUSTOM_JOBS:-""}
CUSTOM_RAMS=${CUSTOM_RAMS:-""}
CUSTOM_CPUS=${CUSTOM_CPUS:-""}

# associative map of module-specific defines (bash 4+)
declare -A MODULE_DEFINES=()
# Example usage:
# MODULE_DEFINES[dreamview]="no_files=true"

function _determine_drivers_disabled() {
  :
}

function _determine_perception_disabled() {
  if [ "${USE_GPU}" -eq 0 ]; then
    DISABLED_TARGETS="${DISABLED_TARGETS} except //modules/perception/..."
  fi
}

function _determine_planning_disabled() {
  if [ "${USE_GPU}" -eq 0 ]; then
    DISABLED_TARGETS="${DISABLED_TARGETS} \
        except //modules/planning/open_space/trajectory_smoother:planning_block"
  fi
}

function _determine_map_disabled() {
  if [ "${USE_GPU}" -eq 0 ]; then
    DISABLED_TARGETS="${DISABLED_TARGETS} except //modules/map/pnc_map:cuda_pnc_util \
                      except //modules/map/pnc_map:cuda_util_test"
  fi
}

function determine_disabled_targets() {
  if [ "$#" -eq 0 ]; then
    _determine_drivers_disabled
    _determine_perception_disabled
    _determine_planning_disabled
    _determine_map_disabled
    echo "${DISABLED_TARGETS}"
    return
  fi

  for component in $@; do
    case "${component}" in
      drivers)
        _determine_drivers_disabled
        ;;
      perception)
        _determine_perception_disabled
        ;;
      planning)
        _determine_planning_disabled
        ;;
      map)
        _determine_map_disabled
        ;;
    esac
  done

  echo "${DISABLED_TARGETS}"
  # DISABLED_CYBER_MODULES="except @core//cyber/record:record_file_integration_test"
}

# components="$(echo -e "${@// /\\n}" | sort -u)"
# if [ ${PIPESTATUS[0]} -ne 0 ]; then ... ; fi

function determine_build_targets_and_defines() {
  local targets_all=""
  local defines_all=""

  if [[ "$#" -eq 0 ]]; then
    targets_all="//modules/... union $(cyber_runtime_targets)"
  else
    for component in "$@"; do
      local selected_target=""
      case "${component}" in
        hardware) selected_target="//tools/hardware:hardware" ;;
        hardware_package) selected_target="//tools/hardware:install" ;;
        algorithms) selected_target="//tools/modules:algorithms" ;;
        drivers) selected_target="@hardware.camera//... @hardware.camera_gst//... @hardware.image_decompress//... @hardware.canbus//... @hardware.lidar//... @hardware.radar//... @hardware.gnss//... @hardware.hal//... @hardware.audio//... @hardware.video//..." ;;
        canbus) selected_target="@hardware.vehicle//..." ;;
        transform) selected_target="@wheelos_transform//modules/transform/..." ;;
      esac
      if [[ -n "${selected_target}" ]]; then
        targets_all+=" ${selected_target}"
      elif [[ "$component" == "cyber" ]]; then
        if [[ "${HOST_OS}" == "Linux" ]]; then
          targets_all+=" $(cyber_runtime_targets) //modules/tools/visualizer/..."
        else
          targets_all+=" $(cyber_runtime_targets)"
        fi
      elif [[ -d "${APOLLO_ROOT_DIR}/modules/${component}" ]]; then
        targets_all+=" //modules/${component}/... union $(cyber_runtime_targets)"
      elif [[ -d "${APOLLO_ROOT_DIR}/${component}" ]]; then
        targets_all+=" //${component}/..."
      elif [[ $component == @* || $component == //* ]]; then
        targets_all+=" ${component}"
      else
        error "Directory ${APOLLO_ROOT_DIR}/modules/${component} not found. Exiting ..."
        exit 1
      fi

      if [[ -n "${MODULE_DEFINES[$component]}" ]]; then
        defines_all+=" --define ${MODULE_DEFINES[$component]}"
      fi
    done
  fi
  echo "${targets_all} ${defines_all}"
}

function _chk_n_set_gpu_arg() {
  local arg="$1"
  local use_gpu=-1
  if [ "${arg}" = "cpu" ]; then
    use_gpu=0
  elif [ "${arg}" = "gpu" ]; then
    use_gpu=1
  else
    # Do nothing
    return 0
  fi

  if [[ "${USE_GPU}" -lt 0 || "${USE_GPU}" = "${use_gpu}" ]]; then
    USE_GPU="${use_gpu}"
    return 0
  fi

  error "Mixed use of '--config=cpu' and '--config=gpu' may" \
    "lead to unexpected behavior. Exiting..."
  exit 1
}

function parse_cmdline_arguments() {
  local -a known_options=()
  local remained_args=""

  for ((pos = 1; pos <= $#; pos++)); do #do echo "$#" "$i" "${!i}"; done
    local opt="${!pos}"
    local optarg

    case "${opt}" in
      --show)
        SHOW_ONLY=1
        ;;
      --explicit-selection)
        EXPLICIT_SELECTION=1
        ;;
      --config=*)
        optarg="${opt#*=}"
        known_options+=("${opt}")
        _chk_n_set_gpu_arg "${optarg}"
        ;;
      --config|-c|--define|--platforms)
        ((++pos))
        optarg="${!pos}"
        if [[ -z "${optarg}" ]]; then
          error "Missing value for ${opt}"
          exit 2
        fi
        known_options+=("${opt}" "${optarg}")
        if [[ "${opt}" == "--config" ]]; then
          _chk_n_set_gpu_arg "${optarg}"
        fi
        ;;
      --jobs|--jobs=*|--cpus|--cpus=*|--rams|--rams=*)
        optarg="${opt#*=}"
        if [[ "${optarg}" == "" || "${optarg}" == "${opt}" ]]; then
          ((++pos))
          optarg="${!pos}"
        fi
        if [[ -z "${optarg}" ]]; then
          error "Missing value for ${opt}"
          exit 2
        fi
        case "${opt%%=*}" in
          --jobs) known_options+=("--jobs=${optarg}") ;;
          --cpus) known_options+=("--local_resources=cpu=${optarg}") ;;
          --rams) known_options+=("--local_resources=memory=${optarg}") ;;
        esac
        ;;
      --*=*)
        known_options+=("${opt}")
        ;;
      --*)
        error "Use --option=value for Bazel options: ${opt}"
        exit 2
        ;;
      *)
        remained_args="${remained_args} ${opt}"
        ;;
    esac
  done
  # Strip leading whitespaces
  remained_args="$(echo "${remained_args}" | sed -e 's/^[[:space:]]*//')"

  CMDLINE_OPTIONS=("${known_options[@]}")
  SHORTHAND_TARGETS="${remained_args}"
}

function determine_cpu_or_gpu_build() {
  if [ "${USE_GPU}" -lt 0 ]; then
    if [ "${USE_GPU_TARGET}" -eq 0 ]; then
      CMDLINE_OPTIONS=(--config=cpu "${CMDLINE_OPTIONS[@]}")
    else
      CMDLINE_OPTIONS=(--config=gpu "${CMDLINE_OPTIONS[@]}")
    fi
    # USE_GPU unset, defaults to USE_GPU_TARGET
    USE_GPU="${USE_GPU_TARGET}"
  elif [ "${USE_GPU}" -gt "${USE_GPU_TARGET}" ]; then
    warning "USE_GPU=${USE_GPU} without GPU can't compile. Exiting ..."
    exit 1
  fi

  if [ "${USE_GPU}" -eq 1 ]; then
    ok "Running GPU build on ${ARCH} platform."
  else
    ok "Running CPU build on ${ARCH} platform."
  fi
}

function format_bazel_targets() {
  local targets="$(echo $@ | xargs)"
  targets="${targets// union / }"   # replace all matches of "A union B" to "A B"
  targets="${targets// except / -}" # replaces all matches of "A except B" to "A-B"
  echo "${targets}"
}

function _read_cgroup_memory_limit() {
  local limit_file
  for limit_file in \
    /sys/fs/cgroup/memory.max \
    /sys/fs/cgroup/memory/memory.limit_in_bytes; do
    if [[ -r "${limit_file}" ]]; then
      local limit
      limit="$(cat "${limit_file}")"
      if [[ "${limit}" != "max" && "${limit}" =~ ^[0-9]+$ ]]; then
        # cgroup v1 uses a very large integer for "unlimited".
        if awk -v limit="${limit}" 'BEGIN { exit !(limit > 0 && limit < 1e18) }'; then
          echo "${limit}"
          return
        fi
      fi
    fi
  done
  awk '/MemTotal/ {printf "%.0f", $2 * 1024}' /proc/meminfo
}

function _read_cgroup_cpu_limit() {
  if [[ -r /sys/fs/cgroup/cpu.max ]]; then
    local quota period
    read -r quota period < /sys/fs/cgroup/cpu.max
    if [[ "${quota}" != "max" && "${quota}" =~ ^[0-9]+$ &&
          "${period}" =~ ^[0-9]+$ && "${period}" -gt 0 ]]; then
      awk -v quota="${quota}" -v period="${period}" \
        'BEGIN { value = int((quota + period - 1) / period); print (value > 0 ? value : 1) }'
      return
    fi
  fi

  if [[ -r /sys/fs/cgroup/cpu/cpu.cfs_quota_us &&
        -r /sys/fs/cgroup/cpu/cpu.cfs_period_us ]]; then
    local quota period
    quota="$(cat /sys/fs/cgroup/cpu/cpu.cfs_quota_us)"
    period="$(cat /sys/fs/cgroup/cpu/cpu.cfs_period_us)"
    if [[ "${quota}" -gt 0 && "${period}" -gt 0 ]]; then
      awk -v quota="${quota}" -v period="${period}" \
        'BEGIN { value = int((quota + period - 1) / period); print (value > 0 ? value : 1) }'
      return
    fi
  fi

  nproc
}

function run_bazel_build() {
  local build_targets
  build_targets="$(determine_build_targets_and_defines ${SHORTHAND_TARGETS})"

  local disabled_targets
  disabled_targets="$(determine_disabled_targets ${SHORTHAND_TARGETS})"
  disabled_targets="$(echo ${disabled_targets} | xargs)"

  # Note(storypku): Workaround for in case "/usr/bin/bazel: Argument list too long"
  # bazel build ${CMDLINE_OPTIONS} ${job_args} $(bazel query ${build_targets})
  local formatted_targets="$(format_bazel_targets ${build_targets} ${disabled_targets})"

  local -a target_args=()
  read -r -a target_args <<< "${formatted_targets}"
  if [[ "${SHOW_ONLY}" -eq 1 ]]; then
    local query="set(${target_args[*]})"
    if [[ " ${target_args[*]} " == *" //tools/hardware:hardware "* ]]; then
      query+=" union //tools/hardware:manifest"
    fi
    bazel cquery "${CMDLINE_OPTIONS[@]}" --output=starlark \
      "--starlark:file=${TOP_DIR}/tools/products/describe.bzl" "${query}"
    return
  fi

  info "Build Overview: "
  if [[ "${EXPLICIT_SELECTION}" -eq 0 ]]; then
    info "${TAB}USE_GPU: ${USE_GPU}  [ 0 for CPU, 1 for GPU ]"
  else
    info "${TAB}Compute mode: explicit Bazel configuration (see analyzed preview)"
  fi
  info "${TAB}Bazel Options: ${GREEN}${CMDLINE_OPTIONS[*]}${NO_COLOR}"
  info "${TAB}Build Targets: ${GREEN}${build_targets}${NO_COLOR}"
  info "${TAB}Disabled:      ${YELLOW}${disabled_targets}${NO_COLOR}"

  local memory_bytes="$(_read_cgroup_memory_limit)"
  local cpu_count="$(_read_cgroup_cpu_limit)"
  if [[ ! "${memory_bytes}" =~ ^[0-9]+$ || "${memory_bytes}" -le 0 ]]; then
    memory_bytes="$(awk '/MemTotal/ {printf "%.0f", $2 * 1024}' /proc/meminfo)"
  fi
  if [[ ! "${cpu_count}" =~ ^[0-9]+$ || "${cpu_count}" -le 0 ]]; then
    cpu_count=1
  fi
  if [[ "${cpu_count}" -gt 8 ]]; then
    cpu_count=8
  fi

  # Keep one compile slot per CPU and reserve roughly 2 GiB per slot.
  local memory_jobs
  memory_jobs="$(awk -v bytes="${memory_bytes}" \
    'BEGIN { value = int(bytes / 2147483648); print (value > 0 ? value : 1) }')"
  local default_jobs="${cpu_count}"
  if [[ "${memory_jobs}" -lt "${default_jobs}" ]]; then
    default_jobs="${memory_jobs}"
  fi

  local jobs_args="--jobs=${default_jobs}"
  if [[ -n "${CUSTOM_JOBS}" ]]; then
    jobs_args="--jobs=${CUSTOM_JOBS}"
  fi
  local cpus_args="--local_resources=cpu=${cpu_count}"
  if [[ -n "${CUSTOM_CPUS}" ]]; then
    cpus_args="--local_resources=cpu=${CUSTOM_CPUS}"
  fi
  # Reserve 70% of the cgroup memory limit for Bazel actions.
  local memory_limit
  memory_limit="$(awk -v bytes="${memory_bytes}" 'BEGIN { printf "%.0f", bytes * 0.7 }')"
  local rams_args="--local_resources=memory=${memory_limit}"
  if [[ -n "${CUSTOM_RAMS}" ]]; then
    rams_args="--local_resources=memory=${CUSTOM_RAMS}"
  fi

  local -a copts_args=()
  if [[ "${EXPLICIT_SELECTION}" -eq 0 && "$(uname -m)" == "aarch64" ]]; then
    copts_args=(--copt=-march=native --host_copt=-march=native --copt=-fPIC --host_copt=-fPIC)
  fi
  local -a build_args=("${copts_args[@]}" "${jobs_args}" "${cpus_args}" "${rams_args}")
  info "${TAB}Build Command: bazel build ${build_args[*]} ${CMDLINE_OPTIONS[*]} -- ${formatted_targets}"

  local bazel_cache_root="${BAZEL_CACHE_DIR:-${APOLLO_CACHE_DIR}/bazel}"
  local repository_cache="${bazel_cache_root}/repo_cache"
  local disk_cache="${bazel_cache_root}/disk_cache"
  if ! mkdir -p "${repository_cache}" "${disk_cache}"; then
    error "Unable to create Bazel caches under ${bazel_cache_root}."
    exit 1
  fi
  if [[ ! -w "${repository_cache}" || ! -w "${disk_cache}" ]]; then
    error "Bazel caches under ${bazel_cache_root} are not writable by $(id -un) (uid $(id -u))."
    error "Fix the mounted cache ownership or configure the mode-specific BAZEL_CACHE_DIR; refusing a temporary cache fallback."
    exit 1
  fi

  local bazel_cache_args=(
    "--repository_cache=${repository_cache}"
    "--disk_cache=${disk_cache}"
  )

  bazel build "${build_args[@]}" "${CMDLINE_OPTIONS[@]}" "${bazel_cache_args[@]}" -- "${target_args[@]}"
}

function main() {
  if ! "${APOLLO_IN_DOCKER}"; then
    error "The build operation must be run from within docker container"
    exit 1
  fi
  parse_cmdline_arguments "$@"
  if [[ "${EXPLICIT_SELECTION}" -eq 0 ]]; then
    determine_cpu_or_gpu_build
  else
    info "Using explicit build configuration; no host GPU/Orin inference."
  fi

  run_bazel_build

  if [ -z "${SHORTHAND_TARGETS}" ]; then
    SHORTHAND_TARGETS="apollo"
  fi

  if [[ "${SHOW_ONLY}" -eq 0 ]]; then
    success "Done building ${SHORTHAND_TARGETS}. Enjoy!"
  fi
}

main "$@"
