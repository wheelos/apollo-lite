#! /usr/bin/env bash
set -e

# Responsibility: dispatch build, test, lint, format, and clean commands.
# Container lifecycle and runtime environment initialization belong elsewhere.

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
APOLLO_BUILD_PHASE=1 source "${TOP_DIR}/scripts/apollo.bashrc"

ARCH="$(uname -m)"
SUPPORTED_ARCHS=" x86_64 aarch64 "
APOLLO_VERSION="@non-git"
APOLLO_ENV=""

: ${STAGE:=dev}

AVAILABLE_COMMANDS="profiles config run install package build build_dbg build_opt build_cpu build_gpu build_opt_gpu test coverage lint \
                    buildify check build_fe build_teleop build_prof doc clean format usage -h --help"

function check_architecture_support() {
    if [[ "${SUPPORTED_ARCHS}" != *" ${ARCH} "* ]]; then
        error "Unsupported CPU arch: ${ARCH}. Currently, Apollo only" \
            "supports running on the following CPU archs:"
        error "${TAB}${SUPPORTED_ARCHS}"
        exit 1
    fi
}

function check_platform_support() {
    local platform="$(uname -s)"
    if [[ "${platform}" != "Linux" ]]; then
        error "Unsupported platform: ${platform}."
        error "${TAB}Apollo is expected to run on Linux systems (E.g., Debian/Ubuntu)."
        exit 1
    fi
}

function check_minimal_memory_requirement() {
    local minimal_mem_gb="2.0"
    local actual_mem_gb="$(free -m | awk '/Mem:/ {printf("%0.2f", $2 / 1024.0)}')"
    if (($(echo "$actual_mem_gb < $minimal_mem_gb" | bc -l))); then
        warning "System memory [${actual_mem_gb}G] is lower than the minimum required" \
            "[${minimal_mem_gb}G]. Apollo build could fail."
    fi
}


function check_apollo_version() {
    local branch="$(git_branch)"
    if [ "${branch}" == "${APOLLO_VERSION}" ]; then
        return
    fi
    local sha1="$(git_sha1)"
    local stamp="$(git_date)"
    APOLLO_VERSION="${branch}-${stamp}-${sha1}"
}

function apollo_env_setup() {
    check_apollo_version

    check_architecture_support
    check_platform_support
    check_minimal_memory_requirement
    determine_gpu_use_target

    APOLLO_ENV="${APOLLO_ENV} STAGE=${STAGE}"
    # Add more here ...

    info "Apollo Environment Settings:"
    info "${TAB}APOLLO_ROOT_DIR: ${APOLLO_ROOT_DIR}"
    info "${TAB}APOLLO_CACHE_DIR: ${APOLLO_CACHE_DIR}"
    info "${TAB}APOLLO_IN_DOCKER: ${APOLLO_IN_DOCKER}"
    info "${TAB}APOLLO_VERSION: ${APOLLO_VERSION}"
    if "${APOLLO_IN_DOCKER}"; then
        info "${TAB}DOCKER_IMG: ${DOCKER_IMG##*:}"
    fi
    info "${TAB}APOLLO_ENV: ${APOLLO_ENV}"
    info "${TAB}USE_GPU: USE_GPU_HOST=${USE_GPU_HOST} USE_GPU_TARGET=${USE_GPU_TARGET}"

}

#TODO(all): Update node modules
function build_dreamview_frontend() {
    pushd "${APOLLO_ROOT_DIR}/modules/dreamview/frontend" >/dev/null
    yarn build
    popd >/dev/null
}

function build_test_and_lint() {
    local -a jetson_orin_args=()
    local jetson_orin_config
    jetson_orin_config="$(determine_jetson_orin_build_config)"
    if [[ -n "${jetson_orin_config}" ]]; then
        jetson_orin_args=("${jetson_orin_config}")
    fi
    env APOLLO_BUILD_PHASE=1 ${APOLLO_ENV} bash "${build_sh}" "${jetson_orin_args[@]}"
    env ${APOLLO_ENV} bash "${test_sh}" --config=unit_test "${jetson_orin_args[@]}"
    env ${APOLLO_ENV} bash "${APOLLO_ROOT_DIR}/scripts/ci/apollo_lint.sh" --all
    success "Build and Test and Lint finished."
}

function _usage() {
    echo -e "\n${RED}Usage${NO_COLOR}:
    .${BOLD}/apollo.sh${NO_COLOR} [OPTION]"
    echo -e "\n${RED}Product builds (inside the managed dev container)${NO_COLOR}:
    bash apollo.sh profiles
    bash apollo.sh build --product=devkit-x86 --show
    bash apollo.sh build --product=devkit-x86
    Custom: build --hardware=hw_x86_devkit --modules=rtk_localization,open_space_planning,control --compute=cpu
    Options: --mode=opt|dbg --jobs=N --cpus=N --rams=MB
    build --all: explicitly build all modules; legacy bare build also builds all.
    Product presets are build inventories, not deployment qualification.
    Migration guide: tools/products/README.md#legacy-command-migration
    Advanced/legacy commands follow."
    echo -e "\n${RED}Options${NO_COLOR}:
    ${BLUE}config${NO_COLOR}: show guidance for local Bazel overrides (.bazelrc.user); read-only.
    ${BLUE}build [module]${NO_COLOR}: run build for cyber (<module> = cyber, includes @core runtime tools) or modules/<module>.  If <module> unspecified, build all.
    ${BLUE}build hardware --config=<profile>${NO_COLOR}: build only selected hardware (see tools/hardware/README.md).
    ${BLUE}build hardware_package --config=<profile>${NO_COLOR}: package the selected hardware overlay.
    ${BLUE}build algorithms --config=<profile>${NO_COLOR}: build selected algorithm modules (see tools/modules/README.md).
    ${BLUE}build_{opt,dbg,cpu,gpu,opt_cpu,dbg_cpu,opt_gpu,dbg_gpu} [module]${NO_COLOR}: legacy mode aliases; module directories remain recursive.
    ${BLUE}test [module]${NO_COLOR}: run unittest for cyber (module='cyber') or modules/<module>. If unspecified, test all.
    ${BLUE}coverage [module]${NO_COLOR}: run coverage test for cyber (module='cyber') or modules/<module>. If unspecified, coverage all.
    ${BLUE}lint${NO_COLOR}: run code style check
    ${BLUE}buildify${NO_COLOR}: run 'buildifier' to fix style of bazel files.
    ${BLUE}check${NO_COLOR}: run build, test and lint on all modules. Recommmened before checking in new code.
    ${BLUE}build_fe${NO_COLOR}: compile frontend JS code for Dreamview. Requires all node_modules pre-installed.
    ${BLUE}build_teleop${NO_COLOR}: run build with teleop enabled.
    ${BLUE}build_prof [module]${NO_COLOR}: build with perf profiling support. Not implemented yet.
    ${BLUE}doc${NO_COLOR}: generate doxygen document
    ${BLUE}run${NO_COLOR}: develop and start a registered runtime (e.g. run transform)
    ${BLUE}install${NO_COLOR}: install a runtime (--destdir=/absolute/path; release sources by default)
    ${BLUE}package${NO_COLOR}: archive a runtime (release sources by default)
    ${BLUE}release${NO_COLOR}: legacy development-source binary delivery
    ${BLUE}clean${NO_COLOR}: cleanup bazel output and log/coredump files
    ${BLUE}format${NO_COLOR}: format C++/Python/Bazel/Shell files
    ${BLUE}usage${NO_COLOR}: show this message and exit
    "
}

function _check_command() {
    local name="${BASH_SOURCE[0]}"
    local commands="$(echo ${AVAILABLE_COMMANDS} | xargs)"
    local help_msg="Run './apollo.sh --help' for usage."
    local cmd="$@"

    python scripts/command_checker.py --name "${name}" --command "${cmd}" --available "${commands}" --helpmsg "${help_msg}"
}

function _config_bazel_local_override() {
    local local_bazelrc="${APOLLO_ROOT_DIR}/.bazelrc.user"
    local opt="${1:-}"

    if [[ -n "${opt}" ]]; then
        error "config is read-only. Edit ${local_bazelrc} explicitly; no files were removed."
        return 2
    fi

    info "Project Bazel defaults are in .bazelrc"
    info "Optional local overrides file: ${local_bazelrc}"
    info "Create it manually if needed, for example:"
    info "${TAB}echo 'build --jobs=4' >> ${local_bazelrc}"
    return 0
}

function main() {
    if [ "$#" -eq 0 ]; then
        _usage
        exit 0
    fi

    local cmd="$1"
    shift
    case "${cmd}" in
        -h|--help|usage) _usage; return ;;
        profiles)
            python3 "${TOP_DIR}/scripts/ci/apollo_product.py" --list "$@"
            return
            ;;
        config) _config_bazel_local_override "$@"; return ;;
        run|install|package)
            python3 "${TOP_DIR}/scripts/ci/apollo_runtime.py" "${cmd}" "$@"
            return
            ;;
        build)
            local arg
            for arg in "$@"; do
                case "${arg}" in
                    -h|--help) _usage; return ;;
                    --product|--product=*|--hardware|--hardware=*|--modules|--modules=*|--compute|--compute=*|--mode|--mode=*|--all|--show)
                        python3 "${TOP_DIR}/scripts/ci/apollo_product.py" "$@"
                        return
                        ;;
                esac
            done
            if [[ "$#" -eq 0 ]]; then
                warning "Bare build compiles ALL modules. Prefer build --product=<name>; use build --all to request a full build explicitly."
            fi
            ;;
    esac
    local option
    for option in "$@"; do
        if [[ "${option}" == --product || "${option}" == --product=* ]]; then
            error "--product is supported by 'build' only. Use explicit module tests; product testing and standalone packaging are not available."
            return 2
        fi
    done
    apollo_env_setup

    local build_sh="${APOLLO_ROOT_DIR}/scripts/ci/apollo_build.sh"
    local test_sh="${APOLLO_ROOT_DIR}/scripts/ci/apollo_test.sh"
    local coverage_sh="${APOLLO_ROOT_DIR}/scripts/ci/apollo_coverage.sh"
    local ci_sh="${APOLLO_ROOT_DIR}/scripts/ci/apollo_ci.sh"
    local -a jetson_orin_args=()
    local jetson_orin_config
    jetson_orin_config="$(determine_jetson_orin_build_config)"
    if [[ -n "${jetson_orin_config}" ]]; then
        jetson_orin_args=("${jetson_orin_config}")
    fi

    case "${cmd}" in
        build)
            env APOLLO_BUILD_PHASE=1 ${APOLLO_ENV} bash "${build_sh}" "${jetson_orin_args[@]}" "$@"
            ;;
        build_opt|build_dbg|build_cpu|build_dbg_cpu|build_opt_cpu|build_gpu|build_dbg_gpu|build_opt_gpu|build_prof|build_teleop)
            local -a modes=() mode_args=()
            local mode
            IFS=_ read -r -a modes <<< "${cmd#build_}"
            for mode in "${modes[@]}"; do
                mode_args+=("--config=${mode}")
            done
            env APOLLO_BUILD_PHASE=1 ${APOLLO_ENV} bash "${build_sh}" "${mode_args[@]}" "${jetson_orin_args[@]}" "$@"
            ;;
        build_fe)
            build_dreamview_frontend
            ;;
        test)
            env ${APOLLO_ENV} bash "${test_sh}" --config=unit_test "${jetson_orin_args[@]}" "$@"
            ;;
        coverage)
            env ${APOLLO_ENV} bash "${coverage_sh}" "${jetson_orin_args[@]}" "$@"
            ;;
        cibuild)
            env ${APOLLO_ENV} bash "${ci_sh}" "build"
            ;;
        citest)
            env ${APOLLO_ENV} bash "${ci_sh}" "test"
            ;;
        cilint)
            env ${APOLLO_ENV} bash "${ci_sh}" "lint"
            ;;
        check)
            build_test_and_lint
            ;;
        buildify)
            env ${APOLLO_ENV} bash "${APOLLO_ROOT_DIR}/scripts/apollo_buildify.sh"
            ;;
        lint)
            env ${APOLLO_ENV} bash "${APOLLO_ROOT_DIR}/scripts/ci/apollo_lint.sh" "$@"
            ;;
        clean)
            env ${APOLLO_ENV} bash "${APOLLO_ROOT_DIR}/scripts/ci/apollo_clean.sh" "$@"
            ;;
        release)
            env ${APOLLO_ENV} bash "${APOLLO_ROOT_DIR}/scripts/ci/apollo_release.sh" "$@"
            ;;
        doc)
            env ${APOLLO_ENV} bash "${APOLLO_ROOT_DIR}/scripts/ci/apollo_docs.sh" "$@"
            ;;
        format)
            env ${APOLLO_ENV} bash "${APOLLO_ROOT_DIR}/scripts/ci/apollo_format.sh" "$@"
            ;;
        *)
            _check_command "${cmd}"
            ;;
    esac
}

main "$@"
