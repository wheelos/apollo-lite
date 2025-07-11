#!/usr/bin/env bash

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

set -euo pipefail

# Get project root directory
APOLLO_ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "${APOLLO_ROOT_DIR}/scripts/apollo.bashrc"

STAGE="${STAGE:-dev}"
DIFF_MODE=0
BASE_COMMIT=""

readonly EXCLUDED_PATHS=(
  ".cache"
  ".teamcity"
  "bazel-apollo"
  "bazel-bin"
  "bazel-out"
  "bazel-testlogs"
  "data"
  "docs"
)
readonly EXCLUDED_FILES=(
  "nohup.out"
)

function get_find_exclude_args() {
  local argv=()
  for p in "${EXCLUDED_PATHS[@]}"; do
    argv+=(-not -path "./${p}/*")
  done
  for f in "${EXCLUDED_FILES[@]}"; do
    argv+=(-not -name "${f}")
  done
  echo "${argv[@]}"
}

function get_git_pathspec_exclude_args() {
    local argv=()
    for p in "${EXCLUDED_PATHS[@]}"; do
        argv+=(":(exclude)${p}/*")
    done
    for f in "${EXCLUDED_FILES[@]}"; do
        argv+=(":(exclude)${f}")
    done
    echo "${argv[@]}"
}

declare -a cached_diff_files=()
cached_diff_done=0

function get_all_changed_files_from_git() {
  if [[ $cached_diff_done -eq 0 ]]; then
    local exclude_args
    read -r -a exclude_args < <(get_git_pathspec_exclude_args)
    mapfile -t cached_diff_files < <(git diff --name-only --diff-filter=ACMRTUXB "${BASE_COMMIT}" HEAD -- . "${exclude_args[@]}")
    cached_diff_done=1
  fi
}

function get_changed_files_by_pattern() {
  local file_pattern="$1"
  if [[ "${DIFF_MODE}" -ne 1 || -z "${BASE_COMMIT}" ]]; then
    return 1
  fi

  get_all_changed_files_from_git

  local filtered_files=()
  for f in "${cached_diff_files[@]}"; do
    if [[ "$f" =~ $file_pattern ]]; then
      filtered_files+=("$f")
    fi
  done

  if [[ "${#filtered_files[@]}" -gt 0 ]]; then
    printf "%s\n" "${filtered_files[@]}"
  fi
}

# ============================================================================
# Check Functions
# ============================================================================

function run_cpp_format() {
  info "::group::Running C++ Format Check (Clang-Format)"
  local CLANG_FORMAT_EXPECTED_VERSION="18"
  local clang_format_cmd="clang-format-${CLANG_FORMAT_EXPECTED_VERSION}"

  if ! command -v "${clang_format_cmd}" &>/dev/null; then
    error "Specific clang-format version '${clang_format_cmd}' not found. Please install."
    return 1
  fi

  local files_to_check=()
  if [[ "${DIFF_MODE}" -eq 1 ]]; then
    mapfile -t files_to_check < <(get_changed_files_by_pattern '.*\.(c|cc|cpp|h|hpp)')
    if [[ ${#files_to_check[@]} -eq 0 ]]; then
      info "No C++ changes detected for format check."
      info "::endgroup::"
      return 0
    fi
    info "Checking ${#files_to_check[@]} changed C++ files..."
  else
    info "Running full C++ format check..."
    local find_exclude_args=()
    read -r -a find_exclude_args < <(get_find_exclude_args)
    mapfile -t files_to_check < <(find . -type f \( -name "*.c" -o -name "*.cc" -o -name "*.cpp" -o -name "*.h" -o -name "*.hpp" \) "${find_exclude_args[@]}")
    if [[ ${#files_to_check[@]} -eq 0 ]]; then
      info "No C++ files found for full format check."
      info "::endgroup::"
      return 0
    fi
  fi

  printf '%s\0' "${files_to_check[@]}" | xargs -0 -r -P 4 "${clang_format_cmd}" --dry-run --Werror || {
    error "C++ formatting issues found. Please run 'clang-format -i <file>...' to fix."
    return 1
  }

  info "C++ format check passed."
  info "::endgroup::"
}

function run_py_checks() {
  info "::group::Running Python Format (Black, isort) and Lint (Flake8) Checks"
  if ! command -v black &>/dev/null || \
     ! command -v isort &>/dev/null || \
     ! command -v flake8 &>/dev/null; then
      error "One or more required Python commands (black, isort, flake8) not found. Please install them via 'pip install black isort flake8'."
      return 1
  fi

  local files_to_check=()
  if [[ "${DIFF_MODE}" -eq 1 ]]; then
    mapfile -t files_to_check < <(get_changed_files_by_pattern '.*\.py')
    if [[ ${#files_to_check[@]} -eq 0 ]]; then
      info "No Python changes detected for check."
      info "::endgroup::"
      return 0
    fi
    info "Checking ${#files_to_check[@]} changed Python files..."
  else
    info "Running full Python checks..."
    local find_exclude_args=()
    read -r -a find_exclude_args < <(get_find_exclude_args)
    mapfile -t files_to_check < <(find . -type f -name "*.py" "${find_exclude_args[@]}")
    if [[ "${#files_to_check[@]}" -eq 0 ]]; then
      info "No Python files found for full check."
      info "::endgroup::"
      return 0
    fi
  fi

  info "Running Black (format check)..."
  black --check "${files_to_check[@]}" || {
      error "Python formatting issues found by Black. Please run 'black <file>...' to fix."
      return 1
  }

  # isort
  info "Running isort (import sorting check)..."
  isort --check-only --profile black "${files_to_check[@]}" || {
      error "Python import sorting issues found by isort. Please run 'isort <file>...' to fix."
      return 1
  }

  # Flake8
  info "Running Flake8 (lint check)..."
  flake8 "${files_to_check[@]}" || {
      error "Python lint issues found by Flake8. Please review the errors above."
      return 1
  }

  info "Python checks passed."
  info "::endgroup::"
}

function run_sh_lint() {
  info "::group::Running Shell Lint Check (shellcheck)"
  if ! command -v shellcheck &>/dev/null; then
    error "Command 'shellcheck' not found. Please install."
    return 1
  fi

  local files_to_check=()
  if [[ "${DIFF_MODE}" -eq 1 ]]; then
    mapfile -t files_to_check < <(get_changed_files_by_pattern '.*\.(sh|bashrc)')
    if [[ ${#files_to_check[@]} -eq 0 ]]; then
      info "No Shell changes detected for lint check."
      info "::endgroup::"
      return 0
    fi
     info "Checking ${#files_to_check[@]} changed shell scripts..."
  else
    info "Running full Shell lint check..."
    local find_exclude_args=()
    read -r -a find_exclude_args < <(get_find_exclude_args)
    mapfile -t files_to_check < <(find . -type f \( -name "*.sh" -o -name "*.bashrc" \) "${find_exclude_args[@]}")
    if [[ ${#files_to_check[@]} -eq 0 ]]; then
      info "No shell scripts found for full lint check."
      info "::endgroup::"
      return 0
    fi
  fi

  shellcheck -x --shell=bash "${files_to_check[@]}" || {
    error "Shell lint issues found by shellcheck. Please review the errors above."
    return 1
  }

  info "Shell lint check passed."
  info "::endgroup::"
}

function run_buildifier_check() {
  info "::group::Running Buildifier Format Check"
  if ! command -v buildifier &>/dev/null; then
    error "Command 'buildifier' not found. Please install it."
    return 1
  fi

  local files_to_check=()
  if [[ "${DIFF_MODE}" -eq 1 ]]; then
    mapfile -t files_to_check < <(get_changed_files_by_pattern '.*(BUILD|bzl|bazelrc)')
    if [[ ${#files_to_check[@]} -eq 0 ]]; then
      info "No Bazel file changes detected for format check."
      info "::endgroup::"
      return 0
    fi
    info "Checking ${#files_to_check[@]} changed Bazel files..."
  else
    info "Running full Buildifier format check..."
    local find_exclude_args=()
    read -r -a find_exclude_args < <(get_find_exclude_args)
    mapfile -t files_to_check < <(find . -type f \( -name "BUILD" -o -name "*.bzl" -o -name "*.bazelrc" \) "${find_exclude_args[@]}")
    if [[ ${#files_to_check[@]} -eq 0 ]]; then
      info "No Bazel files found for full format check."
      info "::endgroup::"
      return 0
    fi
  fi

  printf '%s\0' "${files_to_check[@]}" | xargs -0 -r -P 4 buildifier -mode=check -lint=warn || {
      error "Bazel formatting issues found by Buildifier. Please run 'buildifier <file>...' to fix."
      return 1
  }

  info "Buildifier format check passed."
  info "::endgroup::"
}

function run_cpp_lint() {
  info "::group::Running C++ Lint (Bazel cpplint)"
  if ! command -v bazel &>/dev/null; then
    error "Bazel is not installed. Please set it up correctly."
    return 1
  fi

  pushd "${APOLLO_ROOT_DIR}" >/dev/null
  local cpp_dirs=("cyber")
  if [[ "${STAGE}" == "dev" ]]; then
    cpp_dirs+=("modules")
  fi

  find "${cpp_dirs[@]}" -name BUILD -print0 | while IFS= read -r -d '' prey; do
    if grep -q -E 'cc_library|cc_test|cc_binary|cuda_library' "${prey}" && \
       ! grep -q 'cpplint()' "${prey}"; then
      warning "BUILD file missing cpplint(): ${prey}"
    fi
  done
  popd >/dev/null

  local bazel_targets=("//cyber/...")
  if [[ "${STAGE}" == "dev" ]]; then
    bazel_targets+=("//modules/...")
  fi

  info "Running Bazel cpplint test on targets: ${bazel_targets[*]}"
  bazel test --config=cpplint "${bazel_targets[@]}" || {
    error "C++ lint issues found by Bazel cpplint. Please review the errors above."
    return 1
  }
  info "C++ lint passed."
  info "::endgroup::"
}

# ============================================================================
# Script Entry Point and Argument Parsing
# ============================================================================

function print_usage() {
  cat <<EOF
Usage: $0 [Options]
A comprehensive linting and formatting script for the Apollo project.

Options:
  --diff [commitish]   Enable diff mode. Only check files changed since <commitish>.
                       If [commitish] is omitted, it's auto-detected in CI environments.
  --py                 Run Python format (Black) and lint (Flake8) checks.
  --sh                 Run Shell script lint (Shellcheck) check.
  --cpp-format         Run C++ format (Clang-Format) check.
  --cpp-lint           Run C++ lint (Bazel cpplint) check.
  --bazel              Run Bazel file format (Buildifier) check.
  -a, --all            Run all available checks.
  --stage <dev|prod>   Specify stage for linting (default: dev). Affects C++ lint targets.
  -h, --help           Show this help message and exit.
EOF
}

function main() {
  local CPP_FORMAT_FLAG=0
  local CPP_LINT_FLAG=0
  local PYTHON_CHECKS_FLAG=0
  local SHELL_LINT_FLAG=0
  local BUILDIFIER_CHECK_FLAG=0

  if [[ "$#" -eq 0 ]]; then
    print_usage
    error "No operation specified. Use --all or select specific checks."
    return 1
  fi

  while [[ "$#" -gt 0 ]]; do
    local opt="$1"
    shift
    case "${opt}" in
      --py)         PYTHON_CHECKS_FLAG=1 ;;
      --cpp-lint)   CPP_LINT_FLAG=1 ;;
      --cpp-format) CPP_FORMAT_FLAG=1 ;;
      --sh)         SHELL_LINT_FLAG=1 ;;
      --bazel)      BUILDIFIER_CHECK_FLAG=1 ;;
      --diff)
        DIFF_MODE=1
        if [[ -n "${1-}" && ! "$1" =~ ^- ]]; then
          BASE_COMMIT="$1"
          shift
        fi
        ;;
      --stage)
        if [[ -z "${1-}" || ! "$1" =~ ^(dev|prod)$ ]]; then
          error "Missing or invalid argument for --stage. Use 'dev' or 'prod'."
          return 1
        fi
        STAGE="$1"
        shift
        ;;
      -a|--all)
        PYTHON_CHECKS_FLAG=1
        CPP_LINT_FLAG=1
        CPP_FORMAT_FLAG=1
        SHELL_LINT_FLAG=1
        BUILDIFIER_CHECK_FLAG=1
        ;;
      -h|--help)
        print_usage
        return 0
        ;;
      *)
        error "Unknown option: ${opt}"
        print_usage
        return 1
        ;;
    esac
  done

  if [[ "${DIFF_MODE}" -eq 1 && -z "${BASE_COMMIT}" ]]; then
    if [[ -n "${GITHUB_BASE_REF:-}" ]]; then
      local merge_base
      merge_base=$(git merge-base "origin/${GITHUB_BASE_REF}" HEAD)
      if [[ -n "$merge_base" ]]; then
        BASE_COMMIT="$merge_base"
        info "::notice::Using merge-base origin/${GITHUB_BASE_REF} for diff: ${BASE_COMMIT}"
      else
        error "Could not find merge-base for origin/${GITHUB_BASE_REF}. Cannot perform diff."
        return 1
      fi
    elif [[ "${GITHUB_EVENT_NAME:-}" == "push" ]];then
        BASE_COMMIT="HEAD~1"
        info "::notice::Using HEAD~1 for diff base in push event."
    else
        local remote_main="origin/main"
        if git show-ref --quiet --verify "refs/remotes/${remote_main}"; then
             BASE_COMMIT="${remote_main}"
             info "::notice::Defaulting diff base to ${remote_main}."
        elif git show-ref --quiet --verify "refs/remotes/origin/master"; then
             BASE_COMMIT="origin/master"
             info "::notice::Defaulting diff base to origin/master."
        else
            error "Diff mode enabled but no base commit specified or auto-detected."
            return 1
        fi
    fi
  fi

  if [[ ${CPP_FORMAT_FLAG} -eq 0 && \
        ${CPP_LINT_FLAG} -eq 0 && \
        ${PYTHON_CHECKS_FLAG} -eq 0 && \
        ${SHELL_LINT_FLAG} -eq 0 && \
        ${BUILDIFIER_CHECK_FLAG} -eq 0 ]]; then
    print_usage
    error "No specific checks selected. Use --all or select specific checks like --py, --cpp-format etc."
    return 1
  fi

  if [[ "${CPP_FORMAT_FLAG}" -eq 1 ]]; then
    run_cpp_format
  fi
  if [[ "${CPP_LINT_FLAG}" -eq 1 ]]; then
    run_cpp_lint
  fi
  if [[ "${PYTHON_CHECKS_FLAG}" -eq 1 ]]; then
    run_py_checks
  fi
  if [[ "${SHELL_LINT_FLAG}" -eq 1 ]]; then
    run_sh_lint
  fi
  if [[ "${BUILDIFIER_CHECK_FLAG}" -eq 1 ]]; then
    run_buildifier_check
  fi

  info "All selected checks passed successfully."
}

main "$@"
