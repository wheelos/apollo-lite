#!/usr/bin/env bash

set -euo pipefail

APOLLO_ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
TARGET_PATHS=()
CPP_FORMAT_FLAG=0
PYTHON_FORMAT_FLAG=0
BUILDIFIER_FORMAT_FLAG=0

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

function print_usage() {
  cat <<EOF
Usage: $0 [Options]

Options:
  --cpp                 Format C/C++ files with clang-format.
  --py                  Format Python files with isort and Black.
  --bazel               Format Bazel files with Buildifier.
  --all                 Format C/C++ and Python/Bazel files.
  --path <path>         Limit formatting to a file or directory.
  -h, --help            Show this help message and exit.

Examples:
  $0 --cpp --path modules/open_space_planning
  $0 --all
EOF
}

function require_command() {
  local command_name="$1"
  if ! command -v "${command_name}" &>/dev/null; then
    echo "Error: command '${command_name}' not found." >&2
    return 1
  fi
}

function collect_files() {
  local file_pattern="$1"
  shift
  local paths=("$@")

  if [[ "${#paths[@]}" -eq 0 ]]; then
    paths=(".")
  fi

  if [[ "${#TARGET_PATHS[@]}" -eq 0 ]]; then
    local find_exclude_args=()
    local path
    for path in "${EXCLUDED_PATHS[@]}"; do
      find_exclude_args+=(-not -path "./${path}/*")
    done
    find "${paths[@]}" -type f -regextype posix-extended -regex "${file_pattern}" \
      "${find_exclude_args[@]}"
  else
    find "${paths[@]}" -type f -regextype posix-extended -regex "${file_pattern}"
  fi
}

function format_cpp() {
  require_command clang-format
  local files=()
  mapfile -t files < <(collect_files '.*\.(c|cc|cpp|h|hpp)$' "${TARGET_PATHS[@]}")
  if [[ "${#files[@]}" -gt 0 ]]; then
    printf '%s\0' "${files[@]}" | xargs -0 -r clang-format -i
  fi
}

function format_python() {
  require_command isort
  require_command black
  local files=()
  mapfile -t files < <(collect_files '.*\.py$' "${TARGET_PATHS[@]}")
  if [[ "${#files[@]}" -gt 0 ]]; then
    isort --profile black "${files[@]}"
    black "${files[@]}"
  fi
}

function format_bazel() {
  require_command buildifier
  local files=()
  mapfile -t files < <(collect_files '.*(BUILD|\.bzl|\.bazelrc)$' "${TARGET_PATHS[@]}")
  if [[ "${#files[@]}" -gt 0 ]]; then
    printf '%s\0' "${files[@]}" | xargs -0 -r buildifier -mode=fix -lint=warn
  fi
}

function main() {
  while [[ "$#" -gt 0 ]]; do
    case "$1" in
      --cpp)   CPP_FORMAT_FLAG=1 ;;
      --py)    PYTHON_FORMAT_FLAG=1 ;;
      --bazel) BUILDIFIER_FORMAT_FLAG=1 ;;
      --all)
        CPP_FORMAT_FLAG=1
        PYTHON_FORMAT_FLAG=1
        BUILDIFIER_FORMAT_FLAG=1
        ;;
      --path)
        if [[ -z "${2-}" || "$2" == -* ]]; then
          echo "Error: --path requires a file or directory." >&2
          return 1
        fi
        TARGET_PATHS+=("$2")
        shift
        ;;
      -h|--help)
        print_usage
        return 0
        ;;
      *)
        echo "Error: unknown option '$1'." >&2
        print_usage
        return 1
        ;;
    esac
    shift
  done

  if [[ "${CPP_FORMAT_FLAG}" -eq 0 &&
        "${PYTHON_FORMAT_FLAG}" -eq 0 &&
        "${BUILDIFIER_FORMAT_FLAG}" -eq 0 ]]; then
    print_usage
    return 1
  fi

  cd "${APOLLO_ROOT_DIR}"
  if [[ "${CPP_FORMAT_FLAG}" -eq 1 ]]; then
    format_cpp
  fi
  if [[ "${PYTHON_FORMAT_FLAG}" -eq 1 ]]; then
    format_python
  fi
  if [[ "${BUILDIFIER_FORMAT_FLAG}" -eq 1 ]]; then
    format_bazel
  fi
}

main "$@"