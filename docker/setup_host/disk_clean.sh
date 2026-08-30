#!/usr/bin/env bash
set -euo pipefail

LOG_PREFIX="[WheelOS Disk Clean]"
logit() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') $LOG_PREFIX $1"
}

# 1. Resolve APOLLO_ROOT from docker/.env
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APOLLO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ENV_FILE="${APOLLO_ROOT}/docker/.env"

if [[ -f "${ENV_FILE}" ]]; then
    # Extract APOLLO_ROOT from .env safely
    ENV_APOLLO_ROOT=$(grep -E "^APOLLO_ROOT=" "${ENV_FILE}" | cut -d '=' -f 2 | tr -d '"'\')
    if [[ -n "${ENV_APOLLO_ROOT}" ]]; then
        APOLLO_ROOT="${ENV_APOLLO_ROOT}"
    fi
else
    logit "Warning: ${ENV_FILE} not found. Using script relative path: ${APOLLO_ROOT}"
fi

# Remove trailing slashes
APOLLO_ROOT="${APOLLO_ROOT%/}"

if [[ -z "${APOLLO_ROOT}" || ! -d "${APOLLO_ROOT}" ]]; then
    logit "ERROR: Invalid or missing APOLLO_ROOT: ${APOLLO_ROOT}"
    exit 1
fi

# 2. Strict Target Directories
TARGET_BAG_DIR="${APOLLO_ROOT}/data/bag"
TARGET_LOG_DIR="${APOLLO_ROOT}/data/log"
TARGET_CORE_DIR="${APOLLO_ROOT}/data/core"

BAG_RETENTION_DAYS=7
LOG_RETENTION_DAYS=14
CORE_RETENTION_DAYS=3
MIN_FREE_SPACE=20  # Percentage

# 3. Space Check
if df -h "${APOLLO_ROOT}" &>/dev/null; then
    CURRENT_USE=$(df --output=pcent "${APOLLO_ROOT}" | sed '1d' | tr -d ' %')
    CURRENT_FREE=$((100 - CURRENT_USE))

    if [[ "$CURRENT_FREE" -gt "$MIN_FREE_SPACE" ]]; then
        logit "Space is sufficient (${CURRENT_FREE}% free > ${MIN_FREE_SPACE}%). Exit."
        exit 0
    fi
fi

# 4. Check if recording is active to avoid corrupting running tasks
if pgrep -f "cyber_recorder" > /dev/null; then
    logit "Warning: AD system is recording (cyber_recorder is running). Abort."
    exit 0
fi

# 5. Clean function with strict directory validation
AUTHORIZED_DIRS=("${TARGET_BAG_DIR}" "${TARGET_LOG_DIR}" "${TARGET_CORE_DIR}")

clean_directory() {
    local target_dir=$1
    local max_days=$2
    local pattern=$3

    # Safety Check: Prevent relative paths or path traversal
    # Ensure canonical path
    local real_target_dir
    real_target_dir=$(realpath "${target_dir}" 2>/dev/null || echo "")

    local is_authorized=false
    for auth_dir in "${AUTHORIZED_DIRS[@]}"; do
        local real_auth_dir
        real_auth_dir=$(realpath "${auth_dir}" 2>/dev/null || echo "")
        if [[ -n "${real_target_dir}" && "${real_target_dir}" == "${real_auth_dir}" ]]; then
            is_authorized=true
            break
        fi
    done

    if [[ "${is_authorized}" != true ]]; then
        logit "ERROR: Unauthorized access directory ${target_dir}. Aborting."
        return 1
    fi

    if [[ ! -d "${target_dir}" ]]; then
        logit "Directory ${target_dir} does not exist. Skipping."
        return 0
    fi

    logit "Cleaning ${target_dir} | older than ${max_days} days | pattern: ${pattern}"

    # Use find to locate and safely remove files
    find "${target_dir}" -maxdepth 3 -type f -name "${pattern}" -mtime +"${max_days}" -print0 | while IFS= read -r -d '' file; do
        if pgrep -f "cyber_recorder" > /dev/null; then
             logit "AD task started! Abort deletion of $file."
             exit 0
        fi
        rm -f "${file}"
        logit "Deleted: ${file}"
        sleep 0.1 # I/O throttling
    done
}

logit "Initiating cleanup..."

clean_directory "${TARGET_BAG_DIR}" "${BAG_RETENTION_DAYS}" "*.record.*"

clean_directory "${TARGET_LOG_DIR}" "${LOG_RETENTION_DAYS}" "*.log.*"
clean_directory "${TARGET_LOG_DIR}" "${LOG_RETENTION_DAYS}" "*.out"
clean_directory "${TARGET_LOG_DIR}" "${LOG_RETENTION_DAYS}" "*.INFO*"
clean_directory "${TARGET_LOG_DIR}" "${LOG_RETENTION_DAYS}" "*.WARNING*"
clean_directory "${TARGET_LOG_DIR}" "${LOG_RETENTION_DAYS}" "*.ERROR*"
clean_directory "${TARGET_LOG_DIR}" "${LOG_RETENTION_DAYS}" "*.FATAL*"

clean_directory "${TARGET_CORE_DIR}" "${CORE_RETENTION_DAYS}" "core_*"
clean_directory "${TARGET_CORE_DIR}" "${CORE_RETENTION_DAYS}" "core.*"

logit "Cleanup finished."
