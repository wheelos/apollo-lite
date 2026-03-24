#!/usr/bin/env bash

###############################################################################
# Copyright 2017-2021 The Apollo Authors. All Rights Reserved.
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

set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 1. Source the base environment file
# This must be done first to load essential variables like SERVER_PORT
source "${DIR}/apollo_base.sh"

# 2. Dynamic Variable Setup
# SERVER_PORT is now available from apollo_base.sh (exported)
DREAMVIEW_URL="http://localhost:${SERVER_PORT}"
info "DREAMVIEW_URL is set to ${DREAMVIEW_URL}"

# 3. Change Directory (Crucial for relative path commands like ./scripts/monitor.sh)
cd "${DIR}/.."

# Function to print the last N lines of the unified application log file.
nohup_log() {
  info "--- Check /apollo/nohup.out for more details.  ---"
  tail -n 10 /apollo/nohup.out
}

function stop() {
  ./scripts/dreamview.sh stop
  ./scripts/monitor.sh stop
}

function start() {
  # Enforce clearing the log file to ensure log messages are from the current run
  > /apollo/nohup.out

  # 1. Start Monitor
  ./scripts/monitor.sh start || {
    error "ERROR: Monitor failed to start (Exit Code: $?). Cleaning up..."
    nohup_log
    stop
    return 1
  }

  # 2. Start Dreamview
  ./scripts/dreamview.sh start || {
    error "ERROR: Dreamview failed to start (Exit Code: $?). Cleaning up..."
    nohup_log
    stop
    return 1
  }

  # 3. Execute HTTP Health Check (Service availability check)
  sleep 2 # wait for service initialization

  local http_status
  # Using -w '%{http_code}' to capture status code
  http_status="$(curl -o /dev/null -x '' -I -L -s -w '%{http_code}' ${DREAMVIEW_URL})"

  # Robustness Check: Ensure status_code is a safe integer
  local status_code="${http_status}"
  if ! [[ "$status_code" =~ ^[0-9]+$ ]]; then
      status_code=0
  fi

  if [ "$status_code" -eq 200 ]; then
    info "SUCCESS: Dreamview is fully running at ${DREAMVIEW_URL}"
  else
    # Health check failed (HTTP Status != 200 or connection failure)
    error "ERROR: Dreamview service failed health check (HTTP Status: ${status_code}). Cleaning up..."

    # Print application runtime log for diagnosis
    nohup_log

    stop
    return 1
  fi
}

# --- Main Execution ---

case $1 in
  start)
    start
    ;;
  stop)
    stop
    ;;
  restart)
    stop
    start
    ;;
  *)
    start
    ;;
esac
