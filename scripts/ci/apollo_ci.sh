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

set -e

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"
source "${TOP_DIR}/scripts/apollo.bashrc"

APOLLO_LINT_SH="${APOLLO_ROOT_DIR}/scripts/ci/apollo_lint.sh"

function run_ci_lint() {
  if [[ "$#" -eq 0 ]]; then
    bash "${APOLLO_LINT_SH}" --lint --diff
  else
    bash "${APOLLO_LINT_SH}" --lint "$@"
  fi
}

function main() {
  local cmd="${1:-lint}"
  if [[ "${cmd}" == "lint" || "${cmd}" == "build" || "${cmd}" == "test" ]]; then
    shift
    info "Running CI lint only ..."
    run_ci_lint "$@"
    success "ci lint finished."
    return 0
  fi

  error "Unsupported CI command: ${cmd}. CI runs lint only."
  return 1
}

main "$@"
