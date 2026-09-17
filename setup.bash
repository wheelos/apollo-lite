#!/usr/bin/env bash

# Responsibility: provide the single public runtime environment entrypoint.
# Runtime discovery and variable setup are implemented by runtime_env.sh.

# Public and canonical runtime environment entrypoint.
# scripts/runtime_env.sh remains a compatibility implementation path for
# existing callers; new callers should source this file.

APOLLO_LITE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
export APOLLO_LITE_ROOT

source "${APOLLO_LITE_ROOT}/scripts/runtime_env.sh"
