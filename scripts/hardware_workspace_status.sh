#!/usr/bin/env bash
set -euo pipefail

printf 'STABLE_HARDWARE_REVISION %s\n' "$(git rev-parse HEAD)"
fingerprint="$(
  {
    git diff --binary HEAD --
    git ls-files --others --exclude-standard -z -- \
      tools/hardware tools/modules tools/products scripts/hardware_workspace_status.sh \
      scripts/ci/apollo_product.py \
      scripts/ci/hardware_build_benchmark.py scripts/ci/tests \
      modules/drivers/gnss/parser modules/canbus/vehicle |
      while IFS= read -r -d '' path; do
        sha256sum "${path}"
      done
  } | sha256sum
)"
printf 'STABLE_HARDWARE_SOURCE_FINGERPRINT %s\n' "${fingerprint%% *}"
if git diff --quiet HEAD --; then
  echo 'STABLE_HARDWARE_TRACKED_STATE clean'
else
  echo 'STABLE_HARDWARE_TRACKED_STATE dirty'
fi
