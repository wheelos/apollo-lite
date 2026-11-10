#!/usr/bin/env bash

set -euo pipefail

CURR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
. "${CURR_DIR}/installer_base.sh"

apt_get_update_and_install shellcheck
pip3_install -r "${CURR_DIR}/lint_requirements.txt"

apt-get clean &&
    rm -rf /var/lib/apt/lists/*
