#!/usr/bin/env bash

###############################################################################
# Copyright 2017 The Apollo Authors. All Rights Reserved.
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

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
. "${TOP_DIR}/scripts/apollo_base.sh"

# --- Configuration ---
# Target link path matches the gflags default: --static_file_dir
FIXED_FRONTEND_LINK="${TOP_DIR}/modules/dreamview/frontend/dist"
DREAMVIEW_BIN="${APOLLO_BIN_PREFIX}/modules/dreamview/dreamview"

# --- 1. Locate Physical Path ---
# Dynamically find the real path in Bazel output_base to bypass Bzlmod mangled hashes
OUTPUT_BASE=$(bazel info output_base 2>/dev/null)
REAL_PATH=$(ls -d ${OUTPUT_BASE}/external/*dreamview_frontend_assets*/dist 2>/dev/null | head -n 1)

# --- 2. Update Symlink ---
if [ -d "${REAL_PATH}" ]; then
    echo "Updating frontend symlink: ${FIXED_FRONTEND_LINK} -> ${REAL_PATH}"

    # Remove existing directory or symlink to prevent nested links (e.g., dist/dist)
    # This ensures FIXED_FRONTEND_LINK points directly to the asset contents
    rm -rf "${FIXED_FRONTEND_LINK}"

    # Ensure the parent directory exists before creating the link
    mkdir -p "$(dirname "${FIXED_FRONTEND_LINK}")"

    # Create a symbolic link to the stable logical path
    ln -snf "${REAL_PATH}" "${FIXED_FRONTEND_LINK}"
else
    error "Frontend assets not found! Please ensure '@dreamview_frontend_assets' is correctly fetched."
    exit 1
fi

# --- 3. Execute Module ---
run_module dreamview "$@"
