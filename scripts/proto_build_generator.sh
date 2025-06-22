#!/usr/bin/env bash

###############################################################################
# Copyright 2022 The Apollo Authors. All Rights Reserved.
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

# Define the root directories to search for .proto files.
# These directories are typically the top-level directories containing your protobuf definitions.
roots=(cyber modules modules/common_msgs)

# --- Main Logic ---

echo "Starting protobuf BUILD file generation..."
echo "Searching for .proto files in roots: ${roots[*]}"


find "${roots[@]}" -type f -name '*.proto' -printf '%h\0' \
  | sort -zu \
  | while IFS= read -r -d '' dir; do
      # For each unique directory found that contains .proto files:
      echo "--> Generating BUILD file for directory: $dir"

      # Call the external Python script to generate the BUILD file.
      # The script is expected to create or update '$dir/BUILD'.
      # Ensure proto_build_generator.py is robust and handles errors appropriately.
      if ! python3 scripts/proto_build_generator.py "$dir/BUILD"; then
        echo "Error: Failed to generate BUILD file for $dir." >&2
        exit 1 # Exit if the generator script fails
      fi
    done

echo "Finished protobuf BUILD file generation."
