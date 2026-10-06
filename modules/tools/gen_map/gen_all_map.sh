#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
APOLLO_ROOT_DIR="${APOLLO_ROOT_DIR:-$(cd "${SCRIPT_DIR}/../../.." && pwd -P)}"
if [[ "${APOLLO_ROOT_DIR}" != /* ]]; then
  echo "Error: APOLLO_ROOT_DIR must be an absolute path."
  exit 1
fi
WHEELOS_ASSET_ROOT="${WHEELOS_ASSET_ROOT:-${APOLLO_ROOT_DIR}/assets}"
if [[ "${WHEELOS_ASSET_ROOT}" != /* ]]; then
  echo "Error: WHEELOS_ASSET_ROOT must be an absolute path."
  exit 1
fi
APOLLO_MAP_PATH="${WHEELOS_ASSET_ROOT}/sites/map"

# Check if the map directory is provided as an argument
if [ -z "$1" ]; then
  echo "Usage: $0 <your_map_directory>"
  exit 1
fi

# Define your map directory name from the input argument
YOUR_MAP_DIR="$1"

# Check if the source map directory exists
if [ ! -d "${YOUR_MAP_DIR}" ]; then
  echo "Error: The source map directory '${YOUR_MAP_DIR}' does not exist."
  exit 1
fi

MAP_NAME=$(basename "${YOUR_MAP_DIR}")

# Construct the full path to your map directory
NEW_MAP_PATH="${APOLLO_MAP_PATH}/${MAP_NAME}"

mkdir -p "${APOLLO_MAP_PATH}"

if [ -d "${NEW_MAP_PATH}" ]; then
  echo "Warning: The directory ${NEW_MAP_PATH} already exists. It will be removed."
  rm -rf "${NEW_MAP_PATH}"
fi

cp -r "${YOUR_MAP_DIR}/." "${NEW_MAP_PATH}"

# Define the executable file path for topo_creator
TOPO_CREATOR="./bazel-bin/modules/routing/topo_creator/topo_creator"

# Run the map generator
echo "Generating map files to: ${NEW_MAP_PATH}"
bazel run @wheelos_map//modules/map/tools:sim_map_generator -- \
  -map_dir="${NEW_MAP_PATH}" -output_dir="${NEW_MAP_PATH}" -downsample_distance=1

# Check if the map generator ran successfully (optional, but recommended)
if [ $? -ne 0 ]; then
  echo "Error: Map generator failed to run."
  exit 1
fi

# Run the topology creator
echo "Creating topology information based on the map: ${NEW_MAP_PATH}"
"${TOPO_CREATOR}" -map_dir="${NEW_MAP_PATH}"

# Check if the topology creator ran successfully (optional, but recommended)
if [ $? -ne 0 ]; then
  echo "Error: Topology creator failed to run."
  exit 1
fi

echo "The map is located at: ${NEW_MAP_PATH}"
