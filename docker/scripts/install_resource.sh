#!/usr/bin/env bash

# =============================================================================
# Apollo Perception Model Installation Script (Container Side)
#
# This script is intended to be run *inside* the Apollo development container
# to download and install default perception models using the 'amodel' tool.
# It will attempt to install ALL default models defined in the script.
#
# IMPORTANT: This script assumes the 'amodel' tool is ALREADY INSTALLED
#           and accessible in the container's PATH before execution.
#           The best practice is to install 'amodel' during the Dockerfile build.
# =============================================================================

# --- Script Setup and Error Handling ---
# Exit immediately if a command exits with a non-zero status.
set -e
# Treat unset variables as an error when substituting.
set -u
# Exit if any command in a pipeline fails.
set -o pipefail

# --- Logging Functions ---
# Simple logging functions (can be replaced if global functions exist)
# Using color codes for better visibility in terminal
COLOR_GREEN="\033[0;32m"
COLOR_YELLOW="\033[0;33m"
COLOR_RED="\033[0;31m"
COLOR_BLUE="\033[0;34m"
COLOR_NC="\033[0m" # No Color

info() {
  echo -e "${COLOR_BLUE}[INFO]$(date '+%Y-%m-%d %H:%M:%S')${COLOR_NC} $@"
}

warning() {
  echo -e "${COLOR_YELLOW}[WARN]$(date '+%Y-%m-%d %H:%M:%S')${COLOR_NC} $@" >&2 # Output warnings to stderr
}

error() {
  echo -e "${COLOR_RED}[ERROR]$(date '+%Y-%m-%d %H:%M:%S')${COLOR_NC} $@" >&2 # Output errors to stderr
}

ok() {
  echo -e "${COLOR_GREEN}[OK]$(date '+%Y-%m-%d %H:%M:%S')${COLOR_NC} $@"
}

# --- Constants ---
# Assuming standard Apollo container setup
APOLLO_ROOT_DIR="/apollo" # Although APOLLO_ROOT_DIR is not used in this specific script logic, keep if part of a larger context.

# Define the model repository base URL
MODEL_REPOSITORY="https://apollo-pkg-beta.cdn.bcebos.com/perception_model"

# Define the list of default models to install
# Using a readonly array is a good practice for constants
readonly DEFAULT_INSTALL_MODEL=(
    "${MODEL_REPOSITORY}/tl_detection_caffe.zip"
    "${MODEL_REPOSITORY}/horizontal_caffe.zip"
    "${MODEL_REPOSITORY}/quadrate_caffe.zip"
    "${MODEL_REPOSITORY}/vertical_caffe.zip"
    "${MODEL_REPOSITORY}/darkSCNN_caffe.zip"
    "${MODEL_REPOSITORY}/cnnseg16_caffe.zip"
    "${MODEL_REPOSITORY}/3d-r4-half_caffe.zip"
)

# --- Prerequisite Check ---
info "Checking for required tools..."
# Check if amodel tool is available in PATH
if ! command -v amodel &> /dev/null; then
    error "'amodel' command not found. This script requires 'amodel' to be pre-installed."
    error "Please ensure 'amodel' is installed in the container (ideally via Dockerfile or a separate tool setup)."
    exit 1 # Exit if the prerequisite tool is missing
fi
info "'amodel' tool found: $(command -v amodel)" # Report the path to amodel

# --- Model Installation Logic ---
info "Starting perception model installation inside the container..."

# Initialize counters for summary
install_success_count=0
install_fail_count=0
failed_models=() # Array to store failed model URLs

# Check if the list is empty
if [ "${#DEFAULT_INSTALL_MODEL[@]}" -eq 0 ]; then
    warning "No models defined in the DEFAULT_INSTALL_MODEL list to install."
else
    info "Attempting to install ${#DEFAULT_INSTALL_MODEL[@]} models defined in the list."

    # Loop through the list and install each model
    for model_url in "${DEFAULT_INSTALL_MODEL[@]}"; do
        info " Processing model: ${model_url}"

        if amodel install "${model_url}" -s; then
            info "  Successfully installed model: ${model_url}."
            install_success_count=$((install_success_count + 1))
        else
            warning "  Failed to install model: ${model_url}. Installation for this model skipped."
            install_fail_count=$((install_fail_count + 1))
            failed_models+=("${model_url}") # Add to failed list
        fi
    done

    info "Finished attempting to install models."

    # --- Installation Summary ---
    echo # Add a newline for readability
    info "--- Installation Summary ---"
    info "Total models attempted: ${#DEFAULT_INSTALL_MODEL[@]}"
    info "Successfully installed: ${install_success_count}"
    info "Failed to install: ${install_fail_count}"

    if [ "${install_fail_count}" -gt 0 ]; then
        warning "Details of failed models:"
        for failed_url in "${failed_models[@]}"; do
            warning "  - ${failed_url}"
        done
        # Exit with a non-zero status if any installation failed,
        # indicating partial success/failure of the script's goal.
        # Adjust this if the script must *only* exit 0 on 100% success.
        # For this example, we'll exit 1 on any failure.
        # exit 1 # Uncomment this line if you want the *entire script* to fail if *any* model fails
    fi
fi

ok "Perception model installation script finished."

# If you chose not to exit 1 on individual model failures in the loop,
# but want the script overall to report failure if ANY model failed:
if [ "${install_fail_count}" -gt 0 ]; then
    exit 1
fi
