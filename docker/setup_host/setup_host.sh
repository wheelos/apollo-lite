#!/bin/bash

# ==============================================================================
# Script Name: setup_host.sh
# Description: Orchestrates the setup of an autonomous driving host environment.
#              It sequentially installs Docker, NVIDIA Container Toolkit, and
#              performs host system configurations, with robust error handling
#              and pre-condition checks.
# Author: WheelOS
# Date: June 24, 2025
# ==============================================================================

set -euo pipefail

# --- Variables ---
# Define paths to the sub-scripts. Ensure these scripts are in the same directory
# as this main script, or provide their absolute paths.
APOLLO_ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../.." && pwd )"
INSTALL_DOCKER_SCRIPT="${APOLLO_ROOT_DIR}/docker/setup_host/install_docker.sh"
INSTALL_NVIDIA_TOOLKIT_SCRIPT="${APOLLO_ROOT_DIR}/docker/setup_host/install_nvidia_container_toolkit.sh"
SETUP_HOST_SCRIPT="${APOLLO_ROOT_DIR}/docker/setup_host/config_system.sh"
ISSUES_LINK="https://github.com/wheelos/apollo-lite"
SUPPORT_EMAIL="support@wheelos.cn"
HOST_READY_MARKER="/etc/wheelos_setup_host.done"

# Defaults for config control (can be overridden by env vars)
# If SKIP_SYSTEM_CONFIG=1, we skip system config; otherwise run it.
if [[ "${SKIP_SYSTEM_CONFIG:-0}" == "1" ]]; then
    RUN_SYSTEM_CONFIG=0
else
    RUN_SYSTEM_CONFIG=1
fi

ASCII_WHEELOS=$(cat <<'EOF'
           __              __
 _      __/ /_  ___  ___  / /___  _____
| | /| / / __ \/ _ \/ _ \/ / __ \/ ___/
| |/ |/ / / / /  __/  __/ / /_/ (__  )
|__/|__/_/ /_/\___/\___/_/\____/____/
EOF
)

# --- Functions ---

# Checks if a script file exists at the given path.
# Args:
#   $1: Path to the script file.
#   $2: Name of the script for error reporting.
check_script_existence() {
    local script_path="$1"
    local script_name="$2"
    if [ ! -f "${script_path}" ]; then
        echo "❌ ERROR: Script '${script_name}' not found at '${script_path}'. Please ensure the file exists and the path is correct."
        exit 1
    fi
}

# Checks the status of the last executed command.
# If the command failed, it prints an error message and exits the main script.
# This function assumes the sub-script will print specific error details to stderr/stdout
# before exiting with a non-zero status.
# Args:
#   $1: exit_code of the step/sub-script that was just executed.
#   $2: Name of the step/sub-script that was just executed.
check_status() {
    local exit_code="$1"
    local step_name="$2"
    if [ "$exit_code" -ne 0 ]; then
        echo "❌ ERROR: Step '$step_name' failed. Review the output above for details."
        echo "       Configuration process interrupted."
        exit 1
    fi
}

# Install a lightweight host-level `whl` command by symlinking the repository
# script `docker/scripts/whl.sh` into `/usr/local/bin/whl`. This makes the
# `whl` helper available to other scripts and to interactive users.
install_whl_system_command() {
    local script_path="${APOLLO_ROOT_DIR}/docker/scripts/whl.sh"
    local link_path="/usr/local/bin/whl"

    # If whl already exists on PATH, do nothing.
    if command -v whl >/dev/null 2>&1; then
        echo "✅ 'whl' already available on PATH: $(command -v whl)"
        return 0
    fi

    if [ ! -f "${script_path}" ]; then
        echo "⚠️  whl helper script not found at ${script_path}; skipping installation of system 'whl' command."
        return 0
    fi

    echo "🔧 Installing system 'whl' command -> ${link_path} (requires sudo)"
    sudo ln -sf "${script_path}" "${link_path}" || true
    sudo chmod +x "${script_path}" "${link_path}" || true
    echo "✅ Installed 'whl' -> ${link_path}"
}

show_welcome() {
    echo "${ASCII_WHEELOS}"
    echo ""
    echo "GitHub: ${ISSUES_LINK}"
    echo "Email : ${SUPPORT_EMAIL}"
    echo ""
}

# --- Main Script Flow ---

show_welcome
echo "Please ensure you run this script with a user that has sudo privileges."
echo ""
echo "The script will perform the following steps sequentially:"
echo "1. Install Docker (checks if already installed, then proceeds)"
echo "2. Install NVIDIA Container Toolkit (checks if already installed, depends on Docker)"
echo "3. Perform host system configurations (optional, can be skipped)"
echo ""


# --- Step 1: Install Docker ---
echo "--- 🛠️ Step 1/3: Executing Docker installation script... ---"
echo "This step will install or update Docker CE. It will skip if Docker is already installed."

# Check if the Docker installation script exists
check_script_existence "${INSTALL_DOCKER_SCRIPT}" "Docker Installation Script"

# Execute the sub-script. Sub-script is responsible for 'already installed' checks,
# pre-conditions, and detailed error messages.
sudo bash "${INSTALL_DOCKER_SCRIPT}" install
exit_code=$?
check_status $exit_code "Docker Installation" # Checks the exit status of install_docker.sh

echo "✅ Step 1/3: Docker installation/check complete."
echo ""

# --- Step 2: Install NVIDIA Container Toolkit ---
# Dependency: NVIDIA Container Toolkit requires Docker to be installed and running.
echo "--- 🛠️ Step 2/3: Executing NVIDIA Container Toolkit installation script... ---"
echo "This step will configure your system for NVIDIA GPU accelerated containers with Docker."
echo "It will check for existing installation and Docker pre-requisites."

# Check if the NVIDIA Container Toolkit installation script exists
check_script_existence "${INSTALL_NVIDIA_TOOLKIT_SCRIPT}" "NVIDIA Container Toolkit Installation Script"

# Execute the sub-script. Sub-script is responsible for its own pre-conditions (e.g., Docker existence).
sudo bash "${INSTALL_NVIDIA_TOOLKIT_SCRIPT}"
exit_code=$?
check_status $exit_code "NVIDIA Container Toolkit Installation"

echo "✅ Step 2/3: NVIDIA Container Toolkit installation/check complete."
echo ""

# Ensure whl command exists even when system config step is skipped.
install_whl_system_command
echo ""

# --- Step 3: Setup Host Environment ---
if [[ "${RUN_SYSTEM_CONFIG}" -eq 0 ]]; then
    echo "--- ⏭️ Step 3/3: Host environment setup skipped by request. ---"
    echo "Skipped script: ${SETUP_HOST_SCRIPT}"
    echo "Note: This may skip optional tuning such as udev, PTP, CAN, and auto-start service."
    echo ""
else
    echo "--- 🛠️ Step 3/3: Executing host environment setup script... ---"
    echo "This step performs additional system-level configurations (e.g., network, permissions)."

    # Check if the Host Setup script exists
    check_script_existence "${SETUP_HOST_SCRIPT}" "Host Setup Script"

    # Execute the sub-script (interactive prompts will control optional steps).
    sudo bash "${SETUP_HOST_SCRIPT}"
    exit_code=$?
    check_status $exit_code "Host Environment Setup"

    echo "✅ Step 3/3: Host Environment Setup complete."
    echo ""
fi

echo "--- 🎉 Configuration complete. ---"
echo "--- ⚠️ Close and reopen your terminal to apply changes! ---"

# Mark host setup completion
sudo touch "${HOST_READY_MARKER}"
