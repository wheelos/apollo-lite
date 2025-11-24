#!/usr/bin/env bash

# ==============================================================================
# Script Name: config_system.sh
# Description: Configures the host machine for autonomous driving development,
#              including core dump settings, Bazel cache, NTP synchronization,
#              udev rules, and uvcvideo module options.
#              Each step checks if the configuration is already in place before
#              proceeding. This script focuses on setup, not uninstallation.
# Author: WheelOS
# Date: June 24, 2025
# ==============================================================================

# --- Strict Mode ---
set -euo pipefail

# --- Global Variables ---
# Determine the absolute path to Apollo root directory.
# Assumes this script is located at <APOLLO_ROOT_DIR>/docker/setup_host/config_system.sh
APOLLO_ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# use relative path for multiple instances
CORE_DUMP_DIR="data/core"
CORE_DUMP_CONF_FILE="/etc/sysctl.d/99-core-dump.conf"
BAZEL_CACHE_DIR="/var/cache/bazel/repo_cache"
UVCVIDEO_CONF_FILE="/etc/modprobe.d/uvcvideo.conf"
UDEV_RULES_SRC_DIR="${APOLLO_ROOT_DIR}/docker/setup_host/etc/udev/rules.d"
UDEV_RULES_DEST_DIR="/etc/udev/rules.d"
LIMITS_CONF_SRC_DIR="${APOLLO_ROOT_DIR}/docker/setup_host/etc/security/limits.d"
LIMITS_CONF_DEST_DIR="/etc/security/limits.d"
NETWORK_CONF_SRC_DIR="${APOLLO_ROOT_DIR}/docker/setup_host/etc/systemd/network"
NETWORK_CONF_DEST_DIR="/etc/systemd/network"
SYSTEMD_SRC_DIR="${APOLLO_ROOT_DIR}/docker/setup_host/etc/systemd/system"
SYSTEMD_DEST_DIR="/etc/systemd/system"

# Source path of the service file within the project
AUTOSERVICE_SRC_FILE="${APOLLO_ROOT_DIR}/docker/setup_host/etc/systemd/system/autostart.service"
# Destination path for systemd services
AUTOSERVICE_DEST_FILE="/etc/systemd/system/autostart.service"
# User who will run the autonomous driving stack. SUDO_USER is the user who invoked sudo.
WHL_HOST_USER="${SUDO_USER:-$(whoami)}"

# --- Color Definitions for Output ---
BOLD='\033[1m'
RED='\033[0;31m'
GREEN='\033[0;32m'
WHITE='\033[34m'
NO_COLOR='\033[0m'

# --- Logging Functions ---
# Prints an informational message to stderr.
info() {
  (echo >&2 -e "[${WHITE}${BOLD}INFO${NO_COLOR}] $*")
}

# Prints a success message to stderr.
success() {
  (echo >&2 -e "[${GREEN}${BOLD}SUCCESS${NO_COLOR}] $*")
}

# Prints an error message to stderr.
error() {
  (echo >&2 -e "[${RED}${BOLD}ERROR${NO_COLOR}] $*")
}

# --- Pre-condition Check for the Entire Script ---
# This function checks essential prerequisites before starting any host setup.
# Returns 0 if all pre-conditions are met, 1 otherwise.
check_host_setup_pre_conditions() {
  info "Checking host setup pre-conditions..."

  # APT authentication will fail if the time is incorrect! so time
  # synchronization is required the first time.
  if ! command -v chronyc &>/dev/null; then
    if sudo timedatectl set-ntp true; then
      timedatectl status
    else
      error "Failed to enable NTP via timedatectl"
      timedatectl status
      return 1
    fi
  fi

  # 1. Check for root privileges
  if [ "$(id -u)" -ne 0 ]; then
    error "This script must be run with root privileges (sudo)."
    return 1
  fi

  # 2. Check if APOLLO_ROOT_DIR exists
  if [ ! -d "${APOLLO_ROOT_DIR}" ]; then
    error "APOLLO_ROOT_DIR ('${APOLLO_ROOT_DIR}') not found or is not a directory."
    error "Please ensure this script is run from within the Apollo project structure."
    return 1
  fi
  info "APOLLO_ROOT_DIR (${APOLLO_ROOT_DIR}) detected."

  # 3. Check for essential commands
  local required_cmds=("sudo" "sysctl" "systemctl" "udevadm" "tee" "lsmod" "modprobe" "grep" "mkdir" "cp" "ls")
  for cmd in "${required_cmds[@]}"; do
    if ! command -v "$cmd" &> /dev/null; then
      error "Required command '$cmd' not found. Please ensure it's installed and in PATH."
      return 1
    fi
  done
  info "All required commands are available."

  # 4. Check for existence of udev source rules
  if [ ! -d "${UDEV_RULES_SRC_DIR}" ] || [ -z "$(ls -A "${UDEV_RULES_SRC_DIR}")" ]; then
    error "Udev rules source directory '${UDEV_RULES_SRC_DIR}' not found or is empty."
    error "Please ensure Apollo's udev rules are present in the expected location."
    return 1
  fi
  info "Udev rules source directory detected."

  # 5. Check for existence of autostart service file
  if [ ! -f "${AUTOSERVICE_SRC_FILE}" ]; then
    error "Autostart service file not found at '${AUTOSERVICE_SRC_FILE}'."
    error "Please ensure the service definition file is present."
    return 1
  fi
  info "Autostart service source file detected."

  success "All host setup pre-conditions met."
  return 0
}

# Configures core dump settings.
setup_core_dump() {
  info "Setting up core dump format..."

  # Check if core dump configuration is already applied
  if [ -f "${CORE_DUMP_CONF_FILE}" ] && grep -q "kernel.core_pattern = ${CORE_DUMP_DIR}/core_%e.%p" "${CORE_DUMP_CONF_FILE}" &&
    [[ "$(sudo sysctl -n kernel.core_pattern)" == "${CORE_DUMP_DIR}/core_%e.%p" ]]; then
    info "Core dump configuration already detected and active. Skipping."
    return 0
  fi

  user="${SUDO_USER:-$(whoami)}"
  if [ ! -d "${CORE_DUMP_DIR}" ]; then
    sudo -u "${user}" mkdir -p "${CORE_DUMP_DIR}"
    if [ $? -ne 0 ]; then
      error "Failed to create core dump directory: ${CORE_DUMP_DIR}."
      return 1
    fi
    info "Created core dump directory: ${CORE_DUMP_DIR}."
  fi

  # Ensure the directory has appropriate permissions for core dumps
  sudo chmod 0777 "${CORE_DUMP_DIR}" || {
    error "Failed to set permissions for core dump directory: ${CORE_DUMP_DIR}."
    return 1
  }

  info "Writing core dump configuration to ${CORE_DUMP_CONF_FILE}..."
  sudo tee "${CORE_DUMP_CONF_FILE}" > /dev/null << EOF
kernel.core_pattern = ${CORE_DUMP_DIR}/core_%e.%p
EOF
  if [ $? -ne 0 ]; then
    error "Failed to write core dump configuration."
    return 1
  fi

  info "Applying sysctl configuration..."
  sudo sysctl -p "${CORE_DUMP_CONF_FILE}"
  if [ $? -ne 0 ]; then
    error "Failed to apply core dump sysctl configuration. Check file content or permissions."
    return 1
  fi

  success "Core dump configuration applied."
  return 0
}

# Creates the Bazel cache directory.
setup_bazel_cache_dir() {
  info "Creating Bazel cache directory..."

  # Check if Bazel cache directory already exists
  if [ -d "${BAZEL_CACHE_DIR}" ]; then
    info "Bazel cache directory '${BAZEL_CACHE_DIR}' already exists. Skipping creation."
  else
    sudo mkdir -p "${BAZEL_CACHE_DIR}"
    if [ $? -ne 0 ]; then
      error "Failed to create Bazel cache directory: ${BAZEL_CACHE_DIR}."
      return 1
    fi
    info "Created Bazel cache directory: ${BAZEL_CACHE_DIR}."
  fi

  # Ensure permissions are appropriate for a shared cache
  sudo chmod a+rwx "${BAZEL_CACHE_DIR}" || {
    error "Failed to set permissions for Bazel cache directory."
    return 1
  }

  success "Bazel cache directory configured."
  return 0
}

# Configures NTP synchronization using chrony.
  sudo timedatectl set-ntp true
configure_ntp() {
  info "Standardizing NTP synchronization on chrony..."

  # --- Step 1: Detect and disable conflicting NTP services ---
  if systemctl list-unit-files | grep -q 'systemd-timesyncd.service'; then
    if systemctl is-active --quiet systemd-timesyncd.service; then
      info "Conflict detected: systemd-timesyncd is active. Disabling it now."
      sudo systemctl disable --now systemd-timesyncd.service || warning "Failed to disable systemd-timesyncd."
    fi
  fi

  local ntpd_service
  ntpd_service=$(systemctl list-unit-files | grep -Eo '^(ntp|ntpd)\.service' | head -n 1)
  if [ -n "$ntpd_service" ]; then
    if systemctl is-active --quiet "$ntpd_service"; then
      info "Conflict detected: $ntpd_service is active. Disabling it now."
      sudo systemctl disable --now "$ntpd_service" || warning "Failed to disable $ntpd_service."
    fi
  fi

  # --- Step 2: Check for chrony, install if missing ---
  # 'command -v' is a reliable way to check if a command is in the system's PATH
  if ! command -v chronyc &> /dev/null; then
    info "chrony is not installed. Attempting to install via apt..."

    # Update package list and install chrony
    sudo apt-get update
    sudo apt-get install -y chrony

    if [ $? -ne 0 ]; then
      error "Failed to install 'chrony' via apt. Please check for errors above. Aborting."
      return 1
    fi
    success "'chrony' package installed successfully."

    # Reload systemd to recognize the new service file. This is crucial.
    info "Reloading systemd daemon..."
    sudo systemctl daemon-reload
  fi

  # --- Step 3: Ensure chrony service is enabled and active ---
  # Find the exact service name (usually 'chrony.service' on Debian/Ubuntu)
  local chrony_service
  chrony_service=$(systemctl list-unit-files | grep -Eo '^chrony(d)?\.service' | head -n 1)

  if [ -z "$chrony_service" ]; then
    error "chrony appears to be installed, but its systemd service could not be found. Investigation needed."
    return 1
  fi

  if systemctl is-enabled --quiet "$chrony_service" && systemctl is-active --quiet "$chrony_service"; then
    success "chrony ($chrony_service) is already enabled and running correctly."
  else
    info "Enabling and starting chrony service ($chrony_service)..."
    sudo systemctl enable --now "$chrony_service"
    if [ $? -ne 0 ]; then
      error "Failed to start $chrony_service. Diagnose with 'systemctl status $chrony_service'."
      return 1
    fi
    success "chrony ($chrony_service) has been enabled and started."
  fi

  # --- Step 4: Verify synchronization status ---
  info "Verifying chrony synchronization status..."
  # Allow a few seconds for chrony to establish a connection
  sleep 3
  chronyc tracking

  return 0
}

# Copies and applies udev rules.
apply_udev_rules() {
  info "Adding udev rules from '${UDEV_RULES_SRC_DIR}' to '${UDEV_RULES_DEST_DIR}'..."

  # Check if udev rules are already present (best-effort: check if all source files exist in dest)
  local all_udev_rules_present=true
  if [ -d "${UDEV_RULES_DEST_DIR}" ] && [ "$(ls -A "${UDEV_RULES_SRC_DIR}" | wc -l)" -gt 0 ]; then
    for rule_file in "${UDEV_RULES_SRC_DIR}"/*; do
      if [ -f "${rule_file}" ]; then
        local base_name=$(basename "${rule_file}")
        if [ ! -f "${UDEV_RULES_DEST_DIR}/${base_name}" ]; then
          all_udev_rules_present=false
          break
        fi
      fi
    done
  else
    all_udev_rules_present=false
  fi

  if "${all_udev_rules_present}"; then
    info "Udev rules already appear to be in place. Force copy."
  fi

  sudo cp -r "${UDEV_RULES_SRC_DIR}"/* "${UDEV_RULES_DEST_DIR}/"
  if [ $? -ne 0 ]; then
    error "Failed to copy udev rules. Check source/destination permissions or path."
    return 1
  fi
  info "Udev rules copied."

  info "Reloading udev rules and triggering devices..."
  sudo udevadm control --reload-rules
  if [ $? -ne 0 ]; then
    error "Failed to reload udev rules."
    return 1
  fi
  sudo udevadm trigger
  if [ $? -ne 0 ]; then
    error "Failed to trigger udev devices."
    return 1
  fi
  success "Udev rules applied."
  return 0
}

# Configures uvcvideo module options.
configure_uvcvideo_module() {
  info "Adding uvcvideo clock configuration to ${UVCVIDEO_CONF_FILE}..."

  # Check if uvcvideo clock configuration is already applied
  if [ -f "${UVCVIDEO_CONF_FILE}" ] && grep -q "options uvcvideo clock=realtime" "${UVCVIDEO_CONF_FILE}"; then
    info "uvcvideo clock configuration already detected. Skipping write."
  else
    sudo tee "${UVCVIDEO_CONF_FILE}" > /dev/null << EOF
options uvcvideo clock=realtime
EOF
    if [ $? -ne 0 ]; then
      error "Failed to write uvcvideo configuration."
      return 1
    fi
    info "uvcvideo configuration written."
  fi

  info "Reloading uvcvideo module if loaded..."
  if lsmod | grep -q uvcvideo; then
    sudo modprobe -r uvcvideo
    if [ $? -ne 0 ]; then
      error "Failed to unload uvcvideo module. Manual intervention may be needed."
      return 1
    fi
    info "uvcvideo module unloaded."
  else
    info "uvcvideo module not currently loaded. No unload needed."
  fi
  sudo modprobe uvcvideo
  if [ $? -ne 0 ]; then
    error "Failed to load uvcvideo module with new options. Check module status."
    return 1
  fi
  success "uvcvideo module configured and reloaded."
  return 0
}

# Adds the current user to the Docker group for permissions.
add_user_to_docker_group() {
  # Use SUDO_USER if available, otherwise fallback to whoami
  user="${SUDO_USER:-$(whoami)}"
  if [ -z "${user}" ]; then
    error "Could not determine the user to add to the Docker group. Please run this script with sudo."
    return 1
  fi

  if [[ "${user}" == "root" ]]; then
    info "Running as root. Skipping Docker group addition for root user."
    return 0
  fi

  info "Ensuring user '${user}' is in the Docker group..."

  if id -nG "${user}" | grep -qw 'docker'; then
    info "User '${user}' is already in the Docker group. Skipping."
    return 0
  fi

  sudo usermod -aG docker "${user}"
  if [ $? -ne 0 ]; then
    error "Failed to add user '${user}' to the Docker group. Check permissions or user existence."
    return 1
  fi

  success "User '${user}' added to the Docker group. Please log out and back in for changes to take effect."
}

# Installs and enables the autostart systemd service.
install_autostart_service() {
  info "Installing and configuring the autostart systemd service..."

  # 1: Hard Fail on Root Installation (Security Best Practice) ---
  # Running the main AD stack as root is a major security risk.
  # We now block this by default and require an explicit, intentional override.
  if [[ "${WHL_HOST_USER}" == "root" ]]; then
     error "Running the autonomous driving stack as 'root' is not allowed for security reasons."
     error "To override this critical check, set the environment variable ALLOW_ROOT_INSTALL=yes and re-run."
     if [[ "${ALLOW_ROOT_INSTALL:-no}" != "yes" ]]; then
         return 1 # Hard fail, aborting the installation.
     else
         info "ALLOW_ROOT_INSTALL=yes detected. Proceeding with installation as root. THIS IS NOT RECOMMENDED."
     fi
  fi

  # 2: Input Validation for Username (Prevent Command Injection) ---
  # Ensure the username consists only of safe, alphanumeric characters.
  if ! [[ "${WHL_HOST_USER}" =~ ^[a-zA-Z0-9_][a-zA-Z0-9_-]*$ ]]; then
    error "Invalid username detected: '${WHL_HOST_USER}'. Usernames must be alphanumeric (with _ or -)."
    error "Aborting installation to prevent potential command injection."
    return 1
  fi
  info "Target user '${WHL_HOST_USER}' validated."

  # 3: Input Validation for Path (Prevent Command Injection) ---
  # Ensure the path is a valid absolute path and does not contain characters
  # that could break the sed command or be interpreted by the shell.
  if ! [[ "${APOLLO_ROOT_DIR}" =~ ^/[a-zA-Z0-9_/.-]+$ ]]; then
    error "Invalid APOLLO_ROOT_DIR detected: '${APOLLO_ROOT_DIR}'."
    error "Path must be absolute and contain only safe characters (alphanumeric, /, _, ., -)."
    error "Aborting installation to prevent potential command injection."
    return 1
  fi
  info "Apollo root directory '${APOLLO_ROOT_DIR}' validated."

  info "Copying '${AUTOSERVICE_SRC_FILE}' to '${AUTOSERVICE_DEST_FILE}'..."
  if ! cp "${AUTOSERVICE_SRC_FILE}" "${AUTOSERVICE_DEST_FILE}"; then
    error "Failed to copy service file. Check permissions."
    return 1
  fi

  # 4: Safer Replacement Method ---
  # By using a different delimiter for sed (like '#'), we make the replacement
  # robust even if the path contains the default '/' delimiter.
  # The validation above already mitigates this, but this is a defense-in-depth measure.
  info "Customizing service file with user='${WHL_HOST_USER}' and path='${APOLLO_ROOT_DIR}'..."
  if ! sed -i "s#__USER__#${WHL_HOST_USER}#g" "${AUTOSERVICE_DEST_FILE}"; then
    error "Failed to replace user placeholder in service file."
    return 1
  fi
  if ! sed -i "s#__APOLLO_ROOT_DIR__#${APOLLO_ROOT_DIR}#g" "${AUTOSERVICE_DEST_FILE}"; then
    error "Failed to replace path placeholder in service file."
    return 1
  fi

  # Reload the systemd daemon
  info "Reloading systemd daemon..."
  if ! systemctl daemon-reload; then
    error "Failed to reload systemd daemon. Run 'journalctl -xe' for details."
    return 1
  fi

  # Enable the service
  info "Enabling 'autostart.service' to run on boot..."
  if ! systemctl enable autostart.service; then
    error "Failed to enable autostart.service."
    return 1
  fi

  success "Autostart service installed and enabled successfully."
  info "The autonomous driving system will now start automatically on the next boot."
  return 0
}

# Sets the system-wide limits for core dump file size by copying config.
setup_core_limits() {
    info "Configuring system-wide core dump size limits..."
    if [ ! -d "${LIMITS_CONF_SRC_DIR}" ] || [ -z "$(ls -A "${LIMITS_CONF_SRC_DIR}")" ]; then
        info "Core limits source directory not found or empty. Skipping."
        return 0
    fi
    sudo cp -f "${LIMITS_CONF_SRC_DIR}"/* "${LIMITS_CONF_DEST_DIR}/"
    if [ $? -ne 0 ]; then error "Failed to copy core limits configuration." && return 1; fi
    success "Core dump size limits configured."
    return 0
}

# Configures CAN bus using systemd-networkd for permanent setup.
configure_can_bus() {
    info "Configuring CAN bus interfaces using systemd-networkd..."
    if [ ! -d "${NETWORK_CONF_SRC_DIR}" ] || [ -z "$(ls -A "${NETWORK_CONF_SRC_DIR}")" ]; then
        info "systemd-networkd source directory not found or empty. Skipping CAN bus setup."
        return 0
    fi

    sudo mkdir -p "${NETWORK_CONF_DEST_DIR}"
    # This robustly copies any .network files present (e.g., just 10-can0.network)
    sudo cp -f "${NETWORK_CONF_SRC_DIR}"/*.network "${NETWORK_CONF_DEST_DIR}/"
    if [ $? -ne 0 ]; then error "Failed to copy systemd-networkd CAN configurations." && return 1; fi
    info "CAN network configuration files copied."

    info "Enabling and restarting systemd-networkd service..."
    sudo systemctl enable --now systemd-networkd.service > /dev/null 2>&1
    sudo systemctl restart systemd-networkd.service
    if [ $? -ne 0 ]; then error "Failed to restart systemd-networkd. Check its status." && return 1; fi
    success "CAN bus interfaces configured via systemd-networkd."
    return 0
}

# Sets up Jetson performance optimizations (nvpmodel, jetson_clocks).
setup_jetson_performance() {
    if ! command -v nvpmodel &> /dev/null; then
        info "Not a Jetson device. Skipping Jetson performance setup."
        return 0
    fi

    info "Configuring Jetson performance services (nvpmodel and jetson_clocks)..."
    local service_file="jetson-performance.service"

    if [ ! -f "${SYSTEMD_SRC_DIR}/${service_file}" ]; then
        error "'${service_file}' not found in source directory. Skipping." && return 1
    fi

    sudo cp -f "${SYSTEMD_SRC_DIR}/${service_file}" "${SYSTEMD_DEST_DIR}/"
    if [ $? -ne 0 ]; then error "Failed to copy ${service_file}." && return 1; fi

    sudo systemctl daemon-reload
    sudo systemctl enable --now "${service_file}"
    if [ $? -ne 0 ]; then error "Failed to enable and start ${service_file}." && return 1; fi
    success "Jetson performance service enabled and started."
    return 0
}

# Switches the system to non-graphical multi-user mode (headless).
configure_headless_mode() {
    # This is a significant change, only run if explicitly requested.
    if [[ "${SETUP_HEADLESS_MODE:-no}" != "yes" ]]; then
        info "SETUP_HEADLESS_MODE is not 'yes'. Skipping GUI disable."
        return 0
    fi

    info "Configuring system for headless operation (disabling GUI)..."
    # Check if it's already in the desired state (idempotency)
    if systemctl get-default | grep -q "multi-user.target"; then
        info "System is already set to multi-user (headless) mode."
        return 0
    fi

    sudo systemctl set-default multi-user.target
    if [ $? -ne 0 ]; then
        error "Failed to set default target to multi-user.target."
        return 1
    fi

    success "System configured for headless mode. GUI will be disabled on next reboot."
    info "To revert this change, run: sudo systemctl set-default graphical.target"
    return 0
}


# --- Main Host Setup Orchestration Function ---
# This is the primary entry point for setting up the host machine.
setup_host_machine() {
  info "Starting host machine setup process..."

  # 1. Check pre-conditions before proceeding with setup
  if ! check_host_setup_pre_conditions; then
    error "Host setup pre-conditions not met. Aborting host setup."
    return 1 # Exit with error as pre-conditions failed
  fi

  # 2. Configure core dump settings
  if ! setup_core_dump; then
    error "Failed to configure core dump settings. Aborting host setup."
    return 1
  fi

  if ! setup_core_limits; then
    error "Failed to configure core limits."
    return 1
  fi

  # 3. Setup Bazel cache directory
  if ! setup_bazel_cache_dir; then
    error "Failed to set up Bazel cache directory. Aborting host setup."
    return 1
  fi

  # 4. Configure NTP synchronization
  if ! configure_ntp; then
    error "Failed to configure NTP synchronization. Aborting host setup."
    return 1
  fi

  # 5. Apply udev rules
  if ! apply_udev_rules; then
    error "Failed to apply udev rules. Aborting host setup."
    return 1
  fi

  # 6. Configure uvcvideo module
  if ! configure_uvcvideo_module; then
    error "Failed to configure uvcvideo module. Aborting host setup."
    return 1
  fi

  if ! configure_can_bus; then
    error "Failed to configure CAN bus."
    return 1
  fi

  if ! setup_jetson_performance; then
    error "Failed to configure Jetson performance."
    return 1
  fi

  if ! configure_headless_mode; then
    error "Failed to configure headless mode."
    return 1
  fi

  # 7. Ensure docker permissions are set correctly
  if ! add_user_to_docker_group; then
    error "Failed to add user to Docker group. This may affect Docker permissions."
    return 1
  fi

  # 8. Install and enable the autostart service
  if ! install_autostart_service; then
    error "Failed to install the autostart service. Host setup is incomplete."
    return 1
  fi

  success "Host machine setup completed successfully!"
  return 0 # Final success
}

# --- Script Entry Point ---
# Handles command-line arguments to either setup or uninstall host configurations.
main() {
  # This script now only supports 'install' mode for setting up the host.
  # If no argument is provided, it defaults to install.
  # Any other argument will result in an error message.
  if [ "$#" -eq 0 ] || [ "$1" == "install" ]; then
    if [ "$#" -eq 0 ]; then
      info "No argument provided. Defaulting to 'install' mode."
    fi
    setup_host_machine
  else
    error "Invalid argument: $1"
    info "This script is designed for 'install' mode only. Uninstallation is not supported."
    info "Usage: $0 [install]"
    return 1 # Return 1 for invalid arguments
  fi
}

# Execute the main function with all command-line arguments
main "$@"
