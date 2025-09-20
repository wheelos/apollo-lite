#!/usr/bin/env bash

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

# Fail on first error.
set -euo pipefail

CURR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
. "${CURR_DIR}/installer_base.sh"

# -----------------------------------------------------------------------------
# Global Variables and Dependency Definitions (Moved inside function where applicable,
# or kept global if truly global to the script's execution context.)
#
# IMPORTANT: INSTALLED_DEPS should ideally be cleared or managed per function
# call if this function is called multiple times in a single script execution
# with different goals. For a single execution, keeping it global is fine.
# -----------------------------------------------------------------------------

# Keep track of installed dependencies to prevent redundant calls.
# Keys are dependency script filenames (e.g., "install_opencv.sh").
declare -A INSTALLED_DEPS

# Global dependencies that are commonly used across many modules.
declare -a GLOBAL_DEPS=(
    # Add other truly global dependencies here.
)

# Define dependencies for each module as separate arrays.
# Dependencies are listed without the CURR_DIR prefix, just the script names.
declare -a module_common_deps=("install_osqp.sh")
declare -a module_canbus_deps=()
declare -a module_control_deps=()
declare -a module_dreamview_deps=("install_dreamview_deps.sh")
declare -a module_drivers_deps=("install_drivers_deps.sh")
declare -a module_guardian_deps=()
declare -a module_localization_deps=("install_proj.sh")
declare -a module_map_deps=("install_proj.sh")
declare -a module_monitor_deps=()
declare -a module_perception_deps=("install_paddle_deps.sh" "install_ffmpeg.sh" "install_opencv.sh")
declare -a module_planning_deps=("install_adolc.sh" "install_ipopt.sh" "install_libtorch.sh" "install_opencv.sh")
declare -a module_prediction_deps=("install_opencv.sh")
declare -a module_routing_deps=()
declare -a module_storytelling_deps=()
declare -a module_task_manager_deps=()
declare -a module_tools_deps=("install_python_modules.sh")
declare -a module_transform_deps=()
# declare -a module_audio_deps=("install_fftw3.sh") # Uncomment if 'audio' module is active

# Map module names to the names of their dependency arrays.
declare -A MODULE_TO_DEPS_ARRAY_MAP=(
    ["common"]="module_common_deps"
    ["canbus"]="module_canbus_deps"
    ["control"]="module_control_deps"
    ["dreamview"]="module_dreamview_deps"
    ["drivers"]="module_drivers_deps"
    ["guardian"]="module_guardian_deps"
    ["localization"]="module_localization_deps"
    ["map"]="module_map_deps"
    ["monitor"]="module_monitor_deps"
    ["perception"]="module_perception_deps"
    ["planning"]="module_planning_deps"
    ["prediction"]="module_prediction_deps"
    ["routing"]="module_routing_deps"
    ["storytelling"]="module_storytelling_deps"
    ["task_manager"]="module_task_manager_deps"
    ["tools"]="module_tools_deps"
    ["transform"]="module_transform_deps"
    # ["audio"]="module_audio_deps" # Uncomment if 'audio' module is active
)

# List of all known modules, used for validation and ' --all ' option.
declare -a ALL_MODULES=("${!MODULE_TO_DEPS_ARRAY_MAP[@]}")


# -----------------------------------------------------------------------------
# Helper Functions (These remain global as they are utilities)
# -----------------------------------------------------------------------------

# Function to execute a dependency installation script only once.
# It assumes the target script itself is idempotent.
# Arguments:
#   $1: The dependency script filename (e.g., "install_opencv.sh")
function install_dep() {
    local dep_script_name="$1"
    local dep_script_path="${CURR_DIR}/${dep_script_name}"

    if [[ -z "${INSTALLED_DEPS[${dep_script_name}]:-}" ]]; then
        info "Attempting to install dependency: [${dep_script_name}] ..."
        if [[ -f "${dep_script_path}" ]]; then
            bash "${dep_script_path}"
            INSTALLED_DEPS[${dep_script_name}]=1 # Mark as installed
            ok "Dependency [${dep_script_name}] installed successfully."
        else
            error "Dependency script [${dep_script_path}] not found for [${dep_script_name}]! Skipping."
            return 1 # Indicate failure to install this specific dependency.
        fi
    else
        info "Dependency [${dep_script_name}] already installed. Skipping redundant call."
    fi
    return 0
}

# Displays usage information for the script or the function.
function usage() {
    info "Usage: $0 [OPTIONS]"
    info "Or call install_apollo_modules function directly: install_apollo_modules [OPTIONS]"
    info "Install Apollo module dependencies."
    info ""
    info "Options:"
    info "  --all                        Install dependencies for all known modules."
    info "  --modules <module1,module2,...> Install dependencies for specific modules (comma-separated)."
    info "                             Available modules: $(echo "${ALL_MODULES[@]}" | tr ' ' ', ')"
    info "  -h, --help                   Show this help message."
    info ""
    info "Example: $0 --modules perception,planning"
    info "Example: $0 --all"
    info "Example: install_apollo_modules --modules common"
}


# -----------------------------------------------------------------------------
# Main Logic encapsulated in a function
# -----------------------------------------------------------------------------

# Function to install Apollo module dependencies.
# This function accepts the same arguments as the original script.
# Arguments:
#   $@: Command-line arguments like --all, --modules, -h, --help
function install_apollo_modules() {
    # Local variables for this function call
    local install_all=false
    local selected_modules_raw="" # Store raw string for later processing
    local modules_to_process=()

    # Parse function arguments
    # Note: Using 'local' for these variables is crucial to avoid interfering
    # with global variables if the function is called multiple times.
    # We use 'local ARGS=("$@")' to copy the arguments, then iterate over ARGS.
    local ARGS=("$@")
    local i=0
    while (( i < ${#ARGS[@]} )); do
        local arg="${ARGS[i]}"
        case "${arg}" in
            --all)
                install_all=true
                i=$((i + 1))
                ;;
            --modules)
                local next_arg="${ARGS[i+1]}"
                if [[ -n "${next_arg}" && "${next_arg}" != -* ]]; then
                    selected_modules_raw="${next_arg}"
                    i=$((i + 2))
                else
                    error "Error: --modules requires an argument (comma-separated list of modules)."
                    usage
                    return 1 # Return status for function
                fi
                ;;
            -h|--help)
                usage
                return 0 # Return status for function
                ;;
            *)
                error "Error: Unknown argument '${arg}'"
                usage
                return 1 # Return status for function
                ;;
        esac
    done

    if ! "${install_all}" && [[ -z "${selected_modules_raw}" ]]; then
        info "No modules specified. Please use --all or --modules. Showing usage."
        usage
        return 0 # Return status for function
    fi

    # -----------------------------------------------------------------------------
    # Step 1: Install Global Dependencies
    # -----------------------------------------------------------------------------
    if [[ ${#GLOBAL_DEPS[@]} -gt 0 ]]; then
        info "Installing global dependencies..."
        for dep_script in "${GLOBAL_DEPS[@]}"; do
            install_dep "${dep_script}"
        done
        ok "Global dependencies installation complete."
    else
        info "No global dependencies defined in this script."
    fi

    # -----------------------------------------------------------------------------
    # Step 2: Determine which modules to process and validate selection
    # -----------------------------------------------------------------------------
    if "${install_all}"; then
        info "Installing dependencies for ALL modules."
        modules_to_process=("${ALL_MODULES[@]}")
    else
        # Convert comma-separated string to array
        IFS=',' read -r -a input_modules_array <<< "${selected_modules_raw}"
        info "Processing selected modules: ${input_modules_array[*]}"

        for module_name in "${input_modules_array[@]}"; do
            # Validate if the selected module is a known module
            local module_is_known=false
            for known_module in "${ALL_MODULES[@]}"; do
                if [[ "${known_module}" == "${module_name}" ]]; then
                    module_is_known=true
                    break
                fi
            done

            if "${module_is_known}"; then
                modules_to_process+=("${module_name}")
            else
                warning "Module '${module_name}' is not recognized as a valid Apollo module. Skipping."
            fi
        done
    fi

    if [[ ${#modules_to_process[@]} -eq 0 ]]; then
        warning "No valid modules to process after selection and validation. Exiting."
        return 0 # Return status for function
    fi

    # -----------------------------------------------------------------------------
    # Step 3: Install module-specific dependencies
    # -----------------------------------------------------------------------------
    info "Starting module-specific dependency installation for: ${modules_to_process[*]}..."
    for component_name in "${modules_to_process[@]}"; do
        info "--> Processing module: [${component_name}]"

        local deps_array_name="${MODULE_TO_DEPS_ARRAY_MAP[${component_name}]}"

        if [[ -n "${deps_array_name}" ]]; then
            local current_deps_elements_str
            eval "current_deps_elements_str=\"\${${deps_array_name}[@]}\""
            local -a current_deps_array=($current_deps_elements_str)

            if [[ ${#current_deps_array[@]} -gt 0 ]]; then
                info "    Dependencies for [${component_name}]: ${current_deps_array[*]}"
                for dep_script in "${current_deps_array[@]}"; do
                    install_dep "${dep_script}"
                done
            else
                ok "    Module [${component_name}] has no specific third-party dependencies defined in this script."
            fi
        else
            ok "    Module [${component_name}] has no specific third-party dependencies defined in this script's map."
        fi
    done

    # -----------------------------------------------------------------------------
    # Final cleanup (kept outside the function if it's a global script action,
    # or moved inside if the function fully encapsulates the process)
    # For now, it's global to match original script's flow.
    # -----------------------------------------------------------------------------
    # info "Cleaning up APT cache..."
    # apt-get clean && \
    #     rm -rf /var/lib/apt/lists/*
    # ok "APT cache cleaned successfully."

    ok "All specified Apollo module dependencies installation complete!"
    return 0
}

# -----------------------------------------------------------------------------
# Script Entry Point (calls the function with all original arguments)
# -----------------------------------------------------------------------------

# If this script is executed directly, call the function with its arguments.
# This makes the script behave exactly as before, but its core logic is now
# encapsulated.
install_apollo_modules "$@"

# Final cleanup that always runs after the main logic completes successfully or exits.
# If you want this cleanup to be part of the function's responsibility, move it inside.
info "Cleaning up APT cache..."
apt-get clean && \
    rm -rf /var/lib/apt/lists/*
ok "APT cache cleaned successfully."
