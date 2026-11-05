#!/usr/bin/env bash

# Capability ownership is independent of component selection.
CAPABILITY_PACKAGES=(
  lidar_perception camera_perception multimodal_perception
  world_model localization prediction planning control mission routing
  safety hmi simulation perception
)

module_package() {
  case "$1" in
    lidar_object_detection|lidar_semantic_segmentation|lidar_fusion) echo lidar_perception ;;
    camera_semantic_segmentation) echo camera_perception ;;
    lane|traffic_light|storytelling) echo world_model ;;
    bevfusion) echo multimodal_perception ;;
    planning|local_planning|open_space_planning|map) echo planning ;;
    localization|ndt_localization|rtk_localization) echo localization ;;
    guardian|collision_guardian|monitor) echo safety ;;
    dreamview) echo hmi ;;
    world_model|prediction|control|mission|routing|simulation|perception|transform) echo "$1" ;;
    *) return 1 ;;
  esac
}

capability_package() {
  local package
  for package in "${CAPABILITY_PACKAGES[@]}"; do
    if [[ "$1" == "${package}" ]]; then
      echo "${package}"
      return
    fi
  done
  return 1
}

module_recursive_target() {
  local package
  if [[ "$1" == "openloc" ]]; then
    echo "@@openloc+//..."
  elif package="$(module_package "$1")"; then
    if [[ "${package}" == "$1" ]]; then
      echo "@wheelos_${package}//..."
    else
      echo "@wheelos_${package}//modules/$1/..."
    fi
  elif package="$(capability_package "$1")"; then
    echo "@wheelos_${package}//..."
  else
    return 1
  fi
}

capability_targets() {
  local package
  for package in "${CAPABILITY_PACKAGES[@]}"; do
    printf ' union @wheelos_%s//...' "${package}"
  done
}
