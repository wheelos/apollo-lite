load("@bazel_skylib//rules:common_settings.bzl", "bool_flag")
load("@core//runtime:product.bzl", "runtime_product")

package(default_visibility = ["//visibility:public"])

bool_flag(
    name = "roadlog",
    build_setting_default = False,
)

config_setting(
    name = "with_roadlog",
    flag_values = {":roadlog": "true"},
)

COMMON_PRODUCT_FILES = {
    "//products/low_speed/conf:global_flagfile": "conf/global_flagfile.txt",
    "//products/low_speed/conf:vehicle_config_data": "conf/vehicles/default/vehicle_param.pb.txt",
    "@wheelos_common//modules/common/vehicle_model:vehicle_model_config_data": "conf/vehicles/default/vehicle_model_config.pb.txt",
}

HARDWARE_MODULES = [
    "//tools/hardware:hardware_module",
]

ALGORITHM_MODULES_RTK = [
    "@wheelos_control//modules/control:module",
    "@wheelos_localization//modules/rtk_localization:module",
    "@wheelos_planning//modules/open_space_planning:module",
    "@wheelos_transform//modules/transform:module",
]

ALGORITHM_MODULES_NDT = [
    "@wheelos_control//modules/control:module",
    "@wheelos_localization//modules/ndt_localization:module",
    "@wheelos_planning//modules/planning:module",
    "@wheelos_transform//modules/transform:module",
]

OPTIONAL_TOOL_MODULES = select({
    ":with_roadlog": ["@wheelos_tool_kit//roadlog:module"],
    "//conditions:default": [],
})

runtime_product(
    name = "devkit_x86",
    files = COMMON_PRODUCT_FILES | {
        "//products/low_speed/launch:devkit_x86.launch": "launch/low_speed/devkit_x86.launch",
    },
    modules = HARDWARE_MODULES + ALGORITHM_MODULES_RTK + OPTIONAL_TOOL_MODULES,
    package_file_name = "devkit_x86.tar.gz",
)

runtime_product(
    name = "devkit_x86_ndt",
    files = COMMON_PRODUCT_FILES | {
        "//products/low_speed/launch:devkit_x86_ndt.launch": "launch/low_speed/devkit_x86_ndt.launch",
    },
    modules = HARDWARE_MODULES + ALGORITHM_MODULES_NDT + OPTIONAL_TOOL_MODULES,
    package_file_name = "devkit_x86_ndt.tar.gz",
)

alias(
    name = "runtime",
    actual = ":devkit_x86",
)
