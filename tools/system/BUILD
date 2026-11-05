load("@bazel_skylib//rules:common_settings.bzl", "bool_flag", "string_flag")
load("//tools/hardware:runtime.bzl", "required_runtime_input", "runtime_module")
load("//tools/products:defs.bzl", "runtime_product")

package(default_visibility = ["//visibility:public"])

bool_flag(
    name = "vehicle",
    build_setting_default = False,
)

string_flag(
    name = "sensors",
    build_setting_default = "none",
    values = [
        "none",
        "camera",
        "lidar",
        "combined",
    ],
)

[
    config_setting(
        name = choice,
        flag_values = {":sensors": choice},
    )
    for choice in [
        "camera",
        "lidar",
        "combined",
    ]
]

label_flag(
    name = "vehicle_config",
    build_setting_default = ":configuration_required",
)

label_flag(
    name = "vehicle_parameters",
    build_setting_default = ":configuration_required",
)

label_flag(
    name = "vehicle_flags",
    build_setting_default = ":configuration_required",
)

label_flag(
    name = "cyber_configuration",
    build_setting_default = "@core//cyber:cyber_conf",
)

label_flag(
    name = "camera_config",
    build_setting_default = ":configuration_required",
)

label_flag(
    name = "lidar_config",
    build_setting_default = ":configuration_required",
)

required_runtime_input(
    name = "configuration_required",
    message = "Supply reviewed //tools/system configuration label flags before assembling modules.",
)

config_setting(
    name = "with_vehicle",
    flag_values = {":vehicle": "true"},
)

runtime_module(
    name = "vehicle_module",
    components = ["@hardware.vehicle//:component"],
    files = {
        ":vehicle_config": "conf/vehicle/canbus_conf.pb.txt",
        ":vehicle_parameters": "conf/vehicle/vehicle_param.pb.txt",
        ":vehicle_flags": "conf/vehicle/vehicle.conf",
        "//products/apollo_lite/launch:vehicle.dag": "dag/vehicle/vehicle.dag",
        "//products/apollo_lite/launch:vehicle.launch": "launch/vehicle/vehicle.launch",
    },
    module_name = "vehicle",
)

runtime_module(
    name = "sensors_module",
    components = select({
        ":camera": ["@hardware.camera//:component"],
        ":lidar": ["@hardware.lidar//:hesai"],
        ":combined": [
            "@hardware.camera//:component",
            "@hardware.lidar//:hesai",
        ],
        "//conditions:default": [],
    }),
    files = select({
        ":camera": {
            ":camera_config": "conf/sensors/camera.pb.txt",
            "//products/apollo_lite/launch:camera.dag": "dag/sensors/camera.dag",
            "//products/apollo_lite/launch:camera.launch": "launch/sensors/camera.launch",
        },
        ":lidar": {
            ":lidar_config": "conf/sensors/lidar.pb.txt",
            "//products/apollo_lite/launch:lidar.dag": "dag/sensors/lidar.dag",
            "//products/apollo_lite/launch:lidar.launch": "launch/sensors/lidar.launch",
        },
        ":combined": {
            ":camera_config": "conf/sensors/camera.pb.txt",
            ":lidar_config": "conf/sensors/lidar.pb.txt",
            "//products/apollo_lite/launch:camera.dag": "dag/sensors/camera.dag",
            "//products/apollo_lite/launch:lidar.dag": "dag/sensors/lidar.dag",
            "//products/apollo_lite/launch:combined.launch": "launch/apollo_lite/combined.launch",
        },
        "//conditions:default": {},
    }),
    module_name = "sensors",
)

runtime_product(
    name = "runtime",
    cyber_conf = ":cyber_configuration",
    modules = select({
        ":with_vehicle": [":vehicle_module"],
        "//conditions:default": [],
    }) + select({
        ":camera": [":sensors_module"],
        ":lidar": [":sensors_module"],
        ":combined": [":sensors_module"],
        "//conditions:default": [],
    }),
    package_file_name = "install.tar.gz",
)

alias(
    name = "run",
    actual = ":runtime_run",
)

alias(
    name = "package_files",
    actual = ":runtime_files",
)

alias(
    name = "package",
    actual = ":runtime_package",
)

alias(
    name = "install_dir",
    actual = ":runtime_install",
)
