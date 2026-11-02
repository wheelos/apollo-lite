load("@bazel_skylib//rules:common_settings.bzl", "bool_flag", "string_flag")
load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:tar.bzl", "pkg_tar")
load("//tools/hardware:runtime.bzl", "required_runtime_input", "runtime_layout", "runtime_module", "runtime_package", "runtime_run")

package(default_visibility = ["//visibility:public"])

exports_files([
    "vehicle.dag",
    "vehicle.launch",
])

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
        "vehicle.dag": "dag/vehicle/vehicle.dag",
        "vehicle.launch": "launch/vehicle/vehicle.launch",
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
            "camera.dag": "dag/sensors/camera.dag",
            "camera.launch": "launch/sensors/camera.launch",
        },
        ":lidar": {
            ":lidar_config": "conf/sensors/lidar.pb.txt",
            "lidar.dag": "dag/sensors/lidar.dag",
            "lidar.launch": "launch/sensors/lidar.launch",
        },
        ":combined": {
            ":camera_config": "conf/sensors/camera.pb.txt",
            ":lidar_config": "conf/sensors/lidar.pb.txt",
            "camera.dag": "dag/sensors/camera.dag",
            "lidar.dag": "dag/sensors/lidar.dag",
            "combined.launch": "launch/sensors/combined.launch",
        },
        "//conditions:default": {},
    }),
    module_name = "sensors",
)

runtime_layout(
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
)

runtime_run(
    name = "run",
    runtime = ":runtime",
)

runtime_package(
    name = "package_files",
    runtime = ":runtime",
)

pkg_tar(
    name = "package",
    srcs = [":package_files"],
    extension = "tar.gz",
    package_file_name = "install.tar.gz",
)

pkg_install(
    name = "install_dir",
    srcs = [":package_files"],
)
