load("@bazel_skylib//lib:selects.bzl", "selects")
load("@bazel_skylib//rules:common_settings.bzl", "bool_flag", "string_flag")
load("@rules_pkg//pkg:mappings.bzl", "pkg_attributes", "pkg_files", "strip_prefix")
load("@rules_pkg//pkg:tar.bzl", "pkg_tar")
load(":defs.bzl", "LIDARS", "VEHICLES", "hardware_manifest", "lidar_targets")

package(default_visibility = ["//visibility:public"])

platform(
    name = "linux_x86_64",
    constraint_values = [
        "@platforms//cpu:x86_64",
        "@platforms//os:linux",
    ],
)

string_flag(
    name = "profile",
    build_setting_default = "custom",
)

string_flag(
    name = "camera",
    build_setting_default = "none",
    values = [
        "none",
        "camera",
        "camera_gst",
    ],
)

string_flag(
    name = "canbus",
    build_setting_default = "all",
    values = [
        "none",
        "all",
        "socket",
        "fake",
    ],
)

string_flag(
    name = "gnss",
    build_setting_default = "all",
    values = [
        "none",
        "all",
        "huace",
        "novatel",
        "forsense",
    ],
)

string_flag(
    name = "vehicle",
    build_setting_default = "legacy",
    values = [
        "legacy",
        "none",
    ] + VEHICLES,
)

[
    bool_flag(
        name = "lidar_" + brand,
        build_setting_default = False,
    )
    for brand in LIDARS
]

[
    config_setting(
        name = "with_lidar_" + brand,
        flag_values = {":lidar_" + brand: "true"},
    )
    for brand in LIDARS
]

[
    config_setting(
        name = dimension + "_" + value,
        flag_values = {":" + dimension: value},
    )
    for dimension, values in {
        "camera": [
            "none",
            "camera",
            "camera_gst",
        ],
        "canbus": [
            "none",
            "all",
            "socket",
            "fake",
        ],
        "gnss": [
            "none",
            "all",
            "huace",
            "novatel",
            "forsense",
        ],
    }.items()
    for value in values
]

[
    selects.config_setting_group(
        name = "with_gnss_" + brand,
        match_any = [
            ":gnss_all",
            ":gnss_" + brand,
        ],
    )
    for brand in [
        "huace",
        "novatel",
        "forsense",
    ]
]

[
    selects.config_setting_group(
        name = "with_canbus_" + backend,
        match_any = [
            ":canbus_all",
            ":canbus_" + backend,
        ],
    )
    for backend in [
        "socket",
        "fake",
    ]
]

CAMERA_COMPONENTS = select({
    ":camera_camera": ["//modules/drivers/camera:libcamera_component.so"],
    ":camera_camera_gst": ["//modules/drivers/camera_gst:libcamera_gst_component.so"],
    "//conditions:default": [],
})

CAN_COMPONENTS = select({
    ":canbus_none": [],
    "//conditions:default": ["//modules/drivers/canbus/can_client:can_client_tool"],
})

GNSS_COMPONENTS = select({
    ":gnss_none": [],
    "//conditions:default": ["//modules/drivers/gnss:libgnss_component.so"],
})

VEHICLE_COMPONENTS = select(dict(
    [(
        "//modules/canbus/vehicle:with_" + brand,
        ["//modules/canbus:libcanbus_component.so"],
    ) for brand in VEHICLES] + [
        ("//conditions:default", []),
    ],
))

COMPONENTS = CAMERA_COMPONENTS + CAN_COMPONENTS + GNSS_COMPONENTS + VEHICLE_COMPONENTS

LIDAR_COMPONENTS = lidar_targets()

LIDAR_DATA = lidar_targets(runtime = True)

RUNTIME_DATA = LIDAR_DATA + select({
    ":camera_camera": ["//modules/drivers/camera:runtime_data"],
    ":camera_camera_gst": ["//modules/drivers/camera_gst:runtime_data"],
    "//conditions:default": [],
}) + select({
    ":gnss_none": [],
    "//conditions:default": ["//modules/drivers/gnss:profile_runtime_data"],
}) + select(dict(
    [(
        "//modules/canbus/vehicle:with_" + brand,
        ["//modules/canbus:profile_runtime_data"],
    ) for brand in VEHICLES] + [
        ("//conditions:default", []),
    ],
))

hardware_manifest(
    name = "manifest",
    components = COMPONENTS + LIDAR_COMPONENTS,
    profile = ":profile",
    runtime_data = RUNTIME_DATA,
    settings = dict(
        [(
            ":" + key,
            key,
        ) for key in [
            "camera",
            "canbus",
            "gnss",
            "vehicle",
        ]] +
        [(
            ":lidar_" + brand,
            "lidar_" + brand,
        ) for brand in LIDARS],
    ),
)

filegroup(
    name = "hardware",
    srcs = [":manifest"] + COMPONENTS + LIDAR_COMPONENTS,
)

pkg_files(
    name = "component_files",
    srcs = COMPONENTS + LIDAR_COMPONENTS,
    attributes = pkg_attributes(mode = "0755"),
    prefix = "bazel-bin",
    strip_prefix = strip_prefix.from_root(""),
)

pkg_files(
    name = "runtime_files",
    srcs = RUNTIME_DATA,
    strip_prefix = strip_prefix.from_root(""),
)

pkg_files(
    name = "manifest_file",
    srcs = [":manifest"],
    prefix = "hardware",
    strip_prefix = strip_prefix.files_only(),
)

filegroup(
    name = "gnss_config",
    srcs = [":manifest"],
    output_group = "gnss_config",
)

filegroup(
    name = "vehicle_config",
    srcs = [":manifest"],
    output_group = "vehicle_config",
)

pkg_files(
    name = "gnss_config_file",
    srcs = [":gnss_config"],
    prefix = "modules/drivers/gnss/conf",
    strip_prefix = strip_prefix.files_only(),
)

pkg_files(
    name = "vehicle_config_file",
    srcs = [":vehicle_config"],
    prefix = "modules/canbus/conf",
    strip_prefix = strip_prefix.files_only(),
)

pkg_tar(
    name = "install",
    srcs = [
        ":component_files",
        ":gnss_config_file",
        ":manifest_file",
        ":runtime_files",
        ":vehicle_config_file",
    ],
    extension = "tar.gz",
)

test_suite(
    name = "tests",
    tests = [
        "//modules/canbus/vehicle:vehicle_selection_test",
        "//modules/drivers/canbus/can_client:can_client_factory_test",
        "//modules/drivers/gnss/parser:parser_factory_test",
    ],
)
