load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:tar.bzl", "pkg_tar")
load("@rules_python//python:defs.bzl", "py_binary", "py_test")
load(":defs.bzl", "LIDARS", "VEHICLES", "hardware_manifest", "lidar_targets")
load(":runtime.bzl", "runtime_layout", "runtime_package")

package(default_visibility = ["//visibility:public"])

platform(
    name = "linux_x86_64",
    constraint_values = [
        "@platforms//cpu:x86_64",
        "@platforms//os:linux",
    ],
)

CAMERA_COMPONENTS = select({
    "@hardware.build//tools/hardware:camera_camera": ["@hardware.camera//:component"],
    "@hardware.build//tools/hardware:camera_camera_gst": ["@hardware.camera_gst//:component"],
    "//conditions:default": [],
})

CAN_COMPONENTS = select({
    "@hardware.build//tools/hardware:canbus_none": [],
    "//conditions:default": ["@hardware.canbus//:tool"],
})

GNSS_COMPONENTS = select({
    "@hardware.build//tools/hardware:gnss_none": [],
    "//conditions:default": ["@hardware.gnss//:libgnss_component.so"],
})

VEHICLE_COMPONENTS = select(dict(
    [(
        "@hardware.vehicle//brands:with_" + brand,
        ["@hardware.vehicle//:component"],
    ) for brand in VEHICLES] + [
        ("//conditions:default", []),
    ],
))

COMPONENTS = CAMERA_COMPONENTS + CAN_COMPONENTS + GNSS_COMPONENTS + VEHICLE_COMPONENTS

LIDAR_COMPONENTS = lidar_targets()

LIDAR_DATA = lidar_targets(runtime = True)

RUNTIME_DATA = LIDAR_DATA + select({
    "@hardware.build//tools/hardware:camera_camera": ["@hardware.camera//:runtime_data"],
    "@hardware.build//tools/hardware:camera_camera_gst": ["@hardware.camera_gst//:runtime_data"],
    "//conditions:default": [],
}) + select({
    "@hardware.build//tools/hardware:gnss_none": [],
    "//conditions:default": ["@hardware.gnss//:profile_runtime_data"],
}) + select(dict(
    [(
        "@hardware.vehicle//brands:with_" + brand,
        ["@hardware.vehicle//:profile_runtime_data"],
    ) for brand in VEHICLES] + [
        ("//conditions:default", []),
    ],
))

hardware_manifest(
    name = "manifest",
    components = COMPONENTS + LIDAR_COMPONENTS,
    profile = "@hardware.build//tools/hardware:profile",
    runtime_data = RUNTIME_DATA,
    settings = dict(
        [(
            "@hardware.build//tools/hardware:" + key,
            key,
        ) for key in [
            "camera",
            "canbus",
            "gnss",
            "vehicle",
        ]] +
        [(
            "@hardware.build//tools/hardware:lidar_" + brand,
            "lidar_" + brand,
        ) for brand in LIDARS],
    ),
)

filegroup(
    name = "hardware",
    srcs = [
        ":manifest",
        ":runtime",
    ],
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

py_binary(
    name = "materialize_runtime",
    srcs = ["materialize_runtime.py"],
)

py_test(
    name = "materialize_runtime_test",
    srcs = [
        "materialize_runtime.py",
        "materialize_runtime_test.py",
    ],
    main = "materialize_runtime_test.py",
)

runtime_layout(
    name = "runtime",
    components = COMPONENTS + LIDAR_COMPONENTS,
    generated_configs = [
        ":gnss_config",
        ":vehicle_config",
    ],
    metadata = [":manifest"],
    runtime_data = RUNTIME_DATA,
)

runtime_package(
    name = "package_files",
    runtime = ":runtime",
)

pkg_tar(
    name = "package",
    srcs = [
        ":package_files",
    ],
    extension = "tar.gz",
    package_file_name = "install.tar.gz",
)

pkg_install(
    name = "install_dir",
    srcs = [":package_files"],
)

test_suite(
    name = "tests",
    tests = [
        "@hardware.canbus//can_client:can_client_factory_test",
        "@hardware.gnss//parser:parser_factory_test",
        "@hardware.vehicle//brands:vehicle_selection_test",
    ],
)

filegroup(
    name = "exported_source_files",
    srcs = glob(
        ["**/*"],
        allow_empty = True,
    ),
    visibility = ["//visibility:public"],
)
