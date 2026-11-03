load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:tar.bzl", "pkg_tar")
load(":defs.bzl", "LIDARS", "VEHICLES", "hardware_manifest", "lidar_runtime_resources", "lidar_targets")
load(":runtime.bzl", "runtime_layout", "runtime_module", "runtime_package", "runtime_resources", "runtime_run")

package(default_visibility = ["//visibility:public"])

runtime_run(
    name = "run",
    runtime = ":runtime",
)

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

CAN_EXECUTABLES = select({
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

COMPONENTS = CAMERA_COMPONENTS + GNSS_COMPONENTS + VEHICLE_COMPONENTS

LIDAR_COMPONENTS = lidar_targets()

[
    runtime_resources(
        name = "lidar_" + brand + "_runtime_resources",
        files = ["@hardware.lidar//" + brand + ":runtime_data"],
        module_name = "hardware",
        namespace = "hardware/lidar",
    )
    for brand in LIDARS
]

runtime_resources(
    name = "camera_runtime_resources",
    files = ["@hardware.camera//:runtime_data"],
    module_name = "hardware",
    namespace = "hardware/camera",
)

runtime_resources(
    name = "camera_gst_runtime_resources",
    files = ["@hardware.camera_gst//:runtime_data"],
    module_name = "hardware",
    namespace = "hardware/camera_gst",
)

runtime_resources(
    name = "gnss_runtime_resources",
    files = ["@hardware.gnss//:profile_runtime_data"],
    module_name = "hardware",
    namespace = "hardware/gnss",
)

runtime_resources(
    name = "vehicle_runtime_resources",
    files = ["@hardware.vehicle//:profile_runtime_data"],
    module_name = "hardware",
    namespace = "hardware/vehicle",
)

RUNTIME_RESOURCES = lidar_runtime_resources() + select({
    "@hardware.build//tools/hardware:camera_camera": [":camera_runtime_resources"],
    "@hardware.build//tools/hardware:camera_camera_gst": [":camera_gst_runtime_resources"],
    "//conditions:default": [],
}) + select({
    "@hardware.build//tools/hardware:gnss_none": [],
    "//conditions:default": [":gnss_runtime_resources"],
}) + select(dict(
    [(
        "@hardware.vehicle//brands:with_" + brand,
        [":vehicle_runtime_resources"],
    ) for brand in VEHICLES] + [
        ("//conditions:default", []),
    ],
))

hardware_manifest(
    name = "manifest",
    components = COMPONENTS + CAN_EXECUTABLES + LIDAR_COMPONENTS,
    profile = "@hardware.build//tools/hardware:profile",
    runtime_data = RUNTIME_RESOURCES,
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

alias(
    name = "materialize_runtime",
    actual = "@core//runtime:materialize_runtime",
)

test_suite(
    name = "materialize_runtime_test",
    tests = ["@core//runtime:materialize_runtime_test"],
)

runtime_module(
    name = "hardware_module",
    components = COMPONENTS + LIDAR_COMPONENTS,
    dependency_metadata = select({
        "@hardware.build//tools/hardware:camera_camera": [
            "@hardware.camera//:runtime_metadata",
        ],
        "//conditions:default": [],
    }),
    executables = CAN_EXECUTABLES,
    files = {
        ":gnss_config": "conf/hardware/gnss/",
        ":manifest": "share/hardware/",
        ":vehicle_config": "conf/hardware/vehicle/",
    },
    module_name = "hardware",
    resources = RUNTIME_RESOURCES,
)

runtime_layout(
    name = "runtime",
    files = {
        "//conf:global_flagfile": "conf/global_config/",
        "//conf:vehicle_config_data": "conf/vehicles/default/",
    },
    modules = [":hardware_module"],
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
