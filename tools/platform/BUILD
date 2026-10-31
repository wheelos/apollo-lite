package(
    default_visibility = ["//visibility:public"],
)

config_setting(
    name = "use_gpu",
    define_values = {
        "USE_GPU": "true",
    },
)

config_setting(
    name = "with_teleop",
    define_values = {
        "WITH_TELEOP": "true",
    },
)

alias(
    name = "camera_gst_platform",
    actual = "@hardware.build//tools/platform:camera_gst_platform",
)

alias(
    name = "jetson_orin",
    actual = "@hardware.build//tools/platform:jetson_orin",
)

platform(
    name = "jetson_orin_platform",
    constraint_values = [
        ":jetson_orin",
        "@platforms//cpu:aarch64",
        "@platforms//os:linux",
    ],
)

config_setting(
    name = "is_jetson_orin",
    constraint_values = [":jetson_orin"],
)

filegroup(
    name = "exported_source_files",
    srcs = glob(
        ["**/*"],
        allow_empty = True,
    ),
    visibility = ["//visibility:public"],
)
