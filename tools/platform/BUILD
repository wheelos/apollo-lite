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

constraint_setting(
    name = "camera_gst_platform",
)

constraint_value(
    name = "jetson_orin",
    constraint_setting = ":camera_gst_platform",
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
