load("@rules_cc//cc:defs.bzl", "cc_library")
load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:mappings.bzl", "pkg_attributes", "pkg_filegroup", "pkg_files", "strip_prefix")
load("//tools/platform:build_defs.bzl", "copts_if_teleop", "if_teleop")

package(default_visibility = ["//visibility:public"])

DREAMVIEW_COPTS = ['-DMODULE_NAME=\\"dreamview\\"']

cc_library(
    name = "backend",
    srcs = ["dreamview.cc"],
    hdrs = ["dreamview.h"],
    copts = DREAMVIEW_COPTS + copts_if_teleop(),
    deps = [
        "@wheelos_common//modules/common/configs:vehicle_config_helper",
        "@wheelos_common//modules/common/status",
        "//modules/dreamview/backend/common:dreamview_gflags",
        "//modules/dreamview/backend/handlers:image_handler",
        "//modules/dreamview/backend/handlers:websocket_handler",
        "//modules/dreamview/backend/handlers:map_data_handler",
        "//modules/dreamview/backend/hmi",
        "//modules/dreamview/backend/map:map_service",
        "//modules/dreamview/backend/perception_camera_updater",
        "//modules/dreamview/backend/plugins:plugin_manager",
        "//modules/dreamview/backend/point_cloud:point_cloud_updater",
        "//modules/dreamview/backend/sim_control_manager",
        "//modules/dreamview/backend/sim_control_manager/core:sim_control_base",
        "//modules/dreamview/backend/sim_control_manager/core:sim_control_with_model_base",
        "//modules/dreamview/backend/sim_control_manager/common:interpolation_2d",
        "//modules/dreamview/backend/simulation_world:simulation_world_updater",
        "//modules/dreamview/proto:simulation_world_cc_proto",
        "//modules/dreamview/backend/sim_control_manager/proto:sim_control_internal_cc_proto",
        "//modules/dreamview/backend/sim_control_manager/proto:dynamic_model_conf_cc_proto",
        #"//modules/dreamview/backend/sim_control_manager/proto:fnn_model_cc_proto",
        "@wheelos_map//modules/map/hdmap:hdmap_util",
        "//modules/map/pnc_map",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:pad_msg_cc_proto",
        "@civetweb//:civetserver",
    ] + if_teleop(["//modules/dreamview/backend/teleop"]),
)

pkg_files(
    name = "package_files_data",
    srcs = ["//modules/dreamview/backend/testdata"],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "dreamview/addition_data/backend",
    strip_prefix = strip_prefix.from_root("modules/dreamview/backend"),
)

pkg_files(
    name = "package_files_archives",
    srcs = [":backend"],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "lib",
    strip_prefix = strip_prefix.from_root("modules/dreamview/backend"),
)

pkg_filegroup(
    name = "package_files",
    srcs = [
        ":package_files_archives",
        ":package_files_data",
    ],
)

pkg_install(
    name = "install_dir",
    srcs = [":package_files"],
    tags = ["manual"],
)

filegroup(
    name = "exported_source_files",
    srcs = glob(
        ["**/*"],
        allow_empty = True,
    ),
    visibility = ["//visibility:public"],
)

filegroup(
    name = "exported_source_headers",
    srcs = glob(
        ["**/*.h"],
        allow_empty = True,
    ),
    visibility = ["//visibility:public"],
)
