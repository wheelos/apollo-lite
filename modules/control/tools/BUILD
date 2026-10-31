load("@rules_cc//cc:defs.bzl", "cc_binary")
load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:mappings.bzl", "pkg_attributes", "pkg_filegroup", "pkg_files", "strip_prefix")

package(default_visibility = ["//visibility:public"])

pkg_files(
    name = "package_files_binaries",
    srcs = [
        ":control_tester",
        ":pad_terminal",
    ],
    attributes = pkg_attributes(mode = "0755"),
    include_runfiles = True,
    prefix = "control/bin",
    strip_prefix = strip_prefix.from_root("modules/control/tools"),
)

pkg_filegroup(
    name = "package_files",
    srcs = [":package_files_binaries"],
    visibility = ["//visibility:public"],
)

pkg_install(
    name = "install_dir",
    srcs = [":package_files"],
    tags = ["manual"],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "control_tester",
    srcs = ["control_tester.cc"],
    linkstatic = True,
    deps = [
        "@wheelos_common//modules/common/adapters:adapter_gflags",
        "@wheelos_common//modules/common/util",
        "//modules/control/common:control_gflags",
        "@com_github_gflags_gflags//:gflags",
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/control_msgs:pad_msg_cc_proto",
        "@wheelos_msgs//wheelos_msgs/dreamview_msgs:chart_cc_proto",
        "@wheelos_msgs//wheelos_msgs/localization_msgs:localization_cc_proto",
        "@wheelos_msgs//wheelos_msgs/localization_msgs:localization_status_cc_proto",
        "@wheelos_msgs//wheelos_msgs/localization_msgs:pose_cc_proto",
        "@wheelos_msgs//wheelos_msgs/map_msgs:map_cc_proto",
        "@wheelos_msgs//wheelos_msgs/map_msgs:map_clear_area_cc_proto",
        "@wheelos_msgs//wheelos_msgs/map_msgs:map_crosswalk_cc_proto",
        "@wheelos_msgs//wheelos_msgs/map_msgs:map_geometry_cc_proto",
        "@wheelos_msgs//wheelos_msgs/map_msgs:map_id_cc_proto",
        "@wheelos_msgs//wheelos_msgs/map_msgs:map_junction_cc_proto",
        "@wheelos_msgs//wheelos_msgs/map_msgs:map_lane_cc_proto",
        "@wheelos_msgs//wheelos_msgs/map_msgs:map_overlap_cc_proto",
        "@wheelos_msgs//wheelos_msgs/map_msgs:map_parking_space_cc_proto",
        "@wheelos_msgs//wheelos_msgs/map_msgs:map_pnc_junction_cc_proto",
        "@wheelos_msgs//wheelos_msgs/map_msgs:map_road_cc_proto",
        "@wheelos_msgs//wheelos_msgs/map_msgs:map_signal_cc_proto",
        "@wheelos_msgs//wheelos_msgs/map_msgs:map_speed_bump_cc_proto",
        "@wheelos_msgs//wheelos_msgs/map_msgs:map_stop_sign_cc_proto",
        "@wheelos_msgs//wheelos_msgs/map_msgs:map_yield_sign_cc_proto",
        "@wheelos_msgs//wheelos_msgs/perception_msgs:perception_obstacle_cc_proto",
        "@wheelos_msgs//wheelos_msgs/perception_msgs:traffic_light_detection_cc_proto",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:decision_cc_proto",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:navigation_cc_proto",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:planning_cc_proto",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:planning_internal_cc_proto",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:sl_boundary_cc_proto",
        "@wheelos_msgs//wheelos_msgs/routing_msgs:routing_cc_proto",
    ],
)

cc_binary(
    name = "pad_terminal",
    srcs = ["pad_terminal.cc"],
    linkstatic = True,
    deps = [
        "@wheelos_common//modules/common/adapters:adapter_gflags",
        "@wheelos_common//modules/common/util:util_tool",
        "//modules/control/common",
        "@com_github_gflags_gflags//:gflags",
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/control_msgs:pad_msg_cc_proto",
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
