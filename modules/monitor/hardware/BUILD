load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "copts_if_esd_can", "if_esd_can")

package(default_visibility = ["//visibility:public"])

MONITOR_COPTS = ['-DMODULE_NAME=\\"monitor\\"']

cc_library(
    name = "collision_monitor",
    srcs = ["collision_monitor.cc"],
    hdrs = ["collision_monitor.h"],
    copts = MONITOR_COPTS,
    deps = [
        "//modules/monitor/common:monitor_manager",
        "//modules/monitor/common:recurrent_runner",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:map_util",
        "//modules/common_msgs/perception_msgs:collision_warning_cc_proto",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "gps_monitor",
    srcs = ["gps_monitor.cc"],
    hdrs = ["gps_monitor.h"],
    copts = MONITOR_COPTS,
    deps = [
        "//cyber",
        "//modules/common_msgs/sensor_msgs:gnss_best_pose_cc_proto",
        "//modules/monitor/common:monitor_manager",
        "//modules/monitor/common:recurrent_runner",
        "//modules/monitor/software:summary_monitor",
    ],
)

cc_library(
    name = "resource_monitor",
    srcs = ["resource_monitor.cc"],
    hdrs = ["resource_monitor.h"],
    copts = MONITOR_COPTS,
    deps = [
        "//modules/common/util",
        "//modules/dreamview/proto:hmi_mode_cc_proto",
        "//modules/monitor/common:monitor_manager",
        "//modules/monitor/common:recurrent_runner",
        "//modules/monitor/software:summary_monitor",
        "@boost.filesystem",
    ],
)

cc_library(
    name = "esdcan_monitor",
    srcs = ["esdcan_monitor.cc"],
    hdrs = ["esdcan_monitor.h"],
    copts = MONITOR_COPTS + copts_if_esd_can(),
    deps = [
        "//modules/common/util",
        "//modules/monitor/common:monitor_manager",
        "//modules/monitor/common:recurrent_runner",
        "//modules/monitor/software:summary_monitor",
        ] + if_esd_can([
        "//modules/drivers/canbus/can_client/esd:esd_can_client",
    ]),
)

cc_library(
    name = "socket_can_monitor",
    srcs = ["socket_can_monitor.cc"],
    hdrs = ["socket_can_monitor.h"],
    copts = MONITOR_COPTS,
    deps = [
        "//modules/common/util",
        "//modules/monitor/common:monitor_manager",
        "//modules/monitor/common:recurrent_runner",
        "//modules/monitor/software:summary_monitor",
    ],
)

cpplint()
