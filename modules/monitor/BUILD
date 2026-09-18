load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install", "install_src_files")

package(default_visibility = ["//visibility:public"])

MONITOR_COPTS = ['-DMODULE_NAME=\\"monitor\\"']

cc_binary(
    name = "libmonitor.so",
    linkshared = True,
    linkstatic = False,
    deps = [
        ":monitor_lib",
    ],
)

cc_library(
    name = "monitor_lib",
    srcs = ["monitor.cc"],
    hdrs = ["monitor.h"],
    copts = MONITOR_COPTS,
    visibility = ["//visibility:private"],
    deps = [
        "//modules/common/util:util_tool",
        "//modules/monitor/common:recurrent_runner",
        "//modules/monitor/hardware:gps_monitor",
        "//modules/monitor/hardware:resource_monitor",
        "//modules/monitor/hardware:socket_can_monitor",
        "//modules/monitor/software:camera_monitor",
        "//modules/monitor/software:channel_monitor",
        "//modules/monitor/software:functional_safety_monitor",
        "//modules/monitor/software:latency_monitor",
        "//modules/monitor/software:localization_monitor",
        "//modules/monitor/software:module_monitor",
        "//modules/monitor/software:process_monitor",
        "//modules/monitor/software:recorder_monitor",
        "//modules/monitor/software:summary_monitor",
        "@core//cyber",
    ],
    alwayslink = True,
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

install(
    name = "install",
    data = [
        ":monitor.BUILD",
        ":runtime_data",
    ],
    data_dest = "monitor",
    library_dest = "monitor/lib",
    targets = [
        ":libmonitor.so",
    ],
)

install_src_files(
    name = "install_src",
    deps = [
        ":install_monitor_hdrs",
        ":install_monitor_src",
    ],
)

install_src_files(
    name = "install_monitor_src",
    dest = "monitor/src",
    filter = "*",
    src_dir = ["."],
)

install_src_files(
    name = "install_monitor_hdrs",
    dest = "monitor/include",
    filter = "*.h",
    src_dir = ["."],
)

cpplint()
