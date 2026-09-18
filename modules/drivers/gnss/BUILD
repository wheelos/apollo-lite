load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

GNSS_COPTS = ['-DMODULE_NAME=\\"gnss\\"']

cc_binary(
    name = "libgnss_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":gnss_component_lib"],
)

cc_library(
    name = "gnss_component_lib",
    srcs = ["gnss_component.cc"],
    hdrs = ["gnss_component.h"],
    copts = GNSS_COPTS,
    deps = [
        "//modules/common/monitor_log",
        "//modules/drivers/gnss/proto:gnss_status_cc_proto",
        "//modules/drivers/gnss/stream:raw_stream",
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:drive_state_cc_proto",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:vehicle_signal_cc_proto",
        "@wheelos_msgs//wheelos_msgs/chassis_msgs:chassis_cc_proto",
        "@wheelos_msgs//wheelos_msgs/config_msgs:vehicle_config_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:ins_cc_proto",
    ],
    alwayslink = True,
)

install(
    name = "install",
    data = [
        ":runtime_data",
    ],
    data_dest = "drivers/addition_data/gnss",
    library_dest = "drivers/lib/gnss",
    targets = [
        ":libgnss_component.so",
    ],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

cpplint()
