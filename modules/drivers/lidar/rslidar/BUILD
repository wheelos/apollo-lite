load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

ROBOSENSE_COPTS = ['-DMODULE_NAME=\\"rslidar\\"']

install(
    name = "install",
    data = [
        ":runtime_data",
    ],
    data_dest = "drivers/addition_data/lidar/rslidar",
    library_dest = "drivers/lib/lidar/rslidar",
    targets = [
        "librslidar_component.so",
    ],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "conf/*.conf",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

cc_binary(
    name = "librslidar_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [
        ":rslidar_component_lib",
    ],
)

cc_library(
    name = "rslidar_component_lib",
    srcs = [
        "driver/rslidar_component.cpp",
    ],
    hdrs = [
        "driver/rslidar_component.h",
    ],
    copts = ROBOSENSE_COPTS,
    linkopts = [
        "-lpcap",
    ],
    deps = [
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/drivers/lidar/common:lidar_common",
        "//modules/drivers/lidar/rslidar/proto:rslidar_cc_proto",
        "//modules/drivers/lidar/rslidar/proto:rslidar_config_cc_proto",
        "@rs_driver",
    ],
    alwayslink = True,
)

cpplint()
