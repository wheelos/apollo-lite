load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

VANJEE_COPTS = ['-DMODULE_NAME=\\"vanjeelidar\\"']

install(
    name = "install",
    data = [
        ":runtime_data",
    ],
    data_dest = "drivers/addition_data/lidar/vanjeelidar",
    library_dest = "drivers/lib/lidar/vanjeelidar",
    targets = [
        "libvanjeelidar_component.so",
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
    name = "libvanjeelidar_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [
        ":vanjeelidar_component_lib",
    ],
)

cc_library(
    name = "vanjeelidar_component_lib",
    srcs = [
        "src/vanjeelidar_component.cpp",
    ],
    hdrs = [
        "src/vanjeelidar_component.h",
    ],
    copts = VANJEE_COPTS,
    deps = [
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/drivers/lidar/common:lidar_common",
        "//modules/drivers/lidar/vanjeelidar/proto:vanjeelidar_cc_proto",
        "//modules/drivers/lidar/vanjeelidar/proto:vanjeelidar_config_cc_proto",
        "@vanjee_driver",
    ],
    alwayslink = True,
)

cpplint()
