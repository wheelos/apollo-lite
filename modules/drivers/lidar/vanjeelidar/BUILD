load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

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
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

cc_binary(
    name = "libvanjeelidar_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [
        ":vanjeelidar_component_lib",
    ],
)

cc_library(
    name = "measurement_conversion",
    srcs = ["src/measurement_conversion.cc"],
    hdrs = ["src/measurement_conversion.h"],
    deps = [
        "//modules/drivers/lidar/vanjeelidar/proto:vanjeelidar_cc_proto",
        "@vanjee-lidar-sdk//:vanjee_driver",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
    ],
)

cc_test(
    name = "measurement_conversion_test",
    size = "small",
    srcs = ["src/measurement_conversion_test.cc"],
    deps = [
        ":measurement_conversion",
        "@com_google_googletest//:gtest_main",
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
        ":measurement_conversion",
        "//modules/drivers/lidar/common:lidar_common",
        "//modules/drivers/lidar/vanjeelidar/proto:vanjeelidar_cc_proto",
        "//modules/drivers/lidar/vanjeelidar/proto:vanjeelidar_config_cc_proto",
        "@vanjee-lidar-sdk//:vanjee_driver",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
    ],
    alwayslink = True,
)

cpplint()
