load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

HESAI_COPTS = ['-DMODULE_NAME=\\"hesai\\"']

cc_binary(
    name = "libhesai_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":hesai_component_lib"],
)

cc_library(
    name = "hesai_component_lib",
    srcs = [
        "driver/hesai_component.cc",
    ],
    hdrs = [
        #        "driver/driver.h",
        "driver/hesai_component.h",
    ],
    copts = HESAI_COPTS + [
        "-fopenmp",
    ],
    linkopts = [
        "-lgomp",
        "-lssl",
        "-pthread",
        "-lm",
    ],
    deps = [
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/drivers/lidar/common:lidar_common",
        "//modules/drivers/lidar/hesai/proto:hesai_cc_proto",
        "//modules/drivers/lidar/hesai/proto:hesai_config_cc_proto",
        "@hesai-lidar-sdk//:hesai_sdk_lib",
    ],
    alwayslink = True,
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "conf/*.conf",
        "dag/*.dag",
        "launch/*.launch",
        "correction/*/*.dat",
        "correction/*/*.csv",
    ]),
)

cpplint()
