load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

LIDAR_FUSION_COPTS = ['-DMODULE_NAME=\\"lidar_fusion\\"']

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
    name = "liblidar_fusion_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [
        ":lidar_fusion_component_lib",
    ],
)

cc_library(
    name = "lidar_fusion_component_lib",
    srcs = [
        "lidar_fusion_component.cc",
    ],
    hdrs = [
        "lidar_fusion_component.h",
    ],
    copts = LIDAR_FUSION_COPTS,
    deps = [
        "//cyber",
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/drivers/lidar/fusion/proto:lidar_fusion_config_cc_proto",
        "//modules/transform:buffer",
        "@eigen",
    ],
    alwayslink = True,
)

cpplint()
