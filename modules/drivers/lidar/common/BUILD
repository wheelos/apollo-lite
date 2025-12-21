load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lidar_common",
    srcs = [
        "util.cpp",
    ],
    hdrs = [
        "lidar_component_base.h",
        "lidar_component_base_impl.h",
        "sync_buffering.h",
        "sync_queue.h",
        "util.h",
    ],
    copts = [],
    deps = [
        "//cyber",
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/drivers/lidar/common/proto:lidar_config_base_cc_proto",
        "@boost.format",
    ],
)

cpplint()
