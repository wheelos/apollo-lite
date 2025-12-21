load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

LIDAR_COMPENSATOR_COPTS = ['-DMODULE_NAME=\\"lidar_compensator\\"']

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
    name = "liblidar_compensator_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [
        ":lidar_compensator_component_lib",
    ],
)

cc_library(
    name = "lidar_compensator_component_lib",
    srcs = [
        "compensator.cc",
        "lidar_compensator_component.cc",
    ],
    hdrs = [
        "compensator.h",
        "lidar_compensator_component.h",
    ],
    copts = LIDAR_COMPENSATOR_COPTS,
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/latency_recorder",
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/drivers/lidar/compensator/proto:lidar_compensator_config_cc_proto",
        "//modules/transform:buffer",
        "@eigen",
    ],
    alwayslink = True,
)

cpplint()
