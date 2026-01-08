load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

SEYOND_COPTS = ['-DMODULE_NAME=\\"seyond\\"']

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
    name = "libseyond_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":seyond_component_lib"],
)

cc_library(
    name = "seyond_component_lib",
    srcs = [
        "src/seyond_component.cpp",
        "src/seyond_driver.cpp",
    ],
    hdrs = [
        "src/seyond_component.h",
        "src/seyond_driver.h",
    ],
    copts = SEYOND_COPTS + ["-std=c++17"],
    # linkopts = [
    #     "-linnoclientsdk",
    # ],
    deps = [
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/drivers/lidar/common:lidar_common",
        "//modules/drivers/lidar/seyond/proto:seyond_cc_proto",
        "//modules/drivers/lidar/seyond/proto:seyond_config_cc_proto",
        "@inno-lidar-sdk",
    ],
    alwayslink = True,
)

cpplint()
