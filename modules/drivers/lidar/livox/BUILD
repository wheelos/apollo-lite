load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

LIVOX_COPTS = ['-DMODULE_NAME=\\"livox\\"']

cc_binary(
    name = "liblivox_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":livox_component_lib"],
)

cc_library(
    name = "livox_component_lib",
    srcs = [
        "component/livox_component.cpp",
    ],
    hdrs = [
        "component/livox_component.h",
    ],
    copts = LIVOX_COPTS,
    deps = [
        "//modules/drivers/lidar/common:lidar_common",
        "//modules/drivers/lidar/livox/proto:livox_cc_proto",
        "@livox-sdk",
    ],
    alwayslink = True,
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "conf/*.conf",
        "conf/*.json",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

cpplint()
