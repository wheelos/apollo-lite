load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "driver",
    srcs = [
        "driver.cc",
        "input.cc",
    ],
    hdrs = [
        "driver.h",
        "input.h",
    ],
    copts = ['-DMODULE_NAME=\\"lslidar\\"'],
    linkopts = [
        "-lpcap",
    ],
    deps = [
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/drivers/lidar/lslidar/proto:config_cc_proto",
    ],
)

cpplint()
