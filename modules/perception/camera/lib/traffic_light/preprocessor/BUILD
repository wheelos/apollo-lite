load("//tools:cpplint.bzl", "cpplint")
load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "multi_camera_projection",
    srcs = ["multi_camera_projection.cc"],
    hdrs = ["multi_camera_projection.h"],
    deps = [
        ":pose",
        "//cyber",
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/common/io:io_util",
        "//modules/perception/common/sensor_manager",
    ],
)

cc_library(
    name = "pose",
    srcs = ["pose.cc"],
    hdrs = ["pose.h"],
    deps = [
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/perception/base",
    ],
)

cc_library(
    name = "tl_preprocessor",
    srcs = ["tl_preprocessor.cc"],
    hdrs = ["tl_preprocessor.h"],
    deps = [
        ":multi_camera_projection",
        ":pose",
        "//cyber",
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/camera/lib/interface:base_init_options",
        "//modules/perception/camera/lib/interface:base_tl_preprocessor",
        "//modules/perception/lib/registerer",
        "//modules/perception/lib/config_manager",
        "@boost.circular_buffer",
        "@com_github_gflags_gflags//:gflags",
        "@eigen",
    ],
    alwayslink = True,
)

cpplint()
