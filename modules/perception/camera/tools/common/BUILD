load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "common",
    deps = [
        ":ground_truth",
        ":util",
        ":visualizer",
    ],
)

cc_library(
    name = "ground_truth",
    hdrs = ["ground_truth.h"],
    srcs = ["ground_truth.cc"],
    deps = [
        "@core//cyber",
        "//modules/perception/camera/common",
    ],
)

cc_library(
    name = "util",
    hdrs = ["util.h"],
    srcs = ["util.cc"],
    deps = [
        "@core//cyber",
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/common/io:io_util",
        "@opencv//:imgcodecs",
        "@opencv//:imgproc",
    ],
)

cc_library(
    name = "visualizer",
    hdrs = ["visualizer.h"],
    srcs = ["visualizer.cc"],
    deps = [
        ":util",
        "//modules/perception/camera/common",
        "@opencv//:imgcodecs",
        "@opencv//:imgproc",
    ],
)

cpplint()
