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
    srcs = ["ground_truth.cc"],
    hdrs = ["ground_truth.h"],
    deps = [
        "//modules/perception/camera/common",
        "@core//cyber",
    ],
)

cc_library(
    name = "util",
    srcs = ["util.cc"],
    hdrs = ["util.h"],
    deps = [
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/common/io:io_util",
        "@core//cyber",
        "@opencv//:imgcodecs",
        "@opencv//:imgproc",
    ],
)

cc_library(
    name = "visualizer",
    srcs = ["visualizer.cc"],
    hdrs = ["visualizer.h"],
    deps = [
        ":util",
        "//modules/perception/camera/common",
        "@opencv//:imgcodecs",
        "@opencv//:imgproc",
    ],
)

cpplint()
