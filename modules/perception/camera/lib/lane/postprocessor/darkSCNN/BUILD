load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "darkSCNN_lane_postprocessor",
    srcs = ["darkSCNN_lane_postprocessor.cc"],
    hdrs = ["darkSCNN_lane_postprocessor.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        "//modules/common/util:util_tool",
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/camera/lib/lane/common:common_functions",
        "//modules/perception/lib/registerer",
        "//modules/perception/pipeline:stage",
        "//modules/perception/pipeline/proto/stage:darkSCNN_cc_proto",
        "//modules/perception/pipeline/proto/stage:darkSCNN_postprocessor_cc_proto",
        "@opencv//:imgcodecs",
    ],
    alwayslink = True,
)

cpplint()
