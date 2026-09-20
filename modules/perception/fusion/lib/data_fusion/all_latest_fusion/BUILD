load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "all_latest_fusion",
    srcs = ["all_latest_fusion.cc"],
    hdrs = ["all_latest_fusion.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        "//modules/common/util:string_util",
        "//modules/common/util:util_tool",
        "//modules/perception/fusion/base:sensor_data_manager",
        "//modules/perception/pipeline:stage",
    ],
)

cpplint()
