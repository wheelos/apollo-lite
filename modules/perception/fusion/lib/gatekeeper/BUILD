load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "collect_fused_object",
    srcs = ["collect_fused_object.cc"],
    hdrs = ["collect_fused_object.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        "//modules/common/util:string_util",
        "//modules/common/util:util_tool",
        "//modules/perception/fusion/base:sensor_data_manager",
        "//modules/perception/fusion/lib/gatekeeper/pbf_gatekeeper",
        "//modules/perception/fusion/lib/interface",
        "//modules/perception/pipeline:stage",
    ],
)

cpplint()
