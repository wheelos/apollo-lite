load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "fusion_runtime",
    srcs = ["fusion_runtime.cc"],
    hdrs = ["fusion_runtime.h"],
    deps = [
        "//modules/perception/fusion/base:fusion_frame",
        "@core//cyber",
    ],
)

cc_test(
    name = "fusion_runtime_test",
    srcs = ["fusion_runtime_test.cc"],
    deps = [
        ":fusion_runtime",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "obstacle_multi_sensor_fusion",
    srcs = ["obstacle_multi_sensor_fusion.cc"],
    hdrs = ["obstacle_multi_sensor_fusion.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        "//modules/perception/base",
        "//modules/perception/fusion/base",
        "//modules/perception/fusion/lib/fusion_system/probabilistic_fusion",
        "//modules/perception/fusion/lib/interface:base_fusion_system",
        "//modules/perception/fusion/lib/interface:base_multisensor_fusion",
        "//modules/perception/lib/registerer",
        "//modules/perception/pipeline",
        "@core//cyber",
        "@eigen",
    ],
    alwayslink = True,
)

cpplint()
