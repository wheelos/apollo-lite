load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "object_builder",
    srcs = ["object_builder.cc"],
    hdrs = ["object_builder.h"],
    deps = [
        "//modules/perception/base",
        "//modules/perception/common/geometry:common",
        "//modules/perception/common/geometry:convex_hull_2d",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lib/registerer",
        "//modules/perception/lidar/common:lidar_frame",
        "//modules/perception/pipeline:stage",
    ],
)

cc_test(
    name = "object_builder_test",
    size = "small",
    srcs = ["object_builder_test.cc"],
    deps = [
        ":object_builder",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
