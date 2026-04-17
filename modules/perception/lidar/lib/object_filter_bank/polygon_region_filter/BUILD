load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "polygon_region_filter",
    srcs = ["polygon_region_filter.cc"],
    hdrs = ["polygon_region_filter.h"],
    deps = [
        "//cyber",
        "//modules/perception/base:point_cloud",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lidar/lib/interface:base_object_filter",
        "//modules/perception/pipeline:plugin",
        "//modules/perception/pipeline/proto/plugin:polygon_region_filter_config_cc_proto",
        "@eigen",
    ],
    alwayslink = True,
)

cpplint()
