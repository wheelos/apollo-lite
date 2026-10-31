load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "benchmark_base",
    srcs = [
        "frame.cc",
        "point_cloud_frame.cc",
    ],
    hdrs = [
        "frame.h",
        "point_cloud_frame.h",
        "sequence_maintainer.h",
    ],
    deps = [
        "@wheelos_common//modules/common/util:util_tool",
        "//modules/perception/tool/benchmark/lidar/util:benchmark_util",
        "//modules/perception/tool/benchmark/lidar/util:object_util",
        "@core//cyber",
    ],
)

filegroup(
    name = "exported_source_files",
    srcs = glob(
        ["**/*"],
        allow_empty = True,
    ),
    visibility = ["//visibility:public"],
)

filegroup(
    name = "exported_source_headers",
    srcs = glob(
        ["**/*.h"],
        allow_empty = True,
    ),
    visibility = ["//visibility:public"],
)
