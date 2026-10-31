load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "benchmark_eval",
    srcs = [
        "frame_statistics.cc",
        "lidar_option.cc",
        "meta_statistics.cc",
        "position_metric.cc",
        "sequence_self_statistics.cc",
    ],
    hdrs = [
        "frame_statistics.h",
        "lidar_option.h",
        "meta_statistics.h",
        "position_metric.h",
        "sequence_self_statistics.h",
    ],
    deps = [
        "//modules/perception/tool/benchmark/lidar/base:benchmark_base",
        "//modules/perception/tool/benchmark/lidar/util:benchmark_util",
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
