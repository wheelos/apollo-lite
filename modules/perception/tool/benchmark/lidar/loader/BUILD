load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "benchmark_loader",
    hdrs = [
        "async_sequence_data_loader.h",
        "sequence_data_loader.h",
        "string_compare.h",
    ],
    deps = [
        "//modules/perception/tool/benchmark/lidar/ctpl:benchmark_ctpl",
        "@boost.filesystem",
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
