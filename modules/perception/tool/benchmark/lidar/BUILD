load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "benchmark_evaluation",
    srcs = ["detection_evaluation.cc"],
    hdrs = ["detection_evaluation.h"],
    deps = [
        "//modules/perception/tool/benchmark/lidar/eval:benchmark_eval",
        "//modules/perception/tool/benchmark/lidar/loader:benchmark_loader",
    ],
)

cc_binary(
    name = "lidar_benchmark",
    srcs = ["lidar_detection_benchmark.cc"],
    linkopts = [
        "-llz4",
    ],
    deps = [
        "//modules/perception/tool/benchmark/lidar:benchmark_evaluation",
        "@boost.program_options",
    ],
)

cc_binary(
    name = "lidar_web_visualizer_exporter",
    srcs = ["lidar_web_visualizer_exporter.cc"],
    linkopts = [
        "-llz4",
    ],
    deps = [
        "@core//cyber/common:file",
        "//modules/perception/tool/benchmark/lidar/base:benchmark_base",
        "//modules/perception/tool/benchmark/lidar/eval:benchmark_eval",
        "//modules/perception/tool/benchmark/lidar/loader:benchmark_loader",
        "//modules/perception/tool/benchmark/lidar/util:benchmark_util",
        "@boost.program_options",
        "@com_github_nlohmann_json//:json",
    ],
)

cpplint()
