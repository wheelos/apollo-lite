load("@bazel_tools//tools/cpp:cc.bzl", "cc_library", "cc_binary")

cc_library(
    name = "record_reader_benchmark_lib",
    srcs = ["record_reader_benchmark.cc"],
    deps = [
        "//path/to/record:record_reader",  # Adjust this to your actual dependencies
        "//path/to/record:record_writer",
        "//path/to/base:base_lib",          # Include necessary libraries
    ],
)

cc_binary(
    name = "record_reader_benchmark",
    srcs = ["record_reader_benchmark.cc"],
    deps = [":record_reader_benchmark_lib"],
)

cc_binary(
    name = "record_writer_benchmark",
    srcs = ["record_writer_benchmark.cc"],
    deps = ["//path/to/record:record_writer", "//path/to/base:base_lib"],
)
