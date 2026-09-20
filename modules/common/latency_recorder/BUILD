load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

install(
    name = "install",
    data_dest = "common",
    library_dest = "common/lib",
    runtime_dest = "common/bin",
    targets = [
        ":latency_recorder",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "latency_recorder",
    srcs = [
        "latency_recorder.cc",
    ],
    hdrs = ["latency_recorder.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/latency_recorder/proto:latency_record_cc_proto",
        "//modules/common/util:util_tool",
        "@core//cyber",
    ],
)

cpplint()
