load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    runtime_dest = "tools/roadlog",
    targets = [
        ":smart_recorder",
    ],
)

cc_binary(
    name = "smart_recorder",
    srcs = ["smart_recorder.cc"],
    linkstatic = True,
    deps = [
        ":smart_recorder_gflags",
        "//modules/tools/roadlog/processor:roadlog_runtime",
    ],
)

cc_library(
    name = "smart_recorder_gflags",
    srcs = ["smart_recorder_gflags.cc"],
    hdrs = ["smart_recorder_gflags.h"],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cpplint()
