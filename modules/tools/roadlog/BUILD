load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "smart_recorder",
    srcs = ["smart_recorder.cc"],
    linkstatic = True,
    deps = [
        ":smart_recorder_gflags",
        "//modules/tools/roadlog/processor:post_record_processor",
        "//modules/tools/roadlog/processor:realtime_record_processor",
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
