load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "interval_pool",
    srcs = ["interval_pool.cc"],
    hdrs = ["interval_pool.h"],
    deps = [
        "//cyber",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "channel_pool",
    srcs = ["channel_pool.cc"],
    hdrs = ["channel_pool.h"],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
    ],
)

cpplint()
