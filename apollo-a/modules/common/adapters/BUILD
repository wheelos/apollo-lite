load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

cc_library(
    name = "adapter_gflags",
    srcs = [
        "adapter_gflags.cc",
    ],
    hdrs = ["adapter_gflags.h"],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
    visibility = ["//visibility:public"],
)

cpplint()
