load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    library_dest = "dreamview/lib",
    targets = [":dreamview_gflags"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "dreamview_gflags",
    srcs = [
        "dreamview_gflags.cc",
    ],
    hdrs = ["dreamview_gflags.h"],
    alwayslink = True,
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cpplint()
