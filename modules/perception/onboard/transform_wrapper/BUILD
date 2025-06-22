load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])


cc_library(
    name = "transform_wrapper",
    srcs = [
        "transform_wrapper.cc",
    ],
    hdrs = ["transform_wrapper.h"],
    deps = [
        "//modules/common/util:util_tool",
        "//modules/perception/common/sensor_manager",
        "//modules/transform:buffer",
        "@com_github_gflags_gflags//:gflags",
        "@eigen",
    ],
)

install(
    name = "install",
    library_dest = "perception/lib",
    targets = [
        ":transform_wrapper",
    ],
    visibility = ["//visibility:public"],
)

cpplint()
