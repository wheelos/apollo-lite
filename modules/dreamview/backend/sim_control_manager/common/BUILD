load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    library_dest = "dreamview/lib",
    targets = [":sim_control_gflags"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "sim_control_gflags",
    srcs = [
        "sim_control_gflags.cc",
    ],
    hdrs = ["sim_control_gflags.h"],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "interpolation_2d",
    srcs = [
        "interpolation_2d.cc",
    ],
    hdrs = [
        "interpolation_2d.h",
    ],
    alwayslink = True,
)

cc_library(
    name = "sim_control_util",
    srcs = [
        "sim_control_util.cc",
    ],
    hdrs = [
        "sim_control_util.h",
    ],
    alwayslink = True,
    deps = [
        "//cyber",
    ],
)

cpplint()