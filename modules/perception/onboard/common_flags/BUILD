load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "common_flags",
    srcs = ["common_flags.cc"],
    hdrs = ["common_flags.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

install(
    name = "install",
    library_dest = "perception/lib",
    targets = [
        ":common_flags",
    ],
    visibility = ["//visibility:public"],
)

cpplint()