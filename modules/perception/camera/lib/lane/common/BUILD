load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "common_functions",
    srcs = ["common_functions.cc"],
    hdrs = ["common_functions.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        "//modules/common/util:util_tool",
        "//modules/perception/base",
        "@core//cyber",
        "@eigen",
    ],
)

cpplint()
