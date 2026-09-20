load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "histogram_estimator",
    srcs = ["histogram_estimator.cc"],
    hdrs = ["histogram_estimator.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        "//modules/perception/common/i_lib/core",
        "@core//cyber",
        "@eigen",
    ],
)

cpplint()
