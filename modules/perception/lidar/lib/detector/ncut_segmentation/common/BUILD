load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "flood_fill",
    srcs = ["flood_fill.cc"],
    hdrs = ["flood_fill.h"],
    copts = [
        '-DMODULE_NAME=\"perception\"',
    ],
    deps = [
        "//cyber",
        "//modules/perception/base",
    ],
)

cc_library(
    name = "lr_classifier",
    hdrs = ["lr_classifier.h"],
    deps = [
        "//cyber",
        "//modules/perception/base",
    ],
)

cpplint()
