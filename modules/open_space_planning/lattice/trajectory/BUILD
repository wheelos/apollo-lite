load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "curve1d",
    hdrs = ["curve1d.h"],
)

cc_library(
    name = "polynomial_curve1d",
    hdrs = ["polynomial_curve1d.h"],
    deps = [":curve1d"],
)

cc_library(
    name = "cubic_polynomial_curve1d",
    srcs = ["cubic_polynomial_curve1d.cc"],
    hdrs = ["cubic_polynomial_curve1d.h"],
    deps = [":polynomial_curve1d"],
)

cc_library(
    name = "quartic_polynomial_curve1d",
    srcs = ["quartic_polynomial_curve1d.cc"],
    hdrs = ["quartic_polynomial_curve1d.h"],
    deps = [":polynomial_curve1d"],
)

cc_library(
    name = "quintic_polynomial_curve1d",
    srcs = ["quintic_polynomial_curve1d.cc"],
    hdrs = ["quintic_polynomial_curve1d.h"],
    deps = [":polynomial_curve1d"],
)

cc_library(
    name = "polynomial_curves",
    deps = [
        ":cubic_polynomial_curve1d",
        ":quartic_polynomial_curve1d",
        ":quintic_polynomial_curve1d",
    ],
)

cc_test(
    name = "polynomial_curve1d_test",
    size = "small",
    srcs = ["polynomial_curve1d_test.cc"],
    deps = [
        ":polynomial_curves",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
