load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "adaptive_kalman_filter_test",
    size = "small",
    srcs = ["adaptive_kalman_filter_test.cc"],
    deps = [
        ":adaptive_kalman_filter",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "adaptive_kalman_filter",
    srcs = ["adaptive_kalman_filter.cc"],
    hdrs = ["adaptive_kalman_filter.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        "//modules/perception/base",
        "//modules/perception/radar/lib/interface:base_filter",
        "@eigen",
    ],
    alwayslink = True,
)

filegroup(
    name = "exported_source_files",
    srcs = glob(
        ["**/*"],
        allow_empty = True,
    ),
    visibility = ["//visibility:public"],
)

filegroup(
    name = "exported_source_headers",
    srcs = glob(
        ["**/*.h"],
        allow_empty = True,
    ),
    visibility = ["//visibility:public"],
)
