load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

install(
    name = "install",
    data_dest = "common",
    library_dest = "common/lib",
    targets = [
        ":filters",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "filters",
    hdrs = [
        "digital_filter.h",
        "digital_filter_coefficients.h",
        "mean_filter.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":digital_filter",
        ":digital_filter_coefficients",
        ":mean_filter",
    ],
)

cc_library(
    name = "digital_filter",
    srcs = ["digital_filter.cc"],
    hdrs = ["digital_filter.h"],
    deps = [
        "@core//cyber",
    ],
    alwayslink = True,
)

cc_library(
    name = "mean_filter",
    srcs = ["mean_filter.cc"],
    hdrs = ["mean_filter.h"],
    deps = [
        "@core//cyber",
    ],
    alwayslink = True,
)

cc_library(
    name = "digital_filter_coefficients",
    srcs = ["digital_filter_coefficients.cc"],
    hdrs = ["digital_filter_coefficients.h"],
    deps = [
        "@core//cyber",
    ],
    alwayslink = True,
)

cc_test(
    name = "digital_filter_test",
    size = "small",
    srcs = ["digital_filter_test.cc"],
    linkstatic = True,
    deps = [
        ":digital_filter",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "mean_filter_test",
    size = "small",
    srcs = ["mean_filter_test.cc"],
    linkstatic = True,
    deps = [
        ":mean_filter",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "digital_filter_coefficients_test",
    size = "small",
    srcs = ["digital_filter_coefficients_test.cc"],
    deps = [
        ":digital_filter_coefficients",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
