load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "hough_transfer",
    srcs = ["hough_transfer.cc"],
    hdrs = ["hough_transfer.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
)

cc_test(
    name = "hough_transfer_test",
    size = "small",
    srcs = ["hough_transfer_test.cc"],
    deps = [
        ":hough_transfer",
        "@com_google_googletest//:gtest_main",
    ],
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
