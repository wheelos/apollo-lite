load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "junction_predictor",
    srcs = ["junction_predictor.cc"],
    hdrs = ["junction_predictor.h"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        "//modules/prediction/common:prediction_util",
        "//modules/prediction/predictor",
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
