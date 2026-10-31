load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "move_sequence_predictor",
    srcs = ["move_sequence_predictor.cc"],
    hdrs = ["move_sequence_predictor.h"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        "//modules/prediction/common:prediction_util",
        "//modules/prediction/predictor/sequence:sequence_predictor",
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
