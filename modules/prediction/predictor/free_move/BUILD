load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "free_move_predictor",
    srcs = ["free_move_predictor.cc"],
    hdrs = ["free_move_predictor.h"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        "@wheelos_common//modules/common/math",
        "//modules/prediction/common:prediction_util",
        "//modules/prediction/container/obstacles:obstacles_container",
        "//modules/prediction/predictor",
    ],
)

cc_test(
    name = "free_move_predictor_test",
    size = "small",
    srcs = ["free_move_predictor_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        "@wheelos_common//modules/common/math",
        "//modules/prediction/common:gtest_exit_main",
        "//modules/prediction/common:kml_map_based_test",
        "//modules/prediction/predictor/free_move:free_move_predictor",
    ],
)

cc_test(
    name = "free_move_predictor_eval_test",
    size = "small",
    srcs = ["free_move_predictor_eval_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        "@wheelos_common//modules/common/math",
        "//modules/prediction/common:gtest_exit_main",
        "//modules/prediction/common:kml_map_based_test",
        "//modules/prediction/predictor/free_move:free_move_predictor",
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
