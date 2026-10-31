load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "extrapolation_predictor",
    srcs = ["extrapolation_predictor.cc"],
    hdrs = ["extrapolation_predictor.h"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        "@wheelos_common//modules/common/math",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/common:prediction_map",
        "//modules/prediction/common:prediction_util",
        "//modules/prediction/container/obstacles:obstacle_clusters",
        "//modules/prediction/container/obstacles:obstacles_container",
        "//modules/prediction/predictor/sequence:sequence_predictor",
        "@wheelos_msgs//wheelos_msgs/prediction_msgs:lane_graph_cc_proto",
    ],
)

cc_test(
    name = "extrapolation_predictor_test",
    size = "small",
    srcs = ["extrapolation_predictor_test.cc"],
    deps = [
        ":extrapolation_predictor",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/common:gtest_exit_main",
        "@wheelos_msgs//wheelos_msgs/prediction_msgs:feature_cc_proto",
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
