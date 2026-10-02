load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lane_relative",
    srcs = ["lane_relative.cc"],
    hdrs = ["lane_relative.h"],
    deps = [":types", "@eigen"],
)

cc_library(
    name = "types",
    srcs = ["types.cc"],
    hdrs = ["types.h"],
    deps = ["@eigen"],
)

cc_library(
    name = "observed_subspace",
    srcs = ["observed_subspace.cc"],
    hdrs = ["observed_subspace.h"],
    deps = [":types", "@eigen"],
)

cc_library(
    name = "local_motion",
    srcs = ["local_motion.cc"],
    hdrs = ["local_motion.h"],
    deps = [":types", "@eigen"],
)

cc_library(
    name = "continuous_output",
    srcs = ["continuous_output.cc"],
    hdrs = ["continuous_output.h"],
    deps = [":types", "@eigen"],
)

cc_library(
    name = "local_lio",
    srcs = ["local_lio.cc"],
    hdrs = ["local_lio.h"],
    deps = [":observed_subspace", ":types", "@eigen"],
)

cc_library(
    name = "local_estimator",
    srcs = ["local_estimator.cc"],
    hdrs = ["local_estimator.h"],
    deps = [
        ":continuous_output",
        ":local_lio",
        ":local_motion",
        ":types",
        "@eigen",
    ],
)

cc_library(
    name = "global_types",
    srcs = ["global_types.cc"],
    hdrs = ["global_types.h"],
    deps = [":types", "@eigen"],
)

cc_library(
    name = "global_alignment",
    srcs = ["global_alignment.cc", "fixed_lag_graph.cc"],
    hdrs = ["global_alignment.h", "fixed_lag_graph.h"],
    deps = [
        ":global_types",
        ":local_motion",
        ":types",
        "@eigen",
        "@gtsam//:gtsam",
    ],
)

cc_test(
    name = "lane_relative_test",
    size = "small",
    srcs = ["lane_relative_test.cc"],
    deps = [":lane_relative", "@com_google_googletest//:gtest_main"],
)

cc_test(
    name = "observed_subspace_test",
    size = "small",
    srcs = ["observed_subspace_test.cc"],
    deps = [":observed_subspace", "@com_google_googletest//:gtest_main"],
)

cc_test(
    name = "local_motion_test",
    size = "small",
    srcs = ["local_motion_test.cc"],
    deps = [":local_motion", "@com_google_googletest//:gtest_main"],
)

cc_test(
    name = "continuous_output_test",
    size = "small",
    srcs = ["continuous_output_test.cc"],
    deps = [":continuous_output", "@com_google_googletest//:gtest_main"],
)

cc_test(
    name = "local_lio_test",
    size = "small",
    srcs = ["local_lio_test.cc"],
    deps = [":local_lio", "@com_google_googletest//:gtest_main"],
)

cc_test(
    name = "local_estimator_test",
    size = "small",
    srcs = ["local_estimator_test.cc"],
    deps = [":local_estimator", "@com_google_googletest//:gtest_main"],
)

cc_test(
    name = "fixed_lag_graph_test",
    size = "small",
    srcs = ["fixed_lag_graph_test.cc"],
    deps = [":global_alignment", "@com_google_googletest//:gtest_main"],
)

cc_test(
    name = "global_alignment_test",
    size = "small",
    srcs = ["global_alignment_test.cc"],
    deps = [":global_alignment", "@com_google_googletest//:gtest_main"],
)

cpplint()
