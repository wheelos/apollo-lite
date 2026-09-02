load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "trajectory_validator",
    hdrs = ["trajectory_validator.h"],
    deps = ["//modules/open_space_planning/common:types"],
)

cc_library(
    name = "default_trajectory_validator",
    srcs = ["default_trajectory_validator.cc"],
    hdrs = ["default_trajectory_validator.h"],
    deps = [
        ":trajectory_validator",
        "//modules/open_space_planning/common:types",
    ],
)

cc_test(
    name = "default_trajectory_validator_test",
    size = "small",
    srcs = ["default_trajectory_validator_test.cc"],
    deps = [
        ":default_trajectory_validator",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "fallback_planner",
    hdrs = ["fallback_planner.h"],
    deps = [
        "//modules/open_space_planning/common:status",
        "//modules/open_space_planning/common:types",
    ],
)

cc_library(
    name = "default_fallback_planner",
    srcs = ["default_fallback_planner.cc"],
    hdrs = ["default_fallback_planner.h"],
    deps = [
        ":fallback_planner",
        "//modules/open_space_planning/common:status",
        "//modules/open_space_planning/common:types",
    ],
)

cc_test(
    name = "default_fallback_planner_test",
    size = "small",
    srcs = ["default_fallback_planner_test.cc"],
    deps = [
        ":default_fallback_planner",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()

