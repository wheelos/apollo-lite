load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "open_space_planner",
    srcs = ["open_space_planner.cc"],
    hdrs = ["open_space_planner.h"],
    deps = [
        "//modules/open_space_planning/common:problem_validator",
        "//modules/open_space_planning/common:status",
        "//modules/open_space_planning/common:types",
        "//modules/open_space_planning/route:route_planner",
        "//modules/open_space_planning/safety:fallback_planner",
        "//modules/open_space_planning/safety:trajectory_validator",
        "//modules/open_space_planning/trajectory:trajectory_planner",
    ],
)

cc_test(
    name = "open_space_planner_test",
    size = "small",
    srcs = ["open_space_planner_test.cc"],
    deps = [
        ":open_space_planner",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
