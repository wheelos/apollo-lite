load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "route_planner",
    hdrs = ["route_planner.h"],
    deps = [
        "//modules/open_space_planning/common:status",
        "//modules/open_space_planning/common:types",
    ],
)

cc_library(
    name = "skeleton_corridor_route_planner",
    srcs = ["skeleton_corridor_route_planner.cc"],
    hdrs = ["skeleton_corridor_route_planner.h"],
    deps = [
        ":route_planner",
        "//modules/open_space_planning/common:status",
        "//modules/open_space_planning/common:types",
    ],
)

cc_test(
    name = "skeleton_corridor_route_planner_test",
    size = "small",
    srcs = ["skeleton_corridor_route_planner_test.cc"],
    deps = [
        ":skeleton_corridor_route_planner",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "hybrid_a_star_route_planner",
    srcs = ["hybrid_a_star_route_planner.cc"],
    hdrs = ["hybrid_a_star_route_planner.h"],
    deps = [
        ":route_planner",
        ":skeleton_corridor_route_planner",
        "//modules/common/math:vec2d",
        "//modules/open_space_planning/common:status",
        "//modules/open_space_planning/common:types",
        "//modules/planning/open_space/coarse_trajectory_generator:hybrid_a_star",
        "//modules/planning/proto:planner_open_space_config_cc_proto",
    ],
)

cc_test(
    name = "hybrid_a_star_route_planner_test",
    size = "small",
    srcs = ["hybrid_a_star_route_planner_test.cc"],
    deps = [
        ":hybrid_a_star_route_planner",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()

