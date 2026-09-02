load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "trajectory_planner",
    hdrs = ["trajectory_planner.h"],
    deps = [
        "//modules/open_space_planning/common:status",
        "//modules/open_space_planning/common:types",
    ],
)

cc_library(
    name = "fem_pos_deviation_osqp",
    srcs = ["path_optimizer/fem_pos_deviation_osqp.cc"],
    hdrs = ["path_optimizer/fem_pos_deviation_osqp.h"],
    deps = [
        "//modules/open_space_planning/common:types",
        "@osqp",
    ],
)

cc_library(
    name = "piecewise_jerk_speed_osqp",
    srcs = ["speed_optimizer/piecewise_jerk_speed_osqp.cc"],
    hdrs = ["speed_optimizer/piecewise_jerk_speed_osqp.h"],
    deps = [
        "//modules/open_space_planning/common:types",
        "@osqp",
    ],
)

cc_library(
    name = "open_space_trajectory_planner",
    srcs = ["open_space_trajectory_planner.cc"],
    hdrs = ["open_space_trajectory_planner.h"],
    deps = [
        ":fem_pos_deviation_osqp",
        ":piecewise_jerk_speed_osqp",
        ":trajectory_planner",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math:vec2d",
        "//modules/open_space_planning/common:status",
        "//modules/open_space_planning/common:types",
        "//modules/open_space_planning/route:hybrid_a_star_route_planner",
        "//modules/planning/open_space/trajectory_smoother:distance_approach_problem",
        "//modules/planning/open_space/trajectory_smoother:dual_variable_warm_start_problem",
        "//modules/planning/open_space/trajectory_smoother:iterative_anchoring_smoother",
        "//modules/planning/proto:planner_open_space_config_cc_proto",
        "@eigen",
    ],
)

cc_test(
    name = "fem_pos_deviation_osqp_test",
    size = "small",
    srcs = ["path_optimizer/fem_pos_deviation_osqp_test.cc"],
    deps = [
        ":fem_pos_deviation_osqp",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "piecewise_jerk_speed_osqp_test",
    size = "small",
    srcs = ["speed_optimizer/piecewise_jerk_speed_osqp_test.cc"],
    deps = [
        ":piecewise_jerk_speed_osqp",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "open_space_trajectory_planner_test",
    size = "small",
    srcs = ["open_space_trajectory_planner_test.cc"],
    deps = [
        ":open_space_trajectory_planner",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()



