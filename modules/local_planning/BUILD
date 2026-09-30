load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "scenario",
    srcs = ["scenario.cc"],
    hdrs = ["scenario.h"],
    deps = [
        "//modules/local_planning/planning_context:input_gate",
        "//modules/common/status",
    ],
)

cc_test(
    name = "local_planning_core_test",
    size = "small",
    srcs = ["local_planning_core_test.cc"],
    deps = [
        ":scenario",
        "//modules/local_planning/planning_context:input_gate",
        "//modules/common/status",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "lane_follow_planner",
    srcs = ["lane_follow_planner.cc"],
    hdrs = ["lane_follow_planner.h"],
    deps = [
        ":scenario",
        "//modules/local_planning/trajectory_continuity:trajectory_continuity",
        "//modules/local_planning/trajectory_continuity:trajectory_validation",
        "//modules/local_planning/planning_context:environment_geometry",
        "//modules/local_planning/planning_context:planning_input_builder",
        "//modules/local_planning/planning_context:trajectory_geometry",
        "//modules/common/math:geometry",
        "//modules/common/status",
    ],
)

cc_library(
    name = "closed_loop",
    srcs = ["simulation/closed_loop.cc"],
    hdrs = ["simulation/closed_loop.h"],
    deps = [
        "//modules/local_planning/planning_context:input_gate",
        ":lane_follow_planner",
        "//modules/common/status",
        "//modules/simulation:simulation_engine",
        "//modules/simulation:simulation_types",
        "//modules/world_model/local_map:local_scene_builder",
    ],
)

cc_binary(
    name = "local_planning_sim",
    srcs = ["simulation/main.cc"],
    data = ["//modules/simulation:runtime_data"],
    deps = [
        ":closed_loop",
        "//modules/common/status",
    ],
)

cc_test(
    name = "lane_follow_planner_test",
    size = "small",
    srcs = ["lane_follow_planner_test.cc"],
    deps = [
        ":closed_loop",
        ":lane_follow_planner",
        "//modules/local_planning/planning_context:input_gate",
        "//modules/common/status",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "closed_loop_test",
    size = "medium",
    srcs = ["simulation/closed_loop_test.cc"],
    data = ["//modules/simulation:runtime_data"],
    deps = [
        ":closed_loop",
        "//modules/common/status",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "framework_test",
    size = "small",
    srcs = ["framework_test.cc"],
    deps = [
        ":closed_loop",
        ":lane_follow_planner",
        "//modules/local_planning/trajectory_continuity:trajectory_validation",
        "//modules/local_planning/planning_context:corridor_selector",
        "//modules/local_planning/planning_context:environment_geometry",
        "//modules/local_planning/planning_context:planning_geometry",
        "//modules/local_planning/planning_context:reference_line",
        "//modules/local_planning/planning_context:trajectory_geometry",
        "//modules/world_model/local_map:local_scene_builder",
        "//modules/world_model/local_map:navigation_prior",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
