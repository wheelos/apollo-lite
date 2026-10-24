load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "planner",
    hdrs = ["planner.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "@wheelos_common//modules/common/status",
        "//modules/planning/common:frame",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/scenarios:scenario",
        "//modules/planning/scenarios:scenario_manager",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:pnc_point_cc_proto",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:planning_cc_proto",
    ],
)

cc_library(
    name = "planner_selector",
    srcs = ["planner_selector.cc"],
    hdrs = ["planner_selector.h"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
        "-fopenmp",
    ],
    deps = [
        ":planner",
        "@wheelos_common//modules/common/status",
        "//modules/planning/common:dependency_injector",
        "//modules/planning/planner/public_road:public_road_planner",
        "//modules/planning/proto:planning_config_cc_proto",
    ],
)

cc_test(
    name = "planner_selector_test",
    size = "small",
    srcs = ["planner_selector_test.cc"],
    linkopts = ["-lgomp"],
    deps = [
        ":planner_selector",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
