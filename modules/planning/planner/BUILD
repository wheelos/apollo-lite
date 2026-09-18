load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "planner",
    hdrs = ["planner.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common/status",
        "//modules/planning/common:frame",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/scenarios:scenario",
        "//modules/planning/scenarios:scenario_manager",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:pnc_point_cc_proto",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:planning_cc_proto",
    ],
)

cc_library(
    name = "planner_dispatcher",
    srcs = [
        "navi_planner_dispatcher.cc",
        "on_lane_planner_dispatcher.cc",
        "planner_dispatcher.cc",
    ],
    hdrs = [
        "navi_planner_dispatcher.h",
        "on_lane_planner_dispatcher.h",
        "planner_dispatcher.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
        "-fopenmp",
    ],
    deps = [
        ":planner",
        "//modules/common/status",
        "//modules/common/util",
        "//modules/planning/planner/navi:navi_planner",
        "//modules/planning/planner/public_road:public_road_planner",
        "//modules/planning/planner/rtk:rtk_planner",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:planning_cc_proto",
    ],
)

cc_test(
    name = "on_lane_planner_dispatcher_test",
    size = "small",
    srcs = ["on_lane_planner_dispatcher_test.cc"],
    linkopts = ["-lgomp"],
    linkstatic = True,
    deps = [
        ":planner_dispatcher",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "navi_planner_dispatcher_test",
    size = "small",
    srcs = ["navi_planner_dispatcher_test.cc"],
    linkopts = ["-lgomp"],
    linkstatic = True,
    deps = [
        ":planner_dispatcher",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
