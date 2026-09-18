load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

cc_library(
    name = "narrow_street_u_turn",
    copts = PLANNING_COPTS,
    deps = [
        ":narrow_street_u_turn_scenario",
    ],
)

cc_library(
    name = "narrow_street_u_turn_scenario",
    srcs = ["narrow_street_u_turn_scenario.cc"],
    hdrs = ["narrow_street_u_turn_scenario.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning/scenarios:scenario",
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:planning_cc_proto",
    ],
)

cc_test(
    name = "narrow_street_u_turn_scenario_test",
    size = "small",
    srcs = ["narrow_street_u_turn_scenario_test.cc"],
    data = [
        "//modules/planning:planning_conf",
    ],
    linkopts = ["-lgomp"],
    linkstatic = True,
    deps = [
        ":narrow_street_u_turn",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
