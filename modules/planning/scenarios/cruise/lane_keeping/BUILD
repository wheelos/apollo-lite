load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

cc_library(
    name = "lane_keeping_scenario",
    srcs = ["lane_keeping_scenario.cc"],
    hdrs = ["lane_keeping_scenario.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//cyber",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/scenarios:scenario",
    ],
)

cpplint()
