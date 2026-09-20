load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

cc_library(
    name = "escape_scenario",
    srcs = ["escape_scenario.cc"],
    hdrs = ["escape_scenario.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning/scenarios:scenario",
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:planning_cc_proto",
    ],
)

cpplint()
