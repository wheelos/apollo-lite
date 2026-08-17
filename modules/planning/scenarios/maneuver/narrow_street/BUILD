load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]


cc_library(
    name = "narrow_street_scenario",
    srcs = ["narrow_street_scenario.cc"],
    hdrs = ["narrow_street_scenario.h"],
    copts = PLANNING_COPTS,
    deps = [
        "@core//cyber",
        "//modules/common/configs:vehicle_config_helper",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/scenarios:scenario",
    ],
)


cpplint()
