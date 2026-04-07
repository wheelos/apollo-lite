load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

cc_library(
    name = "mission_idle",
    copts = PLANNING_COPTS,
    deps = [
        ":mission_idle_scenario",
        ":mission_idle_stage",
    ],
)

cc_library(
    name = "mission_idle_stage",
    srcs = ["mission_idle_stage.cc"],
    hdrs = ["mission_idle_stage.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//cyber",
        "//modules/planning/scenarios:stage",
    ],
)

cc_library(
    name = "mission_idle_scenario",
    srcs = ["mission_idle_scenario.cc"],
    hdrs = ["mission_idle_scenario.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":mission_idle_stage",
        "//cyber",
        "//modules/planning/scenarios:scenario",
    ],
)

cpplint()
