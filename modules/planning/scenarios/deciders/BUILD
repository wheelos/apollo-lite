load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

cc_library(
    name = "all_deciders",
    copts = PLANNING_COPTS,
    deps = [
        ":cruise_decider",
        ":emergency_decider",
        ":escape_decider",
        ":intersection_decider",
        ":mission_decider",
        ":park_decider",
    ],
)


cc_library(
    name = "scenario_decider",
    hdrs = ["scenario_decider.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning/common:dependency_injector",
        "//modules/planning/common:frame",
        "//modules/planning/common:planning_common",
        "//modules/planning/scenarios:scenario",
    ],
)


cc_library(
    name = "cruise_decider",
    srcs = ["cruise_decider.cc"],
    hdrs = ["cruise_decider.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":scenario_decider",
        "//modules/planning/common:frame",
        "//modules/planning/common:dependency_injector",
    ],
)


cc_library(
    name = "emergency_decider",
    srcs = ["emergency_decider.cc"],
    hdrs = ["emergency_decider.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":scenario_decider",
        "//modules/planning/common:dependency_injector",
    ],
)


cc_library(
    name = "escape_decider",
    srcs = ["escape_decider.cc"],
    hdrs = ["escape_decider.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":scenario_decider",
        "@core//cyber/time:clock",
        "//modules/planning/common:dependency_injector",
    ],
)


cc_library(
    name = "intersection_decider",
    srcs = ["intersection_decider.cc"],
    hdrs = ["intersection_decider.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":scenario_decider",
        "//modules/map/hdmap:hdmap_util",
        "//modules/planning/common:dependency_injector",
    ],
)


cc_library(
    name = "mission_decider",
    srcs = ["mission_decider.cc"],
    hdrs = ["mission_decider.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":scenario_decider",
        "//modules/map/hdmap:hdmap_util",
        "//modules/planning/common:dependency_injector",
    ],
)


cc_library(
    name = "park_decider",
    srcs = ["park_decider.cc"],
    hdrs = ["park_decider.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":scenario_decider",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/util:point_factory",
        "//modules/map/hdmap:hdmap_util",
        "//modules/planning/common:dependency_injector",
    ],
)

cpplint()
