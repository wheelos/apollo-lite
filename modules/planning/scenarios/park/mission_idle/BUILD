load("@rules_cc//cc:defs.bzl", "cc_library")

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
        "//modules/planning/scenarios:stage",
        "@core//cyber",
    ],
)

cc_library(
    name = "mission_idle_scenario",
    srcs = ["mission_idle_scenario.cc"],
    hdrs = ["mission_idle_scenario.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":mission_idle_stage",
        "//modules/planning/scenarios:scenario",
        "@core//cyber",
    ],
)

filegroup(
    name = "exported_source_files",
    srcs = glob(
        ["**/*"],
        allow_empty = True,
    ),
    visibility = ["//visibility:public"],
)

filegroup(
    name = "exported_source_headers",
    srcs = glob(
        ["**/*.h"],
        allow_empty = True,
    ),
    visibility = ["//visibility:public"],
)
