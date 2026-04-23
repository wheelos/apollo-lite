load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

cc_library(
    name = "scenario",
    srcs = ["scenario.cc"],
    hdrs = ["scenario.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":stage",
        "//cyber/common:file",
        "//cyber/common:log",
        "//modules/common/status",
        "//modules/planning/common:frame",
        "//modules/planning/common:dependency_injector",
        "//modules/planning/proto:planning_config_cc_proto",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "stage",
    srcs = ["stage.cc"],
    hdrs = ["stage.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/status",
        "//modules/planning/common:frame",
        "//modules/planning/common:planning_context",
        "//modules/planning/common/util:util_lib",
        "//modules/planning/proto:planning_config_cc_proto",
        "//modules/planning/tasks:task",
        "//modules/planning/tasks:task_factory",
    ],
)

cc_library(
    name = "transition_guard",
    srcs = ["transition_guard.cc"],
    hdrs = ["transition_guard.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":scenario",
        "//modules/planning/proto:planning_config_cc_proto",
    ],
)

cc_library(
    name = "scenario_manager",
    srcs = ["scenario_manager.cc"],
    hdrs = ["scenario_manager.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":scenario",
        ":transition_guard",
        "//modules/common/status",
        "//modules/planning/common:frame",
        "//modules/planning/common:dependency_injector",
        "//modules/planning/scenarios/deciders:all_deciders",
        "//modules/planning/scenarios/park/valet_parking",
        "//modules/planning/scenarios/park/pull_over:pull_over_scenario",
        "//modules/planning/scenarios/park/park_and_go:park_and_go_scenario",
        "//modules/planning/scenarios/park/mission_idle:mission_idle_scenario",
        "//modules/planning/scenarios/learning_model:learning_model_sample_scenario",
        "//modules/planning/scenarios/intersection/yield_sign:yield_sign_scenario",
        "//modules/planning/scenarios/intersection/traffic_light/unprotected_right_turn:traffic_light_unprotected_right_turn_scenario",
        "//modules/planning/scenarios/intersection/traffic_light/unprotected_left_turn:traffic_light_unprotected_left_turn_scenario",
        "//modules/planning/scenarios/intersection/traffic_light/protected:traffic_light_protected_scenario",
        "//modules/planning/scenarios/intersection/stop_sign/unprotected:stop_sign_unprotected_scenario",
        "//modules/planning/scenarios/intersection/bare_intersection/unprotected:bare_intersection_unprotected_scenario",
        "//modules/planning/scenarios/emergency/emergency_stop:emergency_stop_scenario",
        "//modules/planning/scenarios/emergency/emergency_pull_over:emergency_pull_over_scenario",
        "//modules/planning/scenarios/cruise/lane_follow:lane_follow_scenario",
        "//modules/planning/scenarios/cruise/lane_keeping:lane_keeping_scenario",
        "//modules/planning/scenarios/maneuver/narrow_street_u_turn:narrow_street_u_turn_scenario",
        "//modules/planning/scenarios/maneuver/narrow_street:narrow_street_scenario",
        "//modules/planning/scenarios/maneuver/escape:escape_scenario",
        "//modules/planning/scenarios/maneuver/deadend_turnaround:deadend_turnaround_scenario",
        "//modules/planning/proto:planning_config_cc_proto",
    ],
)

cpplint()
