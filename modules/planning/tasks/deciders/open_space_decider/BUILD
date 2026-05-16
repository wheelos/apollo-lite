load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])
PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

cc_library(
    name = "open_space_roi_decider",
    srcs = ["open_space_roi_decider.cc"],
    hdrs = ["open_space_roi_decider.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/util:point_factory",
        "//modules/planning/common:dependency_injector",
        "//modules/planning/common:frame",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/open_space/parking:parking_pose_selector",
        "//modules/planning/open_space/parking:parking_roi_geometry",
        "//modules/planning/open_space/parking:parking_roi_validator",
        "//modules/planning/open_space/parking:parking_slot_provider",
        "//modules/planning/tasks/deciders:decider_base",
        "//modules/planning/common/util:common_lib",
    ],
)

cc_library(
    name = "open_space_pre_stop_decider",
    srcs = ["open_space_pre_stop_decider.cc"],
    hdrs = ["open_space_pre_stop_decider.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning/common:reference_line_info",
        "//modules/planning/tasks/deciders:decider_base",
        "//modules/planning/common/util:common_lib",
    ],
)

cc_library(
    name = "open_space_fallback_decider",
    srcs = ["open_space_fallback_decider.cc"],
    hdrs = ["open_space_fallback_decider.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning/common:dependency_injector",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/tasks/deciders:decider_base",
        "//modules/planning/common/util:common_lib",
    ],
)

cc_test(
    name = "open_space_roi_decider_test",
    size = "small",
    srcs = ["open_space_roi_decider_test.cc"],
    deps = [
        ":open_space_roi_decider",
        "//modules/planning/testing:gtest_exit_main",
    ],
    tags = ["exclude"]
)

cc_test(
    name = "open_space_fallback_decider_test",
    size = "small",
    srcs = ["open_space_fallback_decider_test.cc"],
    deps = [
        ":open_space_fallback_decider",
        "//modules/planning/common:dependency_injector",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
