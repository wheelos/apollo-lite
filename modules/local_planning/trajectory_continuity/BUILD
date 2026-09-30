load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "trajectory_validation",
    srcs = ["trajectory_validation.cc"],
    hdrs = [
        "trajectory.h",
        "trajectory_validation.h",
    ],
    deps = [
        "//modules/local_planning/planning_context:input_contract",
        "//modules/local_planning/planning_context:environment_geometry",
        "//modules/local_planning/planning_context:input_gate",
        "//modules/local_planning/planning_context:trajectory_geometry",
        "//modules/common/math:geometry",
        "//modules/common/status",
    ],
)

cc_library(
    name = "trajectory_continuity",
    srcs = ["trajectory_continuity.cc"],
    hdrs = ["trajectory_continuity.h"],
    deps = [
        ":trajectory_validation",
        "//modules/common/status",
    ],
)

cc_test(
    name = "trajectory_continuity_test",
    size = "small",
    srcs = ["trajectory_continuity_test.cc"],
    deps = [
        ":trajectory_continuity",
        "@com_google_googletest//:gtest_main",
    ],
)
