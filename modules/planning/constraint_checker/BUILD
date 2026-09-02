load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

cc_library(
    name = "constraint_checker",
    srcs = ["constraint_checker.cc"],
    hdrs = ["constraint_checker.h"],
    copts = PLANNING_COPTS,
    deps = [
        "@core//cyber",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common/trajectory:discretized_trajectory",
    ],
)

cc_library(
    name = "constraint_checker1d",
    srcs = ["constraint_checker1d.cc"],
    hdrs = ["constraint_checker1d.h"],
    copts = PLANNING_COPTS,
    deps = [
        "@core//cyber",
        "//modules/planning/common:frame",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common/trajectory:discretized_trajectory",
        "//modules/planning/math/curve1d",
    ],
)

cpplint()
