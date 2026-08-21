load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "collision_checker",
    srcs = ["collision_checker.cc"],
    hdrs = ["collision_checker.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "@core//cyber",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "@wheelos_msgs//wheelos_msgs/prediction_msgs:prediction_obstacle_cc_proto",
        "//modules/planning/common:frame",
        "//modules/planning/common:obstacle",
        "//modules/planning/common/trajectory:discretized_trajectory",
        "//modules/open_space_planning/lattice/behavior:path_time_graph",
        "//modules/planning/proto:st_drivable_boundary_cc_proto",
    ],
)

cpplint()
