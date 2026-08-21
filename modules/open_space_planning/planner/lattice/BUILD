load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lattice_planner",
    srcs = ["lattice_planner.cc"],
    hdrs = ["lattice_planner.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "@core//cyber",
        "//modules/common/math",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/constraint_checker",
        "//modules/open_space_planning/constraint_checker:collision_checker",
        "//modules/open_space_planning/lattice/behavior:path_time_graph",
        "//modules/open_space_planning/lattice/trajectory_generation:backup_trajectory_generator",
        "//modules/open_space_planning/lattice/trajectory_generation:lattice_trajectory1d",
        "//modules/open_space_planning/lattice/trajectory_generation:trajectory1d_generator",
        "//modules/open_space_planning/lattice/trajectory_generation:trajectory_combiner",
        "//modules/open_space_planning/lattice/trajectory_generation:trajectory_evaluator",
        "//modules/planning/planner",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:planning_cc_proto",
    ],
)

cpplint()
