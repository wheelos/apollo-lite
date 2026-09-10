load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "open_space_planning_component_lib",
    srcs = ["open_space_planning_component.cc"],
    hdrs = ["open_space_planning_component.h"],
    copts = [
        "-DMODULE_NAME=\\\"open_space_planning\\\"",
    ],
    deps = [
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/open_space_planning/common:types",
        "//modules/open_space_planning/proto:open_space_planning_cc_proto",
        "//modules/open_space_planning/route:skeleton_corridor_route_planner",
        "//modules/open_space_planning/runtime:open_space_planner",
        "//modules/open_space_planning/safety:default_fallback_planner",
        "//modules/open_space_planning/safety:default_trajectory_validator",
        "//modules/open_space_planning/trajectory:open_space_trajectory_planner",
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/chassis_msgs:chassis_cc_proto",
        "@wheelos_msgs//wheelos_msgs/localization_msgs:localization_cc_proto",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:planning_cc_proto",
        "@wheelos_msgs//wheelos_msgs/prediction_msgs:prediction_obstacle_cc_proto",
        "@wheelos_msgs//wheelos_msgs/routing_msgs:routing_cc_proto",
    ],
    alwayslink = True,
)

cc_binary(
    name = "libopen_space_planning_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":open_space_planning_component_lib"],
)

filegroup(
    name = "runtime_data",
    srcs = [
        "dag/open_space_planning.dag",
        "launch/open_space_planning.launch",
    ],
)

cpplint()
