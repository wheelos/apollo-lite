load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

MISSION_COPTS = ['-DMODULE_NAME=\\"mission\\"']

cc_library(
    name = "mission_component_lib",
    srcs = ["mission_component.cc"],
    hdrs = ["mission_component.h"],
    copts = MISSION_COPTS,
    deps = [
        "//cyber",
        "//modules/mission/nodes/action",
        "//modules/mission/nodes/condition",
        "//modules/mission/common:mission_context",
        "//modules/common_msgs/basic_msgs:geometry_cc_proto",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/control_msgs:control_runtime_status_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/common_msgs/planning_msgs:mission_directive_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_command_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_runtime_status_cc_proto",
        "//modules/common_msgs/mission_msgs:mission_runtime_status_cc_proto",
        "//modules/common_msgs/mission_msgs:mission_request_cc_proto",
        "//modules/mission/proto:mission_config_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "@behaviortree_cpp//:behaviortree_cpp",
        "@tinyxml2//:tinyxml2"
    ],
    alwayslink = True,
)

cc_binary(
    name = "libmission_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":mission_component_lib"],
)

cpplint()
