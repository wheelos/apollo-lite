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
        "//modules/common/adapters:adapter_gflags",
        "//modules/mission/common:mission_context",
        "//modules/mission/nodes/action",
        "//modules/mission/nodes/condition",
        "//modules/mission/proto:mission_config_cc_proto",
        "@behaviortree_cpp",
        "@core//cyber",
        "@tinyxml2",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:geometry_cc_proto",
        "@wheelos_msgs//wheelos_msgs/chassis_msgs:chassis_cc_proto",
        "@wheelos_msgs//wheelos_msgs/localization_msgs:localization_cc_proto",
        "@wheelos_msgs//wheelos_msgs/mission_msgs:mission_request_cc_proto",
        "@wheelos_msgs//wheelos_msgs/routing_msgs:routing_cc_proto",
    ],
    alwayslink = True,
)

cc_binary(
    name = "libmission_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":mission_component_lib"],
)

cpplint()
