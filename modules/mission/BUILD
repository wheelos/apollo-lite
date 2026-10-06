load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

MISSION_COPTS = ['-DMODULE_NAME=\\"mission\\"']

cc_library(
    name = "mission_component_lib",
    srcs = ["mission_component.cc"],
    hdrs = ["mission_component.h"],
    copts = MISSION_COPTS,
    deps = [
        "@wheelos_common//modules/common/adapters:adapter_gflags",
        "//modules/execution_state_sync:execution_state_gflags",
        "//modules/mission/common:mission_context",
        "//modules/mission/common:mission_request_contract",
        "//modules/mission/nodes/action",
        "//modules/mission/nodes/condition",
        "//modules/mission/proto:mission_config_cc_proto",
        "@behaviortree_cpp",
        "@core//cyber",
        "@tinyxml2",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:geometry_cc_proto",
        "@wheelos_msgs//wheelos_msgs/chassis_msgs:chassis_cc_proto",
        "@wheelos_msgs//wheelos_msgs/localization_msgs:localization_cc_proto",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:mission_directive_cc_proto",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:planning_command_cc_proto",
        "@wheelos_msgs//wheelos_msgs/mission_msgs:mission_runtime_status_cc_proto",
        "@wheelos_msgs//wheelos_msgs/mission_msgs:mission_request_cc_proto",
        "@wheelos_msgs//wheelos_msgs/mission_msgs:mission_request_result_cc_proto",
    ],
    alwayslink = True,
)

cc_binary(
    name = "libmission_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":mission_component_lib"],
)

cc_test(
    name = "mission_lifecycle_test",
    size = "small",
    srcs = ["mission_lifecycle_test.cc"],
    deps = [
        ":mission_component_lib",
        "//modules/execution_state_sync:execution_state_sync",
        "//modules/execution_state_sync:execution_state_gflags",
        "//modules/mission/common:mission_context",
        "//modules/mission/nodes/action:move_to_node",
        "//modules/mission/nodes/action:park_in_node",
        "@behaviortree_cpp",
        "@com_google_googletest//:gtest_main",
        "@core//cyber",
        "@wheelos_common//modules/common/adapters:adapter_gflags",
        "@wheelos_msgs//wheelos_msgs/control_msgs:safety_stop_cc_proto",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:planning_runtime_status_cc_proto",
    ],
)

cpplint()
