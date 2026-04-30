load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])


cc_library(
    name = "mission_command_supervisor",
    srcs = ["mission_command_supervisor.cc"],
    hdrs = ["mission_command_supervisor.h"],
    deps = [
        "//modules/common_msgs/control_msgs:control_runtime_status_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_command_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_runtime_status_cc_proto",
    ],
)

cc_library(
    name = "mission_context",
    srcs = ["mission_context.cc"],
    hdrs = ["mission_context.h"],
    deps = [
        ":mission_command_supervisor",
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/common_msgs/basic_msgs:geometry_cc_proto",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/control_msgs:control_runtime_status_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_command_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_runtime_status_cc_proto",
        "//modules/common_msgs/routing_msgs:routing_cc_proto"
    ],
)

cc_test(
    name = "mission_command_supervisor_test",
    size = "small",
    srcs = ["mission_command_supervisor_test.cc"],
    deps = [
        ":mission_command_supervisor",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "bt_groot_bridge",
    hdrs = ["bt_groot_bridge.h"],
    deps = [
        "//cyber",
        "@behaviortree_cpp//:behaviortree_cpp"
    ],
)

cc_library(
    name = "bt_type_converters",
    hdrs = ["bt_type_converters.h"],
    deps = [
        "@behaviortree_cpp//:behaviortree_cpp"
    ],
)

cpplint()
