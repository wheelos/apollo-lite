load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])


cc_library(
    name = "mission_context",
    srcs = ["mission_context.cc"],
    hdrs = ["mission_context.h"],
    deps = [
        "//cyber",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:geometry_cc_proto",
        "@wheelos_msgs//wheelos_msgs/chassis_msgs:chassis_cc_proto",
        "@wheelos_msgs//wheelos_msgs/localization_msgs:localization_cc_proto",
        "@wheelos_msgs//wheelos_msgs/routing_msgs:routing_cc_proto"
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
