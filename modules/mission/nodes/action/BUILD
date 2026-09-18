load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "action",
    deps = [
        ":charge_node",
        ":move_to_node",
        ":send_pad_node",
        ":station_wait_node",
    ],
)

cc_library(
    name = "send_pad_node",
    srcs = ["send_pad_node.cc"],
    hdrs = ["send_pad_node.h"],
    deps = [
        "//modules/mission/common:mission_context",
        "@behaviortree_cpp",
        "@core//cyber",
    ],
)

cc_library(
    name = "charge_node",
    srcs = ["charge_node.cc"],
    hdrs = ["charge_node.h"],
    deps = [
        "//modules/mission/common:mission_context",
        "@behaviortree_cpp",
        "@core//cyber",
    ],
)

cc_library(
    name = "move_to_node",
    srcs = ["move_to_node.cc"],
    hdrs = ["move_to_node.h"],
    deps = [
        "//modules/mission/common:bt_type_converters",
        "//modules/mission/common:mission_context",
        "@behaviortree_cpp",
        "@core//cyber",
    ],
)

cc_library(
    name = "station_wait_node",
    srcs = ["station_wait_node.cc"],
    hdrs = ["station_wait_node.h"],
    deps = [
        "//modules/mission/common:mission_context",
        "@behaviortree_cpp",
        "@core//cyber",
    ],
)

cpplint()
