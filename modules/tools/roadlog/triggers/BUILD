load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "drive_event_trigger",
    srcs = ["drive_event_trigger.cc"],
    hdrs = ["drive_event_trigger.h"],
    deps = [
        ":trigger_base",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common_msgs/basic_msgs:drive_event_cc_proto",
    ],
)

cc_library(
    name = "emergency_mode_trigger",
    srcs = ["emergency_mode_trigger.cc"],
    hdrs = ["emergency_mode_trigger.h"],
    deps = [
        ":trigger_base",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
    ],
)

cc_library(
    name = "bumper_crash_trigger",
    srcs = ["bumper_crash_trigger.cc"],
    hdrs = ["bumper_crash_trigger.h"],
    deps = [
        ":trigger_base",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common/adapters:adapter_gflags",
    ],
)

cc_library(
    name = "hard_brake_trigger",
    srcs = ["hard_brake_trigger.cc"],
    hdrs = ["hard_brake_trigger.h"],
    deps = [
        ":trigger_base",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
    ],
)

cc_library(
    name = "small_topics_trigger",
    srcs = ["small_topics_trigger.cc"],
    hdrs = ["small_topics_trigger.h"],
    deps = [
        "//modules/tools/roadlog/common:channel_pool",
        ":trigger_base",
    ],
)

cc_library(
    name = "swerve_trigger",
    srcs = ["swerve_trigger.cc"],
    hdrs = ["swerve_trigger.h"],
    deps = [
        ":trigger_base",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
    ],
)

cc_library(
    name = "regular_interval_trigger",
    srcs = ["regular_interval_trigger.cc"],
    hdrs = ["regular_interval_trigger.h"],
    deps = [
        ":trigger_base",
    ],
)

cc_library(
    name = "trigger_base",
    srcs = ["trigger_base.cc"],
    hdrs = ["trigger_base.h"],
    deps = [
        "//cyber",
        "//modules/tools/roadlog/common:interval_pool",
        "//modules/tools/roadlog/proto:smart_recorder_triggers_cc_proto",
    ],
)

cpplint()
