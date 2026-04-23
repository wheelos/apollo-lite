load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "post_record_processor",
    srcs = ["post_record_processor.cc"],
    hdrs = ["post_record_processor.h"],
    deps = [
        ":record_processor",
        "//cyber",
        "//modules/tools/roadlog/common:channel_pool",
    ],
)

cc_library(
    name = "realtime_record_processor",
    srcs = ["realtime_record_processor.cc"],
    hdrs = ["realtime_record_processor.h"],
    deps = [
        ":record_processor",
        "//cyber",
        "//cyber/tools/cyber_recorder:recorder",
        "//modules/tools/roadlog/common:channel_pool",
        "//modules/common_msgs/monitor_msgs:smart_recorder_status_cc_proto",
        "//modules/monitor/common:monitor_manager",
    ],
)

cc_library(
    name = "record_processor",
    srcs = ["record_processor.cc"],
    hdrs = ["record_processor.h"],
    deps = [
        "//cyber",
        "//modules/common/util",
        "//modules/tools/roadlog/common:interval_pool",
        "//modules/tools/roadlog/triggers:bumper_crash_trigger",
        "//modules/tools/roadlog/triggers:drive_event_trigger",
        "//modules/tools/roadlog/triggers:emergency_mode_trigger",
        "//modules/tools/roadlog/triggers:hard_brake_trigger",
        "//modules/tools/roadlog/triggers:regular_interval_trigger",
        "//modules/tools/roadlog/triggers:small_topics_trigger",
        "//modules/tools/roadlog/triggers:swerve_trigger",
    ],
)

cpplint()
