load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "roadlog_recording_service",
    srcs = ["roadlog_recording_service.cc"],
    hdrs = ["roadlog_recording_service.h"],
    deps = [
        "@core//cyber/common:log",
        "@core//cyber/proto:record_cc_proto",
        "@core//cyber/record:header_builder",
        "@core//cyber/tools/cyber_recorder:recorder",
        "//modules/tools/roadlog/common:roadlog_layout",
        "//modules/tools/roadlog/proto:smart_recorder_triggers_cc_proto",
    ],
)

cc_test(
    name = "roadlog_recording_service_test",
    size = "small",
    srcs = ["roadlog_recording_service_test.cc"],
    deps = [
        ":roadlog_recording_service",
        "//modules/planning/common:gtest_exit_main",
    ],
)

cc_library(
    name = "roadlog_trigger_manager",
    srcs = ["roadlog_trigger_manager.cc"],
    hdrs = ["roadlog_trigger_manager.h"],
    deps = [
        "@core//cyber",
        "//modules/tools/roadlog/proto:smart_recorder_triggers_cc_proto",
        "//modules/tools/roadlog/triggers:bumper_crash_trigger",
        "//modules/tools/roadlog/triggers:drive_event_trigger",
        "//modules/tools/roadlog/triggers:emergency_mode_trigger",
        "//modules/tools/roadlog/triggers:hard_brake_trigger",
        "//modules/tools/roadlog/triggers:regular_interval_trigger",
        "//modules/tools/roadlog/triggers:swerve_trigger",
    ],
)

cc_library(
    name = "roadlog_segment_manager",
    srcs = ["roadlog_segment_manager.cc"],
    hdrs = [
        "roadlog_runtime_types.h",
        "roadlog_segment_manager.h",
    ],
    deps = [
        "@core//cyber/common:file",
        "@core//cyber/common:log",
        "@core//cyber/record:record_file_reader",
        "//modules/tools/roadlog/common:roadlog_layout",
        "//modules/tools/roadlog/common:trigger_arbiter",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "roadlog_event_manager",
    srcs = ["roadlog_event_manager.cc"],
    hdrs = ["roadlog_event_manager.h"],
    deps = [
        ":roadlog_segment_manager",
        "//modules/tools/roadlog/common:trigger_arbiter",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "roadlog_event_exporter",
    srcs = ["roadlog_event_exporter.cc"],
    hdrs = ["roadlog_event_exporter.h"],
    deps = [
        ":roadlog_segment_manager",
        "@core//cyber/common:file",
        "@core//cyber/common:log",
        "@core//cyber/time",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "roadlog_runtime",
    srcs = ["roadlog_runtime.cc"],
    hdrs = ["roadlog_runtime.h"],
    deps = [
        ":roadlog_event_exporter",
        ":roadlog_event_manager",
        ":roadlog_recording_service",
        ":roadlog_segment_manager",
        ":roadlog_trigger_manager",
        "@core//cyber",
        "@core//cyber/common:file",
        "@core//cyber/common:log",
        "@core//cyber:init",
        "@wheelos_msgs//wheelos_msgs/monitor_msgs:smart_recorder_status_cc_proto",
        "//modules/monitor/common:monitor_manager",
        "//modules/tools/roadlog/common:roadlog_layout",
        "@com_google_absl//absl/strings",
    ],
)

cpplint()
