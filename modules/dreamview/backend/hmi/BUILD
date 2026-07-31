load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

DREAMVIEW_COPTS = ['-DMODULE_NAME=\\"dreamview\\"']

install(
    name = "install",
    library_dest = "dreamview/lib",
    runtime_dest = "dreamview/bin",
    targets = [
        ":hmi_worker",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "hmi",
    srcs = ["hmi.cc"],
    hdrs = ["hmi.h"],
    copts = DREAMVIEW_COPTS,
    alwayslink = True,
    deps = [
        ":hmi_worker",
        "@core//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/monitor_log",
        "//modules/common/util:util_tool",
        "//modules/common/util:string_util",
        "//modules/dreamview/backend/common:dreamview_gflags",
        "//modules/dreamview/backend/handlers:websocket_handler",
        "//modules/dreamview/backend/map:map_service",
        "//modules/dreamview/backend/point_cloud:point_cloud_updater",
        "//modules/dreamview/proto:preprocess_table_cc_proto",
    ],
)

cc_library(
    name = "hmi_worker",
    srcs = [
        "hmi_status_bridge.cc",
        "hmi_worker.cc",
        "local_runner.cc",
        "mode_registry.cc",
        "process_manager.cc",
        "readiness_probe.cc",
    ],
    hdrs = [
        "hmi_status_bridge.h",
        "hmi_worker.h",
        "local_runner.h",
        "mode_registry.h",
        "process_manager.h",
        "process_types.h",
        "readiness_probe.h",
    ],
    copts = DREAMVIEW_COPTS,
    deps = [
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/audio_msgs:audio_event_cc_proto",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:drive_event_cc_proto",
        "@wheelos_msgs//wheelos_msgs/chassis_msgs:chassis_cc_proto",
        "@wheelos_msgs//wheelos_msgs/control_msgs:pad_msg_cc_proto",
        "@wheelos_msgs//wheelos_msgs/localization_msgs:localization_cc_proto",
        "@wheelos_msgs//wheelos_msgs/monitor_msgs:system_status_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/configs:config_gflags",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/kv_db",
        "//modules/common/util:util_tool",
        "//modules/dreamview/backend/common:dreamview_gflags",
        "//modules/dreamview/proto:hmi_config_cc_proto",
        "@wheelos_msgs//wheelos_msgs/dreamview_msgs:hmi_status_cc_proto",
        "//modules/dreamview/proto:scenario_cc_proto",
        "//modules/dreamview/proto:record_cc_proto",
        "//modules/dreamview/proto:hmi_mode_cc_proto",
        "@boost.thread",
        "@com_google_absl//absl/strings",
    ],
)

cc_test(
    name = "hmi_worker_test",
    size = "small",
    srcs = ["hmi_worker_test.cc"],
    data = ["//modules/dreamview:dreamview_conf"],
    deps = [
        ":hmi_worker",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
