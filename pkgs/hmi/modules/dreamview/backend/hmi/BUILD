load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:mappings.bzl", "pkg_attributes", "pkg_filegroup", "pkg_files", "strip_prefix")

package(default_visibility = ["//visibility:public"])

DREAMVIEW_COPTS = ['-DMODULE_NAME=\\"dreamview\\"']

pkg_files(
    name = "package_files_archives",
    srcs = [":hmi_worker"],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "lib",
    strip_prefix = strip_prefix.from_root("modules/dreamview/backend/hmi"),
)

pkg_filegroup(
    name = "package_files",
    srcs = [":package_files_archives"],
    visibility = ["//visibility:public"],
)

pkg_install(
    name = "install_dir",
    srcs = [":package_files"],
    tags = ["manual"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "hmi",
    srcs = ["hmi.cc"],
    hdrs = ["hmi.h"],
    copts = DREAMVIEW_COPTS,
    deps = [
        ":hmi_worker",
        "@wheelos_common//modules/common/adapters:adapter_gflags",
        "@wheelos_common//modules/common/configs:vehicle_config_helper",
        "@wheelos_common//modules/common/monitor_log",
        "@wheelos_common//modules/common/util:string_util",
        "@wheelos_common//modules/common/util:util_tool",
        "//modules/dreamview/backend/common:dreamview_gflags",
        "//modules/dreamview/backend/handlers:websocket_handler",
        "//modules/dreamview/backend/map:map_service",
        "//modules/dreamview/backend/point_cloud:point_cloud_updater",
        "//modules/dreamview/proto:preprocess_table_cc_proto",
        "@core//cyber",
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
        "@wheelos_common//modules/common/adapters:adapter_gflags",
        "@wheelos_common//modules/common/configs:config_gflags",
        "@wheelos_common//modules/common/configs:vehicle_config_helper",
        "@wheelos_common//modules/common/kv_db",
        "@wheelos_common//modules/common/map:map_selection",
        "@wheelos_common//modules/common/util:util_tool",
        "//modules/dreamview/backend/common:dreamview_gflags",
        "//modules/dreamview/proto:hmi_config_cc_proto",
        "//modules/dreamview/proto:hmi_mode_cc_proto",
        "//modules/dreamview/proto:record_cc_proto",
        "//modules/dreamview/proto:scenario_cc_proto",
        "@boost.thread",
        "@com_google_absl//absl/strings",
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/audio_msgs:audio_event_cc_proto",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:drive_event_cc_proto",
        "@wheelos_msgs//wheelos_msgs/chassis_msgs:chassis_cc_proto",
        "@wheelos_msgs//wheelos_msgs/control_msgs:pad_msg_cc_proto",
        "@wheelos_msgs//wheelos_msgs/dreamview_msgs:hmi_status_cc_proto",
        "@wheelos_msgs//wheelos_msgs/localization_msgs:localization_cc_proto",
        "@wheelos_msgs//wheelos_msgs/monitor_msgs:system_status_cc_proto",
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

filegroup(
    name = "exported_source_files",
    srcs = glob(
        ["**/*"],
        allow_empty = True,
    ),
    visibility = ["//visibility:public"],
)

filegroup(
    name = "exported_source_headers",
    srcs = glob(
        ["**/*.h"],
        allow_empty = True,
    ),
    visibility = ["//visibility:public"],
)
