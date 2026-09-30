load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "conf/*.conf",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

config_setting(
    name = "lidar_mock_tf",
    values = {"define": "lidar_mock_tf=true"},
)

cc_binary(
    name = "liblidar_unified_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":lidar_unified_component_lib"],
)

cc_library(
    name = "lidar_policy_lib",
    srcs = [
        "common/pod_pointcloud_view.cc",
        "policy/lidar_policy_common.cc",
        "policy/lidar_policy_cpu_deskew.cc",
        "policy/lidar_policy_cpu_filter.cc",
        "policy/lidar_policy_cpu_fusion.cc",
    ],
    hdrs = [
        "common/pod_pointcloud_view.h",
        "policy/lidar_policy_common.h",
        "policy/lidar_policy_cpu.h",
        "policy/lidar_policy_interface.h",
    ],
    copts = select({
        "@platforms//cpu:aarch64": [
            "-march=armv8-a+simd",
            "-DAPOLLO_LIDAR_USE_NEON=1",
        ],
        "//conditions:default": ["-DAPOLLO_LIDAR_USE_NEON=0"],
    }),
    deps = [
        "//modules/drivers/lidar/proto:lidar_unified_component_config_cc_proto",
        "//modules/transform:buffer",
        "//modules/transform:transform_query",
        "@core//cyber",
        "@eigen",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
    ],
)

cc_library(
    name = "safety_policy_lib",
    srcs = [
        "safety/degrade_policy.cc",
        "safety/dtc_reporter.cc",
        "safety/ts_sanity.cc",
    ],
    hdrs = [
        "safety/degrade_policy.h",
        "safety/dtc_reporter.h",
        "safety/ts_sanity.h",
    ],
    deps = [
        "@core//cyber",
    ],
)

cc_library(
    name = "lidar_unified_component_lib",
    srcs = [
        "control/pose_bins_builder.cc",
        "control/sync_gate.cc",
        "control/time_contract.cc",
        "lidar_unified_component.cc",
        "lidar_unified_component_input.cc",
        "lidar_unified_component_observability.cc",
        "lidar_unified_component_validation.cc",
    ],
    hdrs = [
        "common/pod_pointcloud_view.h",
        "control/frame_handle.h",
        "control/pose_bins_builder.h",
        "control/sync_gate.h",
        "control/time_contract.h",
        "lidar_unified_component.h",
        "policy/lidar_policy_cpu.h",
        "policy/lidar_policy_interface.h",
    ],
    copts = ['-DMODULE_NAME=\\\"lidar_unified_component\\\"'] + select({
        ":lidar_mock_tf": ["-DAPOLLO_LIDAR_PROCESSOR_MOCK_TF"],
        "//conditions:default": [],
    }),
    deps = [
        ":lidar_policy_lib",
        ":safety_policy_lib",
        "//modules/drivers/lidar/proto:lidar_unified_component_config_cc_proto",
        "//modules/transform:buffer",
        "//modules/transform:timed_transform_resolver",
        "//modules/transform:transform_query",
        "@boost.circular_buffer",
        "@core//cyber",
        "@eigen",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
    ],
    alwayslink = True,
)

cc_test(
    name = "lidar_policy_test",
    size = "small",
    srcs = ["policy/lidar_policy_test.cc"],
    deps = [
        ":lidar_policy_lib",
        ":lidar_unified_component_lib",
        "@com_google_googletest//:gtest",
        "@wheelos_msgs//wheelos_msgs/transform_msgs:transform_cc_proto",
    ],
)

cpplint()
