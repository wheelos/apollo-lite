load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("@rules_cuda//cuda:defs.bzl", "cuda_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility=["//visibility:public"])

config_setting(
    name = "lidar_gpu_backend_enabled",
    values = {"define": "lidar_gpu_backend=true"},
)

config_setting(
    name = "lidar_mode_force_cpu",
    values = {"define": "lidar_policy_mode=cpu"},
)

config_setting(
    name = "lidar_mode_force_gpu",
    values = {"define": "lidar_policy_mode=gpu"},
)

cc_binary(
    name="liblidar_unified_component.so",
    linkshared=True,
    linkstatic=True,
    deps=[":lidar_unified_component_lib"],
)

cuda_library(
    name = "lidar_policy_cuda_kernels_lib",
    srcs = ["policy/lidar_policy_cuda_kernels.cu"],
    hdrs = ["policy/lidar_policy_cuda_kernels.h"],
    deps = [
        "@cuda//:cuda_runtime",
    ],
)

cc_library(
    name="lidar_policy_lib",
    srcs=[
        "policy/lidar_policy_common.cc",
        "policy/lidar_policy_cpu_deskew.cc",
        "policy/lidar_policy_cpu_filter.cc",
        "policy/lidar_policy_cpu_fusion.cc",
        "policy/lidar_policy_factory.cc",
        "policy/lidar_policy_gpu_deskew.cc",
        "policy/lidar_policy_gpu_filter.cc",
        "policy/lidar_policy_gpu_fusion.cc",
    ],
    hdrs=[
        "policy/cpu_lidar_policy.h",
        "policy/gpu_lidar_policy.h",
        "policy/lidar_policy_common.h",
        "policy/lidar_policy_interface.h",
        "policy/lidar_policy_cuda_kernels.h",
    ],
    copts = select({
        ":lidar_gpu_backend_enabled": ["-DAPOLLO_LIDAR_POLICY_GPU_ENABLED"],
        "//conditions:default": [],
    }) + select({
        ":lidar_mode_force_cpu": ["-DAPOLLO_LIDAR_POLICY_FORCE_CPU"],
        ":lidar_mode_force_gpu": ["-DAPOLLO_LIDAR_POLICY_FORCE_GPU"],
        "//conditions:default": [],
    }),
    deps=[
        "//cyber",
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/drivers/lidar/proto:lidar_unified_component_config_cc_proto",
        "//modules/transform:buffer",
        "@eigen",
    ] + select({
        ":lidar_gpu_backend_enabled": [":lidar_policy_cuda_kernels_lib"],
        "//conditions:default": [],
    }),
)

cc_library(
    name="safety_policy_lib",
    srcs=[
        "safety/degrade_policy.cc",
        "safety/dtc_reporter.cc",
        "safety/ts_sanity.cc",
    ],
    hdrs=[
        "safety/degrade_policy.h",
        "safety/dtc_reporter.h",
        "safety/ts_sanity.h",
    ],
    deps=[
        "//cyber",
    ],
)

cc_library(
    name="lidar_unified_component_lib",
    srcs=[
        "control/pose_bins_builder.cc",
        "control/sync_gate.cc",
        "lidar_unified_component.cc",
        "lidar_unified_component_config.cc",
        "lidar_unified_component_ingest.cc",
        "lidar_unified_component_observability.cc",
    ],
    hdrs=[
        "control/frame_handle.h",
        "control/pose_bins_builder.h",
        "control/sync_gate.h",
        "lidar_unified_component.h",
        "policy/cpu_lidar_policy.h",
        "policy/gpu_lidar_policy.h",
        "policy/lidar_policy_interface.h",
    ],
    copts=['-DMODULE_NAME=\\"lidar_unified_component\\"'] + select({
        ":lidar_mode_force_cpu": ["-DAPOLLO_LIDAR_POLICY_FORCE_CPU"],
        ":lidar_mode_force_gpu": ["-DAPOLLO_LIDAR_POLICY_FORCE_GPU"],
        "//conditions:default": [],
    }),
    deps=[
        "//cyber",
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/drivers/lidar/proto:lidar_unified_component_config_cc_proto",
        ":lidar_policy_lib",
        ":safety_policy_lib",
        "//modules/transform:buffer",
        "//modules/transform:pose_cache",
        "@boost.circular_buffer",
        "@eigen",
    ],
    alwayslink=True,
)

cc_test(
    name="lidar_policy_test",
    size="small",
    srcs=[
        "lidar_policy_test.cc",
        "lidar_policy_test_main.cc",
    ],
    copts = select({
        ":lidar_gpu_backend_enabled": ["-DAPOLLO_LIDAR_POLICY_GPU_ENABLED"],
        "//conditions:default": [],
    }) + select({
        ":lidar_mode_force_cpu": ["-DAPOLLO_LIDAR_POLICY_FORCE_CPU"],
        ":lidar_mode_force_gpu": ["-DAPOLLO_LIDAR_POLICY_FORCE_GPU"],
        "//conditions:default": [],
    }),
    deps=[
        ":lidar_unified_component_lib",
        ":lidar_policy_lib",
        "//modules/common_msgs/transform_msgs:transform_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
