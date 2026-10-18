load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install", "install_files")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "model_input",
    srcs = ["types/model_input.cc"],
    hdrs = ["types/model_input.h"],
    deps = [
        "//modules/lidar_object_detection/proto:lidar_object_detection_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
    ],
)

cc_library(
    name = "sweep_fusion",
    srcs = ["types/sweep_fusion.cc"],
    hdrs = ["types/sweep_fusion.h"],
    deps = [
        "@eigen",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
    ],
)

cc_library(
    name = "centerpoint_runner",
    srcs = ["inference/centerpoint_runner.cc"],
    hdrs = [
        "inference/centerpoint_backend.h",
        "inference/centerpoint_runner.h",
    ],
    deps = [
        ":model_input",
        "//modules/lidar_object_detection/proto:lidar_object_detection_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
    ],
)

cc_library(
    name = "centerpoint_runtime_backend",
    srcs = ["inference/centerpoint_runtime_backend.cc"],
    hdrs = ["inference/centerpoint_runtime_backend.h"],
    linkopts = ["-ldl"],
    deps = [
        ":centerpoint_runner",
        "//modules/lidar_object_detection/proto:lidar_object_detection_cc_proto",
        "//modules/lidar_object_detection/third_party/centerpoint_runtime:runtime_api",
    ],
)

cc_library(
    name = "centerpoint_component_lib",
    srcs = ["component/centerpoint_component.cc"],
    hdrs = ["component/centerpoint_component.h"],
    copts = ['-DMODULE_NAME=\\"lidar_object_detection\\"'],
    deps = [
        ":centerpoint_runner",
        ":centerpoint_runtime_backend",
        ":sweep_fusion",
        "//modules/lidar_object_detection/proto:lidar_object_detection_cc_proto",
        "//modules/perception/onboard/transform_wrapper",
        "@core//cyber",
        "@eigen",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
    ],
    alwayslink = True,
)

cc_binary(
    name = "libcenterpoint_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":centerpoint_component_lib"],
)

cc_binary(
    name = "libcenterpoint_runtime.so",
    linkshared = True,
    linkstatic = True,
    linkopts = ["-Wl,-rpath,$$ORIGIN"],
    deps = [
        "//modules/lidar_object_detection/third_party/centerpoint_runtime:runtime_core",
    ],
)

cc_library(
    name = "offline_result_writer",
    srcs = ["tools/offline_result_writer.cc"],
    hdrs = ["tools/offline_result_writer.h"],
    deps = [
        "//modules/lidar_object_detection/proto:lidar_object_detection_cc_proto",
    ],
)

cc_binary(
    name = "pcd_to_centerpoint_input",
    srcs = ["tools/pcd_to_centerpoint_input.cc"],
    deps = [
        ":model_input",
        "//modules/perception/lidar/common:pcl_util",
        "@com_github_gflags_gflags//:gflags",
        "@com_google_protobuf//:protobuf",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
    ],
)

cc_test(
    name = "model_input_test",
    size = "small",
    srcs = ["types/model_input_test.cc"],
    deps = [
        ":model_input",
        "@com_google_googletest//:gtest_main",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
    ],
)

cc_test(
    name = "sweep_fusion_test",
    size = "small",
    srcs = ["types/sweep_fusion_test.cc"],
    deps = [
        ":model_input",
        ":sweep_fusion",
        "@com_google_googletest//:gtest_main",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
    ],
)

filegroup(
    name = "runtime_config",
    srcs = glob([
        "conf/**",
        "dag/**",
    ]),
)

install_files(
    name = "install_centerpoint_spconv",
    files = ["@local_config_spconv//:spconv_shared_library"],
    dest = "lidar_object_detection/lib",
    strip_prefix = ["third_party/_main~_repo_rules~local_config_spconv"],
)

install(
    name = "install",
    deps = [":install_centerpoint_spconv"],
    data = [
        ":runtime_config",
    ],
    data_dest = "lidar_object_detection",
    library_dest = "lidar_object_detection/lib",
    targets = [
        ":libcenterpoint_component.so",
        ":libcenterpoint_runtime.so",
    ],
)

cpplint()
