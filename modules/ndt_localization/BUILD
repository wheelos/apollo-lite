load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ndt_localization_component_lib",
    srcs = [
        "localization_pose_buffer.cc",
        "ndt_localization.cc",
        "ndt_localization_component.cc",
    ],
    hdrs = [
        "localization_pose_buffer.h",
        "ndt_localization.h",
        "ndt_localization_component.h",
    ],
    copts = ["-DMODULE_NAME=\\\"NDTlocalization\\\""],
    deps = [
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/math",
        "//modules/common/monitor_log",
        "//modules/common/util:util_tool",
        "//modules/localization/common:localization_gflags",
        "//modules/localization/common:rigid_transform_helper",
        "//modules/localization/msf/common/util",
        "//modules/ndt_localization/ndt_locator:ndt_lidar_locator",
        "//modules/transform:transform_broadcaster",
        "//modules/transform:transform_query",
        "@com_github_google_glog//:glog",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@core//cyber",
        "@eigen",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:geometry_cc_proto",
        "@wheelos_msgs//wheelos_msgs/localization_msgs:gps_cc_proto",
        "@wheelos_msgs//wheelos_msgs/localization_msgs:localization_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:gnss_best_pose_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:gnss_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:ins_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
    ],
    alwayslink = True,
)

cc_binary(
    name = "libndt_localization_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":ndt_localization_component_lib"],
)

cc_test(
    name = "ndt_localization_test",
    size = "small",
    timeout = "short",
    srcs = ["ndt_localization_test.cc"],
    linkstatic = True,
    deps = [
        ":ndt_localization_component_lib",
        "@com_google_googletest//:gtest_main",
        "@core//cyber",
    ],
)

cc_test(
    name = "ndt_localization_pose_buffer_test",
    size = "small",
    timeout = "short",
    srcs = ["localization_pose_buffer_test.cc"],
    linkstatic = True,
    deps = [
        ":ndt_localization_component_lib",
        "@com_google_googletest//:gtest_main",
        "@core//cyber",
        "@eigen",
    ],
)

filegroup(
    name = "runtime_data",
    srcs = [
        "dag/dag_streaming_ndt_localization.dag",
        "conf/ndt_localization.conf",
        "launch/ndt_localization.launch",
    ],
)

filegroup(
    name = "test_data",
    srcs = glob(["test_data/**"]),
)

install(
    name = "install",
    data = [
        ":runtime_data",
        ":test_data",
    ],
    data_dest = "ndt_localization",
    library_dest = "ndt_localization/lib",
    targets = [
        ":libndt_localization_component.so",
    ],
)

cpplint()
