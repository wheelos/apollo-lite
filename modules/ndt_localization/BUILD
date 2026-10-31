load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:mappings.bzl", "pkg_attributes", "pkg_filegroup", "pkg_files", "strip_prefix")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ndt_localization_lib",
    srcs = [
        "localization_pose_buffer.cc",
        "ndt_localization.cc",
    ],
    hdrs = [
        "localization_pose_buffer.h",
        "ndt_localization.h",
    ],
    copts = ["-DMODULE_NAME=\\\"NDTlocalization\\\""],
    deps = [
        "//modules/ndt_localization/common:localization_gflags",
        "//modules/ndt_localization/common:rigid_transform_helper",
        "//modules/ndt_localization/common/util",
        "//modules/ndt_localization/ndt_locator:ndt_lidar_locator",
        "@wheelos_transform//modules/transform:buffer",
        "@wheelos_transform//modules/transform:transform_query",
        "@com_github_google_glog//:glog",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@core//cyber",
        "@eigen",
        "@wheelos_common//modules/common/map:map_selection",
        "@wheelos_common//modules/common/math",
        "@wheelos_common//modules/common/monitor_log",
        "@wheelos_common//modules/common/util:util_tool",
        "@wheelos_common//modules/common/vehicle_calibration:lidar_height",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:geometry_cc_proto",
        "@wheelos_msgs//wheelos_msgs/localization_msgs:gps_cc_proto",
        "@wheelos_msgs//wheelos_msgs/localization_msgs:localization_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:gnss_best_pose_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:gnss_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:ins_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
    ],
)

cc_library(
    name = "ndt_localization_component_lib",
    srcs = ["ndt_localization_component.cc"],
    hdrs = ["ndt_localization_component.h"],
    copts = ["-DMODULE_NAME=\\\"NDTlocalization\\\""],
    deps = [
        ":ndt_localization_lib",
        "//modules/ndt_localization/common:localization_gflags",
        "@wheelos_transform//modules/transform:transform_broadcaster",
        "@core//cyber",
        "@wheelos_common//modules/common/adapters:adapter_gflags",
        "@wheelos_common//modules/common/math",
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
    name = "ndt_extrinsic_test",
    size = "small",
    srcs = ["ndt_extrinsic_test.cc"],
    copts = ["-fno-access-control"],
    deps = [
        ":ndt_localization_lib",
        "@wheelos_transform//modules/transform:buffer",
        "@wheelos_transform//modules/transform:buffer_test_peer",
        "@com_google_googletest//:gtest_main",
        "@eigen",
    ],
)

cc_test(
    name = "ndt_localization_test",
    size = "small",
    timeout = "short",
    srcs = ["ndt_localization_test.cc"],
    deps = [
        ":ndt_localization_lib",
        "@com_google_googletest//:gtest_main",
        "@core//cyber",
    ],
)

cc_test(
    name = "ndt_localization_pose_buffer_test",
    size = "small",
    timeout = "short",
    srcs = ["localization_pose_buffer_test.cc"],
    deps = [
        ":ndt_localization_lib",
        "@com_google_googletest//:gtest_main",
        "@core//cyber",
        "@eigen",
    ],
)

filegroup(
    name = "runtime_data",
    srcs = [
        "conf/ndt_localization.conf",
        "dag/dag_streaming_ndt_localization.dag",
        "launch/ndt_localization.launch",
        "params/velodyne64_height.yaml",
    ],
)

filegroup(
    name = "test_data",
    srcs = glob(["test_data/**"]),
)

pkg_files(
    name = "package_files_data",
    srcs = [
        ":runtime_data",
        ":test_data",
    ],
    attributes = pkg_attributes(mode = "0644"),
    prefix = "ndt_localization",
    strip_prefix = strip_prefix.from_root("modules/ndt_localization"),
)

pkg_files(
    name = "package_files_libraries",
    srcs = [":libndt_localization_component.so"],
    attributes = pkg_attributes(mode = "0755"),
    prefix = "ndt_localization/lib",
    strip_prefix = strip_prefix.from_root("modules/ndt_localization"),
)

pkg_filegroup(
    name = "package_files",
    srcs = [
        ":package_files_data",
        ":package_files_libraries",
        "//modules/ndt_localization/common:package_files",
        "//modules/ndt_localization/map_creation:package_files",
    ],
)

pkg_install(
    name = "install_dir",
    srcs = [":package_files"],
    tags = ["manual"],
)
