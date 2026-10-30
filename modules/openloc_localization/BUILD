load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("@com_google_protobuf//bazel:proto_library.bzl", "proto_library")
load("@com_google_protobuf//bazel:cc_proto_library.bzl", "cc_proto_library")

package(default_visibility = ["//visibility:private"])

exports_files([
    "patches/eigen_required_unsupported_modules.patch",
    "patches/gtsam_sl4_cassert.patch",
])

cc_library(
    name = "native_runtime",
    includes = ["."],
    visibility = ["//visibility:public"],
    deps = [
        ":native_global_worker",
        ":native_lio_config",
        ":native_lio_worker",
        ":native_source_ingress",
    ],
)

cc_library(
    name = "native_graph_test_support",
    testonly = True,
    hdrs = [
        "native_global_test_input.h",
        "native_graph_test_input.h",
    ],
    includes = ["."],
    visibility = ["//visibility:public"],
    deps = [
        ":native_runtime",
        "@openloc//:native_odometry",
    ],
)

proto_library(
    name = "native_lio_host_config_proto",
    srcs = ["native_lio_host_config.proto"],
    strip_import_prefix = "",
    visibility = ["//visibility:public"],
    deps = ["@wheelos_msgs//wheelos_msgs/basic_msgs:geometry_proto"],
)

cc_proto_library(
    name = "native_lio_host_config_cc_proto",
    deps = [":native_lio_host_config_proto"],
)

cc_library(
    name = "native_source_ingress",
    srcs = ["native_source_ingress.cc"],
    hdrs = ["native_source_ingress.h"],
)

cc_test(
    name = "native_source_ingress_test",
    size = "small",
    srcs = ["native_source_ingress_test.cc"],
    deps = [
        ":native_source_ingress",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "native_lio_config",
    srcs = ["native_lio_config.cc"],
    hdrs = ["native_lio_config.h"],
    deps = [
        ":native_lio_host_config_cc_proto",
        "@openloc//:native_odometry_types",
    ],
)

proto_library(
    name = "native_odometry_diagnostic_proto",
    srcs = ["native_odometry_diagnostic.proto"],
    strip_import_prefix = "",
    deps = [
        "@wheelos_msgs//wheelos_msgs/basic_msgs:geometry_proto",
        "@wheelos_msgs//wheelos_msgs/transform_msgs:transform_proto",
    ],
)

cc_proto_library(
    name = "native_odometry_diagnostic_cc_proto",
    deps = [":native_odometry_diagnostic_proto"],
)

cc_library(
    name = "native_lio_worker",
    srcs = ["native_lio_worker.cc"],
    hdrs = ["native_lio_worker.h"],
    deps = [
        ":native_publication",
        ":native_graph_transport",
        "@openloc//:native_odometry",
        "@openloc//:native_motion_bootstrap",
    ],
)

proto_library(
    name = "native_joint_graph_proto",
    srcs = ["native_joint_graph.proto"],
    strip_import_prefix = "",
    deps = [":native_odometry_diagnostic_proto"],
)

cc_proto_library(
    name = "native_joint_graph_cc_proto",
    deps = [":native_joint_graph_proto"],
)

cc_library(
    name = "native_graph_transport",
    srcs = ["native_graph_transport.cc"],
    hdrs = ["native_graph_transport.h"],
    deps = [
        ":native_joint_graph_cc_proto",
        ":native_publication",
        "@openloc//:fixed_lag_global_alignment",
    ],
)

cc_test(
    name = "native_graph_transport_test",
    size = "small",
    srcs = ["native_graph_transport_test.cc"],
    deps = [
        ":native_graph_transport",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "native_publication",
    srcs = ["native_publication.cc"],
    hdrs = ["native_publication.h"],
    deps = [
        ":native_odometry_diagnostic_cc_proto",
        "@openloc//:native_odometry_types",
    ],
)

cc_library(
    name = "native_cyber_publisher",
    srcs = ["native_cyber_publisher.cc"],
    hdrs = ["native_cyber_publisher.h"],
    deps = [
        ":native_publication",
        "@core//cyber",
    ],
)

proto_library(
    name = "native_global_diagnostic_proto",
    srcs = ["native_global_diagnostic.proto"],
    strip_import_prefix = "",
    visibility = ["//visibility:public"],
)

cc_proto_library(
    name = "native_global_diagnostic_cc_proto",
    deps = [":native_global_diagnostic_proto"],
)

cc_library(
    name = "native_global_worker",
    srcs = ["native_global_worker.cc"],
    hdrs = ["native_global_worker.h"],
    deps = [
        ":native_global_diagnostic_cc_proto",
        ":native_graph_transport",
        ":native_gga_adapter",
        ":native_structured_gnss_adapter",
        ":native_map_matcher",
        ":native_publication",
        "@openloc//:diagnostic_global_alignment",
    ],
)

cc_library(
    name = "native_map_matcher",
    srcs = ["native_map_matcher.cc"],
    hdrs = ["native_map_matcher.h"],
    deps = [
        ":native_global_diagnostic_cc_proto",
        "@openloc//:scan_to_map",
        "@openloc//:diagnostic_regional_search",
        "@eigen",
        "@local_config_pcl//:io",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
    ],
)

cc_test(
    name = "native_map_matcher_test",
    size = "small",
    srcs = [
        "native_map_matcher_test.cc",
        "native_map_test_input.h",
        "native_global_test_input.h",
    ],
    deps = [
        ":native_global_worker",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "native_gga_adapter",
    srcs = ["native_gga_adapter.cc"],
    hdrs = ["native_gga_adapter.h"],
    deps = [
        ":native_global_diagnostic_cc_proto",
        "@openloc//:gnss_position_gate",
        "@proj",
    ],
)

cc_library(
    name = "native_structured_gnss_adapter",
    srcs = ["native_structured_gnss_adapter.cc"],
    hdrs = ["native_structured_gnss_adapter.h"],
    deps = [
        ":native_global_diagnostic_cc_proto",
        "@openloc//:gnss_position_gate",
        "@proj",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:gnss_best_pose_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:heading_cc_proto",
    ],
)

cc_test(
    name = "native_structured_gnss_adapter_test",
    size = "small",
    srcs = ["native_structured_gnss_adapter_test.cc", "native_global_test_input.h"],
    deps = [
        ":native_structured_gnss_adapter",
        ":native_global_worker",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "native_global_cyber",
    srcs = ["native_global_cyber.cc"],
    hdrs = ["native_global_cyber.h"],
    deps = [
        ":native_odometry_diagnostic_cc_proto",
        ":native_global_worker",
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:gnss_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:gnss_best_pose_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:heading_cc_proto",
    ],
)

cc_test(
    name = "native_gga_adapter_test",
    size = "small",
    srcs = ["native_gga_adapter_test.cc", "native_global_test_input.h"],
    deps = [
        ":native_gga_adapter",
        ":native_global_worker",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_binary(
    name = "native_global_diagnostic",
    srcs = ["native_global_diagnostic.cc"],
    deps = [
        ":native_global_cyber",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_test(
    name = "native_global_worker_test",
    size = "small",
    srcs = [
        "native_global_worker_test.cc",
        "native_global_test_input.h",
    ],
    deps = [
        ":native_global_worker",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "native_global_cyber_test",
    size = "medium",
    srcs = [
        "native_global_cyber_test.cc",
        "native_global_test_input.h",
        "native_map_test_input.h",
        "native_graph_test_input.h",
    ],
    args = ["$(location :native_global_diagnostic)"],
    data = [":native_global_diagnostic"],
    deps = [
        ":native_global_cyber",
        "@openloc//:native_odometry",
        "@com_google_protobuf//:protobuf",
        "@com_google_googletest//:gtest",
    ],
)

cc_binary(
    name = "native_global_cyber_replay",
    testonly = True,
    srcs = ["native_global_cyber_replay.cc"],
    deps = [
        ":native_global_cyber",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_test(
    name = "native_lio_config_test",
    size = "small",
    srcs = ["native_lio_config_test.cc"],
    data = ["testdata/native_lio_profile.pb.txt"],
    deps = [
        ":native_lio_config",
        "@com_google_protobuf//:protobuf",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "native_lio_worker_test",
    size = "small",
    srcs = ["native_lio_worker_test.cc"],
    deps = [
        ":native_lio_worker",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "native_publication_test",
    size = "small",
    srcs = ["native_publication_test.cc"],
    deps = [
        ":native_publication",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "native_cyber_publisher_test",
    size = "small",
    srcs = ["native_cyber_publisher_test.cc"],
    deps = [
        ":native_cyber_publisher",
        "@core//cyber",
        "@openloc//:native_odometry",
        "@com_google_googletest//:gtest",
    ],
)

cc_binary(
    name = "native_lio_cyber_replay",
    testonly = True,
    srcs = [
        "native_lio_cyber_replay.cc",
        "native_recorded_cloud.h",
        "native_recorded_input.h",
    ],
    deps = [
        ":native_lio_config",
        ":native_odometry_diagnostic_cc_proto",
        ":native_global_diagnostic_cc_proto",
        "@core//cyber",
        "@com_google_protobuf//:protobuf",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:gnss_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:imu_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
    ],
)

cc_test(
    name = "native_recorded_cloud_test",
    size = "small",
    srcs = [
        "native_recorded_cloud_test.cc",
        "native_recorded_cloud.h",
        "native_recorded_input.h",
    ],
    deps = [
        "@com_google_googletest//:gtest_main",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
    ],
)

cc_binary(
    name = "native_lio_recorded_replay",
    testonly = True,
    srcs = ["native_lio_recorded_replay.cc", "native_recorded_input.h"],
    deps = [
        ":native_lio_config",
        ":native_lio_worker",
        ":native_cyber_publisher",
        "@core//cyber",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_binary(
    name = "native_lio_diagnostic",
    srcs = ["native_lio_diagnostic.cc"],
    deps = [
        ":native_lio_config",
        ":native_lio_worker",
        ":native_source_ingress",
        ":native_cyber_publisher",
        "@core//cyber",
        "@com_google_protobuf//:protobuf",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:imu_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
    ],
)

cc_binary(
    name = "native_imu_diagnostic",
    srcs = ["native_imu_diagnostic.cc"],
    deps = [
        ":native_cyber_publisher",
        "@core//cyber",
        "@openloc//:native_odometry",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:imu_cc_proto",
    ],
)

cc_test(
    name = "native_imu_host_test",
    size = "small",
    srcs = [
        "native_imu_host_test.cc",
        "native_graph_test_input.h",
        "native_map_test_input.h",
        "native_global_test_input.h",
        "native_recorded_input.h",
    ],
    args = [
        "$(location :native_imu_diagnostic)",
        "$(location :native_lio_diagnostic)",
        "$(location testdata/native_lio_profile.pb.txt)",
        "$(location :native_global_diagnostic)",
        "$(location :native_lio_cyber_replay)",
    ],
    data = [
        ":native_imu_diagnostic",
        ":native_lio_diagnostic",
        ":native_global_diagnostic",
        ":native_lio_cyber_replay",
        "testdata/native_lio_profile.pb.txt",
    ],
    deps = [
        ":native_odometry_diagnostic_cc_proto",
        ":native_lio_host_config_cc_proto",
        ":native_global_worker",
        "@core//cyber",
        "@openloc//:native_odometry",
        "@openloc//:native_odometry_types",
        "@com_google_protobuf//:protobuf",
        "@com_google_googletest//:gtest",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:imu_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:gnss_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
    ],
)
