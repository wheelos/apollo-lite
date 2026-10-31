load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "point_cloud_conversion",
    srcs = ["point_cloud_conversion.cc"],
    hdrs = ["point_cloud_conversion.h"],
    deps = [
        "@eigen",
        "@local_config_pcl//:filters",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
    ],
)

cc_test(
    name = "point_cloud_conversion_test",
    srcs = ["point_cloud_conversion_test.cc"],
    deps = [
        ":point_cloud_conversion",
        "@wheelos_transform//modules/transform:buffer",
        "@wheelos_transform//modules/transform:buffer_test_peer",
        "@wheelos_transform//modules/transform:ground_reference",
        "@wheelos_transform//modules/transform:static_transform_loader",
        "@wheelos_transform//modules/transform:transform_query",
        "@com_google_googletest//:gtest_main",
        "@eigen",
        "@wheelos_common//modules/common/vehicle_calibration:test_bundle",
    ],
)

cc_library(
    name = "point_cloud_updater",
    srcs = ["point_cloud_updater.cc"],
    hdrs = ["point_cloud_updater.h"],
    deps = [
        ":point_cloud_conversion",
        "//modules/dreamview/backend/common:dreamview_gflags",
        "//modules/dreamview/backend/handlers:websocket_handler",
        "//modules/dreamview/backend/simulation_world:simulation_world_updater",
        "//modules/dreamview/proto:point_cloud_cc_proto",
        "@wheelos_transform//modules/transform:buffer",
        "@wheelos_transform//modules/transform:ground_reference",
        "@wheelos_transform//modules/transform:transform_query",
        "@boost.thread",
        "@com_github_nlohmann_json//:json",
        "@com_google_protobuf//:protobuf",
        "@core//cyber",
        "@local_config_pcl//:filters",
        "@wheelos_common//modules/common/adapters:adapter_gflags",
        "@wheelos_common//modules/common/math",
        "@wheelos_common//modules/common/vehicle_calibration:lidar_height",
        "@wheelos_msgs//wheelos_msgs/localization_msgs:localization_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:sensor_image_cc_proto",
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
