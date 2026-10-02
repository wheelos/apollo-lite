load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "messages",
    srcs = ["messages.cc"],
    hdrs = ["messages.h"],
    deps = [
        "//modules/localization/core:global_types",
        "//modules/localization/core:local_motion",
        "//modules/localization/core:types",
        "//modules/localization/proto:unified_localization_cc_proto",
        "//modules/localization_health/proto:localization_health_cc_proto",
        "@eigen",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:geometry_cc_proto",
        "@wheelos_msgs//wheelos_msgs/localization_msgs:localization_cc_proto",
    ],
)

cc_library(
    name = "health_bridge",
    srcs = ["health_bridge.cc"],
    hdrs = ["health_bridge.h"],
    deps = [
        ":messages",
        "//modules/localization/common:rigid_transform_helper",
        "//modules/localization/core:lane_relative",
        "//modules/localization_health/proto:localization_health_cc_proto",
        "@core//cyber",
        "@eigen",
        "@wheelos_msgs//wheelos_msgs/perception_msgs:perception_lane_cc_proto",
    ],
    alwayslink = True,
)

cc_library(
    name = "gnss_adapter",
    srcs = ["gnss_adapter.cc"],
    hdrs = ["gnss_adapter.h"],
    deps = [
        ":messages",
        "//modules/localization/core:global_alignment",
        "@eigen",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:gnss_best_pose_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:heading_cc_proto",
    ],
)

cc_library(
    name = "local_component",
    srcs = ["local_component.cc"],
    hdrs = ["local_component.h"],
    deps = [
        ":messages",
        "//modules/localization/common:rigid_transform_helper",
        "//modules/localization/core:local_estimator",
        "//modules/transform:transform_broadcaster",
        "@core//cyber",
        "@eigen",
        "@wheelos_msgs//wheelos_msgs/chassis_msgs:chassis_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:imu_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
    ],
    alwayslink = True,
)

cc_library(
    name = "map_resources",
    srcs = ["map_resources.cc"],
    hdrs = ["map_resources.h"],
    deps = [
        ":messages",
        "//modules/localization/map:map_matcher",
    ],
)

cc_library(
    name = "global_component",
    srcs = ["global_component.cc"],
    hdrs = ["global_component.h"],
    deps = [
        ":gnss_adapter",
        ":map_resources",
        ":messages",
        "//modules/localization/common:rigid_transform_helper",
        "//modules/localization/core:global_alignment",
        "//modules/localization/core:observed_subspace",
        "//modules/localization/map:map_matcher",
        "//modules/transform:transform_broadcaster",
        "@core//cyber",
        "@eigen",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:gnss_best_pose_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:heading_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
    ],
    alwayslink = True,
)

cc_binary(
    name = "liblocal_localization_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":local_component"],
)

cc_binary(
    name = "libglobal_localization_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":global_component"],
)

cc_binary(
    name = "liblocalization_health_bridge.so",
    linkshared = True,
    linkstatic = False,
    deps = [":health_bridge"],
)

install(
    name = "install",
    library_dest = "localization/lib",
    targets = [
        ":libglobal_localization_component.so",
        ":liblocal_localization_component.so",
        ":liblocalization_health_bridge.so",
    ],
)

cc_test(
    name = "map_resources_test",
    size = "small",
    srcs = ["map_resources_test.cc"],
    deps = [":map_resources", "@com_google_googletest//:gtest_main"],
)

cc_test(
    name = "messages_test",
    size = "small",
    srcs = ["messages_test.cc"],
    deps = [":messages", "@com_google_googletest//:gtest_main"],
)

cc_test(
    name = "gnss_adapter_test",
    size = "small",
    srcs = ["gnss_adapter_test.cc"],
    deps = [":gnss_adapter", "@com_google_googletest//:gtest_main"],
)

cc_test(
    name = "localization_pipeline_test",
    size = "small",
    srcs = ["localization_pipeline_test.cc"],
    deps = [
        ":messages",
        "//modules/localization/core:global_alignment",
        "//modules/localization/core:local_estimator",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
