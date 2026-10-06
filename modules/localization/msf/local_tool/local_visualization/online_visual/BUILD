load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "online_visualizer_component_lib",
    srcs = ["online_visualizer_component.cc"],
    hdrs = ["online_visualizer_component.h"],
    deps = [
        "@wheelos_common//modules/common/adapters:adapter_gflags",
        "@wheelos_common//modules/common/math",
        "@wheelos_common//modules/common/map:map_selection",
        "@wheelos_common//modules/common/status",
        "@wheelos_common//modules/common/util",
        "@wheelos_common//modules/common/vehicle_calibration:registry",
        "//modules/localization/common:localization_gflags",
        "//modules/localization/msf/common/io:common_io",
        "//modules/localization/msf/local_pyramid_map/base_map",
        "//modules/localization/msf/local_tool/local_visualization/engine",
        "//modules/localization/proto:localization_config_cc_proto",
        "@com_google_googletest//:gtest_main",
        "@core//cyber",
        "@local_config_pcl//:common",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:geometry_cc_proto",
        "@wheelos_msgs//wheelos_msgs/localization_msgs:localization_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_cc_proto",
    ],
    alwayslink = True,
)

cc_binary(
    name = "online_visualizer_compenont.so",
    linkshared = True,
    linkstatic = False,
    deps = [":online_visualizer_component_lib"],
)

install(
    name = "install",
    library_dest = "localization/lib/msf/local_tool/local_visualization/online_visual",
    targets = [
        ":online_visualizer_compenont.so",
    ],
)

cpplint()
