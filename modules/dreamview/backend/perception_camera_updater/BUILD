load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_camera_updater",
    srcs = ["perception_camera_updater.cc"],
    hdrs = ["perception_camera_updater.h"],
    copts = ['-DMODULE_NAME=\\"dreamview\\"'],
    alwayslink = True,
    deps = [
        "@core//cyber",
        "//modules/common/adapters:adapter_gflags",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:geometry_cc_proto",
        "//modules/dreamview/backend/handlers:websocket_handler",
        "//modules/dreamview/proto:camera_update_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:sensor_image_cc_proto",
        "@wheelos_msgs//wheelos_msgs/localization_msgs:localization_cc_proto",
        "//modules/transform:transform_query",
        "@eigen",
        "@opencv//:imgcodecs",
        "@opencv//:imgproc",
    ],
)

cc_test(
    name = "perception_camera_updater_test",
    size = "small",
    srcs = ["perception_camera_updater_test.cc"],
    linkstatic = True,
    deps = [
        ":perception_camera_updater",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
