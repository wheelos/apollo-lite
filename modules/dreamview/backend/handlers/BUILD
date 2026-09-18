load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

DREAMVIEW_COPTS = ['-DMODULE_NAME=\\"dreamview\\"']

cc_library(
    name = "websocket_handler",
    srcs = ["websocket_handler.cc"],
    hdrs = ["websocket_handler.h"],
    copts = DREAMVIEW_COPTS,
    deps = [
        "//modules/common/util:util_tool",
        "@civetweb",
        "@civetweb//:civetserver",
        "@com_github_nlohmann_json//:json",
        "@core//cyber",
    ],
)

cc_library(
    name = "map_data_handler",
    srcs = ["map_data_handler.cc"],
    hdrs = ["map_data_handler.h"],
    copts = DREAMVIEW_COPTS,
    deps = [
        "@civetweb",
        "@civetweb//:civetserver",
    ],
)

cc_library(
    name = "image_handler",
    srcs = ["image_handler.cc"],
    hdrs = ["image_handler.h"],
    copts = DREAMVIEW_COPTS,
    deps = [
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/configs:config_gflags",
        "@civetweb//:civetserver",
        "@core//cyber",
        "@opencv//:imgcodecs",
        "@opencv//:imgproc",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:sensor_image_cc_proto",
    ],
)

cc_test(
    name = "websocket_handler_test",
    size = "small",
    srcs = ["websocket_handler_test.cc"],
    deps = [
        ":websocket_handler",
        "@civetweb//:civetserver",
        "@com_google_googletest//:gtest_main",
        "@core//cyber",
    ],
)

cpplint()
