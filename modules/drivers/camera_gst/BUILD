load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CAMERA_GST_COPTS = ['-DMODULE_NAME=\\"camera_gst\\"']

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

install(
    name = "install",
    data = [
        ":runtime_data",
    ],
    data_dest = "drivers/addition_data/camera_gst",
    library_dest = "drivers/lib/camera_gst",
    targets = [
        ":libcamera_gst_component.so",
    ],
)

cc_library(
    name = "camera_gst_core",
    srcs = [
        "camera_gst_driver.cc",
        "frame_source.cc",
        "frame_stitcher.cc",
        "streamer.cc",
    ],
    hdrs = [
        "camera_gst_driver.h",
        "frame_source.h",
        "frame_stitcher.h",
        "streamer.h",
    ],
    copts = CAMERA_GST_COPTS,
    deps = [
        "//cyber",
        "//modules/drivers/camera_gst/proto:config_cc_proto",
        "@gstreamer//:gstreamer",
        "@opencv//:core",
        "@opencv//:imgproc",
        "@opencv//:videoio",
    ],
)

cc_library(
    name = "camera_gst_component_lib",
    srcs = ["camera_gst_component.cc"],
    hdrs = ["camera_gst_component.h"],
    copts = CAMERA_GST_COPTS,
    deps = [
        ":camera_gst_core",
        "//cyber",
        "//modules/common/util:rate_limiter",
        "//modules/common_msgs/sensor_msgs:sensor_image_cc_proto",
        "@gstreamer//:gstreamer",
    ],
    alwayslink = True,
)

cc_binary(
    name = "libcamera_gst_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":camera_gst_component_lib"],
)

cc_test(
    name = "frame_stitcher_test",
    size = "small",
    srcs = ["frame_stitcher_test.cc"],
    deps = [
        ":camera_gst_core",
        "//modules/planning/common:gtest_exit_main",
    ],
)

cc_test(
    name = "streamer_test",
    size = "small",
    srcs = ["streamer_test.cc"],
    deps = [
        ":camera_gst_core",
        "//modules/planning/common:gtest_exit_main",
    ],
)

cpplint()
