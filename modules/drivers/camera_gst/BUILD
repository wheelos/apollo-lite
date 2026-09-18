load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

CAMERA_GST_COPTS = ['-DMODULE_NAME=\\"camera_gst\\"']

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/**/*.txt",
        "dag/**/*.dag",
        "launch/**/*.launch",
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
        "frame_extractor.cc",
        "pipeline_builder.cc",
        "streamer.cc",
    ],
    hdrs = [
        "camera_gst_driver.h",
        "frame_extractor.h",
        "frame_types.h",
        "pipeline_builder.h",
        "streamer.h",
    ],
    copts = CAMERA_GST_COPTS,
    target_compatible_with = ["//tools/platform:jetson_orin"],
    deps = [
        "//modules/drivers/camera_gst/proto:config_cc_proto",
        "@core//cyber",
        "@gstreamer",
    ],
)

cc_library(
    name = "camera_gst_component_lib",
    srcs = ["camera_gst_component.cc"],
    hdrs = ["camera_gst_component.h"],
    copts = CAMERA_GST_COPTS,
    target_compatible_with = ["//tools/platform:jetson_orin"],
    deps = [
        ":camera_gst_core",
        "@core//cyber",
        "@gstreamer",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:sensor_image_cc_proto",
    ],
    alwayslink = True,
)

cc_binary(
    name = "libcamera_gst_component.so",
    linkshared = True,
    linkstatic = False,
    target_compatible_with = ["//tools/platform:jetson_orin"],
    deps = [":camera_gst_component_lib"],
)

cc_test(
    name = "streamer_test",
    size = "small",
    srcs = ["streamer_test.cc"],
    copts = CAMERA_GST_COPTS,
    target_compatible_with = ["//tools/platform:jetson_orin"],
    deps = [
        ":camera_gst_core",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
