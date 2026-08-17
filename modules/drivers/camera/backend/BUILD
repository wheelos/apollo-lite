load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CAMERA_COPTS = ['-DMODULE_NAME=\\"camera\\"']


cc_library(
    name = "v4l2_device",
    srcs = ["v4l2_device.cc"],
    hdrs = ["v4l2_device.h"],
    copts = CAMERA_COPTS,
    deps = [
        "@core//cyber",
        "//modules/drivers/camera/proto:config_cc_proto",
    ]
)

cc_library(
    name = "image_processor",
    srcs = ["image_processor.cc"],
    hdrs = ["image_processor.h"],
    copts = CAMERA_COPTS,
    deps = [
        "@core//cyber",
        "//modules/drivers/camera/proto:config_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:sensor_image_cc_proto",
        "@ffmpeg//:avcodec",
        "@ffmpeg//:avformat",
        "@ffmpeg//:avutil",
        "@ffmpeg//:swscale",
        "@opencv//:imgcodecs",
    ]
)

cc_library(
    name = "camera_device",
    srcs = ["camera_device.cc"],
    hdrs = ["camera_device.h"],
    copts = CAMERA_COPTS,
    deps = [
        ":image_processor",
        ":v4l2_device",
        "@core//cyber",
        "//modules/drivers/camera/proto:config_cc_proto",
    ]
)

cpplint()
