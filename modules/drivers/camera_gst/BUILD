load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

CAMERA_GST_COPTS = ['-DMODULE_NAME=\\"camera_gst\\"']

cc_library(
    name = "camera_gst_frame_extractor",
    srcs = ["frame_extractor.cc"],
    hdrs = [
        "frame_extractor.h",
        "frame_types.h",
    ],
    copts = CAMERA_GST_COPTS,
    deps = [
        "@core//cyber",
        "@gstreamer",
    ],
)

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
    target_compatible_with = ["//tools/platform:jetson_orin"],
    targets = [
        ":libcamera_gst_component.so",
    ],
)

cc_library(
    name = "camera_gst_core",
    srcs = [
        "camera_gst_driver.cc",
        "pipeline_builder.cc",
        "streamer.cc",
    ],
    hdrs = [
        "camera_gst_driver.h",
        "frame_types.h",
        "pipeline_builder.h",
        "streamer.h",
    ],
    copts = CAMERA_GST_COPTS,
    target_compatible_with = ["//tools/platform:jetson_orin"],
    deps = [
        ":camera_gst_frame_extractor",
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
        "@core//cyber",
    ],
)

cc_test(
    name = "frame_extractor_test",
    size = "small",
    srcs = ["frame_extractor_test.cc"],
    copts = CAMERA_GST_COPTS,
    deps = [
        ":camera_gst_frame_extractor",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
        "@core//cyber",
        "@gstreamer",
    ],
)

cpplint()
