load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    data = [
        ":runtime_files",
    ],
    data_dest = "drivers/addition_data/video",
    library_dest = "drivers/lib/video",
    targets = [
        "libvideo_driver_component.so",
    ],
)

filegroup(
    name = "runtime_files",
    srcs = glob([
        "conf/*.txt",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

cc_binary(
    name = "libvideo_driver_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":video_driver_component_lib"],
)

cc_library(
    name = "video_driver_component_lib",
    srcs = ["video_driver_component.cc"],
    hdrs = ["video_driver_component.h"],
    copts = ['-DMODULE_NAME=\\"video\\"'],
    deps = [
        "//modules/drivers/video:driver",
        "//modules/drivers/video:socket",
        "@core//cyber",
        "@wheelos_common//modules/common/util:util_tool",
    ],
    alwayslink = True,
)

cc_library(
    name = "driver",
    srcs = ["driver.cc"],
    hdrs = [
        "driver.h",
    ],
    deps = [
        ":socket",
        "//modules/drivers/video/proto:video_h265cfg_cc_proto",
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:sensor_image_cc_proto",
    ],
)

cc_library(
    name = "socket",
    srcs = ["socket_input.cc"],
    hdrs = [
        "socket_input.h",
    ],
    deps = [
        ":frame_assembler",
        "@core//cyber",
        "@wheelos_common//modules/common/util",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:error_code_cc_proto",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:header_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:sensor_image_cc_proto",
    ],
)

cc_library(
    name = "frame_assembler",
    srcs = ["frame_assembler.cc"],
    hdrs = [
        "frame_assembler.h",
        "input.h",
    ],
    deps = [
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:sensor_image_cc_proto",
    ],
)

cc_test(
    name = "frame_assembler_test",
    size = "small",
    srcs = ["frame_assembler_test.cc"],
    deps = [
        ":frame_assembler",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
