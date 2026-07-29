load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")
load("//tools/platform:build_defs.bzl", "if_x86_64")

package(default_visibility = ["//visibility:public"])

CAMERA_COPTS = ['-DMODULE_NAME=\\"camera\\"']

cc_binary(
    name = "libcamera_component.so",
    linkopts = [
        "-latomic",
    ],
    linkshared = True,
    linkstatic = True,
    deps = [
        ":camera_component_lib",
        ":compress_component_lib",
        # ":compress_hardware_component_lib",
        "@ffmpeg//:avcodec",
        "@ffmpeg//:avformat",
        "@ffmpeg//:swscale",
    ],
)

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
    data_dest = "drivers/addition_data/camera",
    library_dest = "drivers/lib/camera",
    targets = [
        ":libcamera_component.so",
    ],
)

cc_library(
    name = "camera_component_lib",
    srcs = ["camera_component.cc"],
    hdrs = ["camera_component.h"],
    copts = CAMERA_COPTS,
    deps = [
        "@core//cyber",
        "//modules/common/util:rate_limiter",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:error_code_cc_proto",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:header_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:sensor_image_cc_proto",
        "//modules/drivers/camera/backend:camera_device",
    ],
    alwayslink = True,
)

cc_library(
    name = "compress_component_lib",
    srcs = ["compress_component.cc"],
    hdrs = ["compress_component.h"],
    copts = CAMERA_COPTS,
    deps = [
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:error_code_cc_proto",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:header_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:sensor_image_cc_proto",
        "//modules/drivers/camera/proto:config_cc_proto",
        "@opencv//:imgcodecs",
        "@opencv//:imgproc",
    ],
    alwayslink = True,
)

# cc_library(
#     name = "compress_hardware_component_lib",
#     srcs = ["compress_hardware_component.cc"],
#     hdrs = ["compress_hardware_component.h"],
#     copts = CAMERA_COPTS,
#     alwayslink = True,
#     deps = [
#         "@core//cyber",
#         "@wheelos_msgs//wheelos_msgs/basic_msgs:error_code_cc_proto",
#         "@wheelos_msgs//wheelos_msgs/basic_msgs:header_cc_proto",
#         "//modules/drivers/camera/proto:config_cc_proto",
#         "@wheelos_msgs//wheelos_msgs/sensor_msgs:sensor_image_cc_proto",
#         "@opencv//:imgcodecs",
#         "@opencv//:imgproc",
#         "@local_config_cuda//cuda:cuda_headers",
#         "@local_config_cuda//cuda:cudart",
#     ],
#     linkopts = [
#         "-L/usr/local/cuda/targets/x86_64-linux/lib/",
#         "-L/usr/lib/aarch64-linux-gnu/tegra/",
#         "-lnvjpeg",
#         "-lcuda",
#     ],
# )

cpplint()
