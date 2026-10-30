load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    data = [
        ":runtime_data",
    ],
    data_dest = "drivers/addition_data/tools/image_decompress",
    library_dest = "drivers/lib/tools/image_decompress",
    targets = [
        ":libimage_decompress.so",
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

cc_library(
    name = "image_decoder",
    srcs = ["image_decoder.cc"],
    hdrs = ["image_decoder.h"],
    deps = [
        "@core//cyber",
        "@opencv//:imgcodecs",
        "@opencv//:imgproc",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:sensor_image_cc_proto",
    ],
)

cc_library(
    name = "image_decompress_lib",
    srcs = ["image_decompress.cc"],
    hdrs = ["image_decompress.h"],
    copts = ['-DMODULE_NAME=\\"image_decompress\\"'],
    deps = [
        ":image_decoder",
        "//modules/drivers/tools/image_decompress/proto:config_cc_proto",
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:error_code_cc_proto",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:header_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:sensor_image_cc_proto",
    ],
    alwayslink = True,
)

cc_test(
    name = "image_decompress_test",
    size = "small",
    srcs = ["image_decompress_test.cc"],
    deps = [
        ":image_decoder",
        "@com_google_googletest//:gtest_main",
        "@opencv//:imgcodecs",
    ],
)

cc_binary(
    name = "libimage_decompress.so",
    linkshared = True,
    linkstatic = False,
    deps = [":image_decompress_lib"],
)

cpplint()
