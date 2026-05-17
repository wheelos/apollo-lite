load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "onnx_obstacle_detector",
    srcs = ["onnx_obstacle_detector.cc"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    hdrs = [
        "onnx_obstacle_detector.h",
    ],
    deps = [
        "//modules/perception/common:perception_gflags",
        "//modules/perception/inference:inference_lib",
        "@cuda//:cuda_runtime",
        "@local_config_tensorrt//:tensorrt",
    ],
)

cc_library(
    name = "libtorch_obstacle_detector",
    srcs = ["libtorch_obstacle_detector.cc"],
    hdrs = [
        "libtorch_obstacle_detector.h",
    ],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        "//cyber",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/inference:inference_lib",
        "@libtorch_gpu",
    ],
)

cpplint()
