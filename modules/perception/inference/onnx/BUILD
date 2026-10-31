load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "onnx_obstacle_detector",
    srcs = ["onnx_obstacle_detector.cc"],
    hdrs = [
        "onnx_obstacle_detector.h",
    ],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
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
        "//modules/perception/common:perception_gflags",
        "//modules/perception/inference:inference_lib",
        "@core//cyber",
        "@libtorch_gpu",
    ],
)

filegroup(
    name = "exported_source_files",
    srcs = glob(
        ["**/*"],
        allow_empty = True,
    ),
    visibility = ["//visibility:public"],
)

filegroup(
    name = "exported_source_headers",
    srcs = glob(
        ["**/*.h"],
        allow_empty = True,
    ),
    visibility = ["//visibility:public"],
)
