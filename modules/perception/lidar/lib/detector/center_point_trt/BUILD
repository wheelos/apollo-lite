load("@rules_cuda//cuda:defs.bzl", "cuda_library")
load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cuda_library(
    name = "voxel_downsample_cuda",
    srcs = ["voxel_downsample.cu"],
    hdrs = ["voxel_downsample.h"],
    deps = [
        "//modules/perception/base:point",
        "@cuda//:cuda_runtime",
    ],
)

cc_library(
    name = "gpu_down_sample",
    srcs = ["gpu_down_sample.cc"],
    hdrs = ["gpu_down_sample.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        ":voxel_downsample_cuda",
        "@core//cyber",
        "//modules/perception/base",
        "@cuda//:cuda_runtime",
    ],
)

cuda_library(
    name = "center_point_trt_cuda",
    srcs = ["center_point_trt.cu"],
    hdrs = ["center_point_trt_cuda_api.h"],
    deps = [
        "@core//cyber",
        "//modules/perception/base:blob",
        "//modules/perception/base:point_cloud",
        "@cuda//:cuda_runtime",
    ],
)

cc_library(
    name = "center_point_trt",
    srcs = ["center_point_trt.cc"],
    hdrs = ["center_point_trt.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        ":center_point_trt_cuda",
        ":gpu_down_sample",
        "@core//cyber",
        "//modules/perception/base",
        "//modules/perception/inference/onnx_multi_batch:onnx_multi_batch_infer",
        "//modules/perception/lidar/common",
        "//modules/perception/lidar/lib/interface:base_lidar_detector",
        "//modules/perception/lib/config_manager:config_manager",
        "//modules/perception/pipeline/proto/stage:center_point_trt_detection_config_cc_proto",
        "//modules/perception/pipeline:stage",
        "@cuda//:cuda_runtime",
        "@local_config_tensorrt//:tensorrt",
    ],
    alwayslink = True,
)

cpplint()
