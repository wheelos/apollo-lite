load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("@rules_cuda//cuda:defs.bzl", "cuda_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "perception_inference_tensorrt_plugins",
    hdrs = glob(["*.h"]),
    deps = [
        ":dfmb_psroi_align_plugin",
        ":perception_inference_tensorrt_plugins_argmax_cuda",
        ":perception_inference_tensorrt_plugins_leakyrelu_cuda",
        ":perception_inference_tensorrt_plugins_slice_cuda",
        ":perception_inference_tensorrt_plugins_softmax_cuda",
        ":rcnn_proposal_plugin",
        ":rpn_proposal_ssd_plugin",
        "@cuda//:cublas",
        "@cuda//:cuda_runtime",
    ],
    alwayslink = True,
)

cuda_library(
    name = "kernels_cuda",
    srcs = [
        "kernels.cu",
        "nms_cuda.cu",
    ],
    hdrs = ["kernels.h"],
    # copts = ["-DUSE_GPU=1"],
    deps = [
        "//modules/perception/base:common",
        "@cuda//:cuda_runtime",
    ],
    alwayslink = True,
)

cuda_library(
    name = "dfmb_psroi_align_plugin",
    srcs = ["dfmb_psroi_align_plugin.cu"],
    hdrs = ["dfmb_psroi_align_plugin.h"],
    deps = [
        ":kernels_cuda",
        "@core//cyber",
        "//modules/perception/inference/tensorrt:rt_common_without_protobuf",
        "@cuda//:cuda_runtime",
        "@local_config_tensorrt//:tensorrt",
    ],
)

cuda_library(
    name = "rcnn_proposal_plugin",
    srcs = ["rcnn_proposal_plugin.cu"],
    hdrs = ["rcnn_proposal_plugin.h"],
    deps = [
        ":kernels_cuda",
        "//modules/perception/inference/tensorrt:rt_common_without_protobuf",
        "@cuda//:cuda_runtime",
        "@local_config_tensorrt//:tensorrt",
    ],
)

cuda_library(
    name = "rpn_proposal_ssd_plugin",
    srcs = ["rpn_proposal_ssd_plugin.cu"],
    hdrs = ["rpn_proposal_ssd_plugin.h"],
    deps = [
        ":kernels_cuda",
        "//modules/perception/inference/tensorrt:rt_common_without_protobuf",
        "@cuda//:cuda_runtime",
        "@local_config_tensorrt//:tensorrt",
    ],
)

cuda_library(
    name = "perception_inference_tensorrt_plugins_slice_cuda",
    srcs = ["slice_plugin.cu"],
    hdrs = ["slice_plugin.h"],
    deps = [
        "//modules/perception/inference/tensorrt:rt_common_without_protobuf",
        "@cuda//:cuda_runtime",
        "@local_config_tensorrt//:tensorrt",
    ],
)

cuda_library(
    name = "perception_inference_tensorrt_plugins_argmax_cuda",
    srcs = ["argmax_plugin.cu"],
    hdrs = ["argmax_plugin.h"],
    deps = [
        "//modules/perception/inference/tensorrt:rt_common_without_protobuf",
        "@cuda//:cuda_runtime",
        "@local_config_tensorrt//:tensorrt",
    ],
)

cuda_library(
    name = "perception_inference_tensorrt_plugins_leakyrelu_cuda",
    srcs = ["leakyReLU_plugin.cu"],
    hdrs = ["leakyReLU_plugin.h"],
    deps = [
        "//modules/perception/inference/tensorrt:rt_common_without_protobuf",
        "@cuda//:cuda_runtime",
        "@local_config_tensorrt//:tensorrt",
    ],
)

cuda_library(
    name = "perception_inference_tensorrt_plugins_softmax_cuda",
    srcs = ["softmax_plugin.cu"],
    hdrs = ["softmax_plugin.h"],
    deps = [
        "//modules/perception/inference/tensorrt:rt_common_without_protobuf",
        "@cuda//:cublas",
        "@cuda//:cuda_runtime",
        "@local_config_cudnn//:cudnn",
        "@local_config_tensorrt//:tensorrt",
    ],
)

cc_test(
    name = "slice_plugin_test",
    size = "small",
    srcs = ["slice_plugin_test.cc"],
    linkstatic = True,
    deps = [
        ":perception_inference_tensorrt_plugins",
        "//modules/perception/inference/tensorrt:rt_common",
        "//modules/perception/inference/tensorrt:rt_net",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "argmax_plugin_test",
    size = "small",
    srcs = ["argmax_plugin_test.cc"],
    linkstatic = True,
    deps = [
        ":perception_inference_tensorrt_plugins",
        "//modules/perception/inference/tensorrt:rt_common",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
