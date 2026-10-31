load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("@rules_cuda//cuda:defs.bzl", "cuda_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_binary(
    name = "sparse_backbone_benchmark",
    testonly = True,
    srcs = ["src/sparse_backbone_benchmark.cpp"],
    deps = [
        "@wheelos_common//perception/model_io/onnx:sparse_model",
        "@wheelos_common//perception/ops/spconv:spconv_backbone",
    ],
)

cuda_library(
    name = "centerpoint_cuda_kernels",
    srcs = [
        "src/postprocess_kernels.cu",
        "src/preprocess_kernels.cu",
    ],
    hdrs = [
        "include/common.h",
        "include/kernel.h",
    ],
    copts = ["-std=c++17"],
    includes = ["include"],
    deps = [
        "@cuda//:cuda_runtime",
    ],
    alwayslink = True,
)

cc_library(
    name = "centerpoint",
    srcs = [
        "src/centerpoint.cpp",
        "src/postprocess.cpp",
        "src/preprocess.cpp",
        "src/tensorrt.cpp",
        ":centerpoint_cuda_kernels",
    ],
    hdrs = [
        "include/centerpoint.h",
        "include/common.h",
        "include/kernel.h",
        "include/postprocess.h",
        "include/preprocess.h",
        "include/tensorrt.hpp",
        "include/timer.hpp",
    ],
    copts = ["-std=c++17"],
    includes = [
        "include",
        "src",
    ],
    deps = [
        "@cuda//:cuda_runtime",
        "@local_config_tensorrt//:tensorrt",
        "@wheelos_common//perception/model_io/onnx:sparse_model",
        "@wheelos_common//perception/ops/spconv:spconv_backbone",
    ],
    alwayslink = True,
)
