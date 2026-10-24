load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("@rules_cuda//cuda:defs.bzl", "cuda_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "sparse_backbone_api",
    hdrs = ["include/sparse_backbone.h"],
    includes = ["include"],
    deps = ["@cuda//:cuda_runtime"],
)

cc_library(
    name = "sparse_model",
    srcs = ["src/sparse_model.cpp"],
    hdrs = ["include/sparse_model.h"],
    includes = ["include"],
    deps = [
        "//third_party/onnx:onnx_proto_cc",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_test(
    name = "sparse_model_test",
    size = "small",
    srcs = ["src/sparse_model_test.cpp"],
    deps = [
        ":sparse_model",
        "//third_party/onnx:onnx_proto_cc",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_binary(
    name = "sparse_backbone_benchmark",
    testonly = True,
    srcs = ["src/sparse_backbone_benchmark.cpp"],
    deps = [
        ":sparse_backbone_api",
        ":spconv_backbone",
    ],
)

cuda_library(
    name = "spconv_backbone",
    srcs = ["src/spconv_backbone.cu"],
    copts = [
        "-std=c++17",
        "-O3",
        "--extended-lambda",
    ],
    deps = [
        ":sparse_backbone_api",
        ":sparse_model",
        "@spconv",
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
        ":sparse_backbone_api",
        ":spconv_backbone",
        "@cuda//:cuda_runtime",
        "@local_config_tensorrt//:tensorrt",
    ],
    alwayslink = True,
)
