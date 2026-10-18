load("@rules_cuda//cuda:defs.bzl", "cuda_library")
load("@com_google_protobuf//bazel:cc_proto_library.bzl", "cc_proto_library")
load("@com_google_protobuf//bazel:proto_library.bzl", "proto_library")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "onnx_proto",
    srcs = [
        "src/onnx/onnx-ml.proto",
        "src/onnx/onnx-operators-ml.proto",
    ],
    strip_import_prefix = "src/onnx",
)

cc_proto_library(
    name = "onnx_proto_cc",
    deps = [":onnx_proto"],
)

cc_library(
    name = "runtime_api",
    hdrs = ["include/centerpoint_runtime_api.h"],
)

cuda_library(
    name = "runtime_cuda_kernels",
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
    name = "runtime_core",
    srcs = [
        "3rdparty/libspconv/parser/onnx-parser.cpp",
        "src/centerpoint.cpp",
        "src/centerpoint_runtime_api.cpp",
        "src/postprocess.cpp",
        "src/preprocess.cpp",
        "src/tensorrt.cpp",
        ":runtime_cuda_kernels",
    ],
    hdrs = glob([
        "3rdparty/libspconv/include/**/*.hpp",
        "3rdparty/libspconv/parser/**/*.hpp",
        "include/*.h",
        "include/*.hpp",
    ]),
    copts = ["-std=c++17"],
    includes = [
        "3rdparty/libspconv/include",
        "3rdparty/libspconv/parser",
        "include",
        "src",
    ],
    deps = [
        ":onnx_proto_cc",
        ":runtime_api",
        "@com_google_protobuf//:protobuf",
        "@cuda//:cuda_runtime",
        "@local_config_spconv//:spconv",
        "@local_config_tensorrt//:tensorrt",
    ],
    alwayslink = True,
)
