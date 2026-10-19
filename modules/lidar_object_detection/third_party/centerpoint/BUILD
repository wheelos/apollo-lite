load("@rules_cuda//cuda:defs.bzl", "cuda_library")
package(default_visibility = ["//visibility:public"])

licenses(["notice"])

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
    hdrs = glob([
        "include/*.h",
        "include/*.hpp",
    ]),
    copts = ["-std=c++17"],
    includes = [
        "include",
        "src",
    ],
    deps = [
        "//third_party/spconv:spconv_headers",
        "//third_party/spconv:spconv_onnx_parser",
        "@com_google_protobuf//:protobuf",
        "@cuda//:cuda_runtime",
        "@local_config_tensorrt//:tensorrt",
    ],
    alwayslink = True,
)
