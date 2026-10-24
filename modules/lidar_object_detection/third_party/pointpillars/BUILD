load("@rules_cc//cc:defs.bzl", "cc_library")
load("@rules_cuda//cuda:defs.bzl", "cuda_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cuda_library(
    name = "kernels",
    srcs = [
        "src/pillarScatterKernels.cu",
        "src/postprocess_kernels.cu",
        "src/preprocess_kernels.cu",
    ],
    hdrs = glob(["include/*.h"]),
    includes = ["include"],
    deps = [
        "@cuda//:cuda_runtime",
        "@local_config_tensorrt//:tensorrt",
    ],
    alwayslink = True,
)

cc_library(
    name = "pointpillars",
    srcs = [
        "src/pillarScatter.cpp",
        "src/postprocess.cpp",
    ],
    hdrs = glob(["include/*.h"]),
    includes = ["include"],
    deps = [
        ":kernels",
        "@cuda//:cuda_runtime",
        "@local_config_tensorrt//:tensorrt",
    ],
    alwayslink = True,
)
