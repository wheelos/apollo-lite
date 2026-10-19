load("@com_google_protobuf//bazel:cc_proto_library.bzl", "cc_proto_library")
load("@com_google_protobuf//bazel:proto_library.bzl", "proto_library")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "onnx_proto",
    srcs = [
        "onnx/onnx-ml.proto",
        "onnx/onnx-operators-ml.proto",
    ],
    strip_import_prefix = "onnx",
)

cc_proto_library(
    name = "onnx_proto_cc",
    deps = [":onnx_proto"],
)

cc_library(
    name = "spconv_headers",
    hdrs = glob(["include/spconv/**/*.hpp"]),
    includes = ["include"],
    deps = ["@cuda//:cuda_runtime"],
)

cc_library(
    name = "spconv_onnx_parser",
    srcs = ["parser/onnx-parser.cpp"],
    hdrs = ["parser/onnx-parser.hpp"],
    includes = ["parser"],
    deps = [
        ":onnx_proto_cc",
        ":spconv_headers",
        "@com_google_protobuf//:protobuf",
        "@cuda//:cuda_runtime",
        "@local_config_spconv//:spconv",
        "@local_config_tensorrt//:tensorrt",
    ],
)

exports_files(["extensions.bzl"])
