load("@com_google_protobuf//bazel:cc_proto_library.bzl", "cc_proto_library")
load("@com_google_protobuf//bazel:proto_library.bzl", "proto_library")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "onnx_proto",
    srcs = [
        "onnx-ml.proto",
        "onnx-operators-ml.proto",
    ],
    strip_import_prefix = "/third_party/onnx",
)

cc_proto_library(
    name = "onnx_proto_cc",
    deps = [":onnx_proto"],
)
