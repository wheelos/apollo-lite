load("@com_google_protobuf//bazel:cc_proto_library.bzl", "cc_proto_library")
load("@com_google_protobuf//bazel:proto_library.bzl", "proto_library")
load("@com_google_protobuf//bazel:py_proto_library.bzl", "py_proto_library")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "rtk_config_cc_proto",
    deps = [":rtk_config_proto"],
)

proto_library(
    name = "rtk_config_proto",
    srcs = ["rtk_config.proto"],
)

py_proto_library(
    name = "rtk_config_py_pb2",
    deps = [":rtk_config_proto"],
)
